#pragma once

#include "i_servo_drive.h"
#include "joint_unit_converter.h"
#include <memory>
#include <string>
#include <atomic>

class CiA402Motor;

/**
 * @brief 单轴错误码
 */
enum class AxisError : uint8_t {
    OK = 0,
    MotorNotEnabled,      ///< 电机未使能，无法执行运动命令
    PositionOutOfLimit,   ///< 目标位置超软限位
    VelocityOutOfLimit,   ///< 目标速度超限
    InvalidArgument,      ///< 参数非法（非有限值等）
    ConversionOverflow,   ///< 单位转换时整数溢出
    HomingNotComplete,    ///< 回零未完成（仅 requireHomingComplete 打开时）
    TorqueOutOfLimit,     ///< 目标转矩超限（超出 -100% ~ 100%）
    ModeNotSupported      ///< 该运行模式不被当前驱动器支持
};

/**
 * @class Axis
 * @brief 单轴封装（工业级，实时安全）
 *
 * 组合 IServoDrive（默认 CiA402Motor，亦可 MockServoDrive）和 JointUnitConverter，对外提供 SI 单位接口。
 *
 * ## RT / NRT 契约
 * - **RT**：`realtimeCycle`、`setPositionRT`、状态/位置查询、`followingError`
 * - **NRT**：`setConverterConfig`、`getModeName`/`lastError`（字符串）、构造
 * - **命令投递**（NRT 生产 / RT 消费）：`enable`/`moveAbsolute`/…（有界环）
 */
class Axis {
public:
    /**
     * @brief 构造函数
     * @param name 轴名称
     * @param motor 独占的伺服对象（`CiA402Motor` / `MockServoDrive` 均可）
     * @param converter 单位转换器（拷贝）
     */
    Axis(const std::string& name,
         std::unique_ptr<IServoDrive> motor,
         const JointUnitConverter& converter);

    // 禁止拷贝，允许移动
    Axis(const Axis&) = delete;
    Axis& operator=(const Axis&) = delete;
    Axis(Axis&&) noexcept = default;
    Axis& operator=(Axis&&) noexcept = default;

    /** @brief 轴名称 */
    const std::string& name() const noexcept { return name_; }

    /** @brief 更新单位转换器配置（运行时调用，非实时安全） */
    void setConverterConfig(const JointAxisConfig& config);

    // ===================== 实时周期入口 =====================
    /**
     * @brief 实时周期任务（必须在每个 EtherCAT 周期内调用）
     * @param max_commands 最多处理的异步命令数
     */
    void realtimeCycle(std::size_t max_commands = 8) noexcept;

    // ===================== 基本控制 =====================
    /** @brief 使能轴（异步命令） */
    void enable() noexcept;
    /** @brief 禁能轴（异步命令） */
    void disable() noexcept;
    /** @brief 清除故障（异步命令） */
    void clearFault() noexcept;

    // ===================== 运动命令（异步，SI 单位） =====================
    /**
     * @brief 绝对位置运动（CSP 模式）
     * @param rad 目标关节角 [rad]
     * @return 错误码，OK 表示命令已入队
     */
    AxisError moveAbsolute(double rad) noexcept;

    /**
     * @brief 相对位置运动（基于当前位置）
     * @param delta_rad 位移 [rad]
     * @return 错误码
     */
    AxisError moveRelative(double delta_rad) noexcept;

    /**
     * @brief 速度模式运动（CSV 模式）
     * @param rad_per_s 目标速度 [rad/s]
     * @return 错误码
     */
    AxisError moveVelocity(double rad_per_s) noexcept;

    /**
     * @brief 正向点动
     * @param rad_per_s 点动速度 [rad/s]，自动取绝对值
     * @return 错误码
     */
    AxisError jogPositive(double rad_per_s) noexcept;

    /**
     * @brief 反向点动
     * @param rad_per_s 点动速度 [rad/s]，自动取绝对值
     * @return 错误码
     */
    AxisError jogNegative(double rad_per_s) noexcept;

    /** @brief 停止点动（速度指令置零） */
    void jogStop() noexcept;

    /** @brief 普通暂停（置位 Halt） */
    void stop() noexcept;
    /** @brief 清除暂停 */
    void clearStop() noexcept;
    /** @brief 快停 */
    void quickStop() noexcept;
    /** @brief 清除快停 */
    void clearQuickStop() noexcept;

    /** @brief 启动回零（Homing 模式） */
    void startHoming() noexcept;

    /** @brief 设置回零超时（控制周期数，0=不超时） */
    void setHomingTimeoutCycles(uint32_t cycles) noexcept;

    /**
     * @brief 未回原完成时是否拒绝运动命令（默认 false，不挡住 rcat-tool 等无回原流程）
     */
    void setRequireHomingComplete(bool require) noexcept {
        require_homing_complete_.store(require, std::memory_order_relaxed);
    }
    bool requireHomingComplete() const noexcept {
        return require_homing_complete_.load(std::memory_order_relaxed);
    }

    /** NRT：写 0x6098 / 0x6099 / 0x609A / 0x607C */
    bool setHomingMethod(int8_t method);
    bool setHomingSpeeds(uint32_t switch_search, uint32_t zero_search);
    bool setHomingAcceleration(uint32_t acceleration);
    bool setHomeOffset(int32_t offset);
    bool setVendorHomingU32(uint16_t index, uint8_t subindex, uint32_t value);

    /**
     * @brief CSP 周期直写目标位置（RT 安全，无命令队列）
     * @param rad 目标关节角 [rad]
     */
    AxisError setPositionRT(double rad) noexcept;

    /**
     * @brief 转矩模式运动（CST 模式）
     * @param torque_percent 目标转矩 [%]，范围 -100.0 ~ 100.0
     * @return 错误码，OK 表示命令已入队
     *
     * 说明：
     * 如果当前不是 CST/PT 模式，会自动尝试切换模式。
     * 转矩值会经过安全限幅：超出 ±100% 会被拒绝。
     */
    AxisError moveTorque(double torque_percent) noexcept;

    /**
     * @brief 设置运行模式（异步命令）
     * @param mode 运行模式（CiA402 数值）
     *
     * moveAbsolute() / moveVelocity() / moveTorque() 内部会在必要时
     * 自动调用 setMode()，用户通常不需要显式调用。
     */
    void setMode(ServoOperationMode mode) noexcept;

    /**
     * @brief 获取当前运行模式
     * @return 当前模式显示（从驱动器读取的 0x6061 值）
     */
    ServoOperationMode getMode() const noexcept;

    /**
     * @brief 获取当前运行模式名称（便于显示/日志）
     * @return 模式名称字符串，如 "CSP", "CSV", "Homing" 等
     */
    std::string getModeName() const noexcept;

    // ===================== 状态查询（实时安全，可跨线程） =====================
    bool isEnabled() const noexcept;         ///< 是否已使能
    bool hasFault() const noexcept;          ///< 是否故障
    bool isTargetReached() const noexcept;   ///< 目标是否到达
    bool isHomingComplete() const noexcept;  ///< 回零是否完成（成功后闩锁，切 CSP 仍为 true）
    bool isHomingActive() const noexcept;    ///< 回零进行中
    bool isHomingError() const noexcept;     ///< 回零错误/超时
    bool isQuickStopActive() const noexcept; ///< 快停是否激活

    /** @brief 获取当前关节角 [rad] */
    double getPosition() const noexcept;
    /** @brief 获取当前关节速度 [rad/s] */
    double getVelocity() const noexcept;
    /** @brief 获取当前关节转矩 [%]，范围 -100.0 ~ 100.0 */
    double getTorque() const noexcept;

    /**
     * @brief 获取最近一次下发的目标位置 [rad]
     * @return 通过 moveAbsolute()/moveRelative() 下发的目标关节角
     *
     * 说明：跨线程安全（原子读），可直接在 GUI 线程周期性调用用于绘图/显示。
     */
    double getTargetPosition() const noexcept { return last_target_rad_.load(std::memory_order_relaxed); }

    /** @brief 关节跟随误差 [rad] = 目标 - 实际 */
    double followingError() const noexcept {
        return getTargetPosition() - getPosition();
    }

    /**
     * @brief 获取最近一次下发的目标速度 [rad/s]
     * @return 通过 moveVelocity() 下发的目标速度
     *
     * 说明：跨线程安全（原子读），可直接在 GUI 线程周期性调用用于绘图/显示。
     */
    double getTargetVelocity() const noexcept { return target_velocity_rad_s_.load(std::memory_order_relaxed); }

    /** @brief 获取最近一次错误信息 */
    std::string lastError() const;

    /** @brief 获取单位转换器的只读引用 */
    const JointUnitConverter& converter() const noexcept { return converter_; }

    /** @brief 底层伺服（CiA402 或 Mock） */
    IServoDrive* drive() noexcept { return motor_.get(); }
    const IServoDrive* drive() const noexcept { return motor_.get(); }

    /**
     * @brief 若底层是 `CiA402Motor` 则返回之，否则 nullptr（例如 Mock）
     * @note 旧代码 `axis.motor()->...` 在真机路径上仍可用
     */
    CiA402Motor* motor() noexcept;
    const CiA402Motor* motor() const noexcept;

    /** @brief 最近一次下发的目标脉冲（若有） */
    int32_t lastTargetPulse() const noexcept { return last_target_pulse_; }

private:
    // 内部安全转换，钳位后返回错误码
    AxisError convertPositionOrClamp(double rad, int32_t& pulse) noexcept;
    AxisError convertVelocityOrClamp(double rad_per_s, int32_t& pulse_vel) noexcept;
    AxisError checkMotionAllowed() const noexcept;

    std::string name_;
    std::unique_ptr<IServoDrive> motor_;
    JointUnitConverter converter_;

    std::atomic<bool> require_homing_complete_{false};

    // 上次目标（用于调试 + 对外暴露给UI绘图，原子类型支持跨线程安全读）
    std::atomic<double> last_target_rad_{0.0};
    int32_t last_target_pulse_ = 0;
    std::atomic<double> target_velocity_rad_s_{0.0};
};