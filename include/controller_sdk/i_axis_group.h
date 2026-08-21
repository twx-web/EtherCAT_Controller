#pragma once

/**
 * @file i_axis_group.h
 * @brief 轴组抽象（SI 单位），供 RobotController / 上层解耦
 *
 * ## RT / NRT 契约
 * - **RT**：`realtimeCycleAll`、`setJointRT`、布尔状态查询、
 *   `copyPositions` / `copyMotor*`（无堆分配）。
 * - **NRT**：`enableAll` 等命令投递、`getPositions`（返回 vector）、
 *   `faultSummary`（字符串）、配置类接口。
 *
 * 默认实现：`AxisGroup`。无硬件：`MockAxisGroup`。
 */

#include "diagnostics_snapshot.h"
#include "joint_unit_converter.h"

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

/** @brief 多轴操作错误码 */
enum class AxisGroupError : uint8_t {
    OK = 0,                 ///< 成功
    NotAllEnabled,          ///< 并非全部轴已使能
    PositionOutOfLimit,     ///< 目标位置超软限位
    VelocityOutOfLimit,     ///< 目标速度超限
    ConversionOverflow,     ///< 单位转换整数溢出
    SizeMismatch,           ///< 向量长度与轴数不一致
    InvalidArgument,        ///< 参数非法（非有限值等）
    FaultPresent            ///< 存在轴故障
};

/**
 * @class IAxisGroup
 * @brief 多轴组接口（关节角 rad；电机层脉冲由实现转换）
 */
class IAxisGroup {
public:
    virtual ~IAxisGroup() = default;

    /**
     * @brief 轴数量
     * @return 组内轴数
     */
    virtual size_t size() const noexcept = 0;

    /**
     * @brief 对全部轴执行实时周期任务
     * @param max_commands_per_axis 每轴本周期最多处理的异步命令数
     */
    virtual void realtimeCycleAll(std::size_t max_commands_per_axis = 8) noexcept = 0;

    /**
     * @brief CSP 周期直写全部关节目标位置（RT）
     * @param positions 各轴目标角 [rad]，长度须等于 `size()`
     * @return 错误码；长度不符为 `SizeMismatch`
     */
    virtual AxisGroupError setJointRT(const std::vector<double>& positions) noexcept = 0;

    /**
     * @brief CSP 周期直写（RT，无堆分配）
     * @param positions 各轴目标角 [rad]
     * @param n 长度，须等于 `size()`
     * @return 错误码；`positions==nullptr` 为 `InvalidArgument`
     * @note 默认实现会拷到临时 `vector`（可能分配）。`AxisGroup` / `MockAxisGroup` 已覆盖为定长路径。
     */
    virtual AxisGroupError setJointRT(const double* positions, size_t n) noexcept {
        if (!positions) return AxisGroupError::InvalidArgument;
        try {
            return setJointRT(std::vector<double>(positions, positions + n));
        } catch (...) {
            return AxisGroupError::InvalidArgument;
        }
    }

    /**
     * @brief 将当前关节位置写入 `out[0..n)`
     * @param out 输出缓冲
     * @param n 缓冲长度，必须等于 `size()`
     * @return true 成功；长度不符返回 false
     */
    virtual bool copyPositions(double* out, size_t n) const noexcept = 0;

    /** @brief 是否全部已使能 */
    virtual bool allEnabled() const noexcept = 0;
    /** @brief 是否存在任意轴故障 */
    virtual bool anyFault() const noexcept = 0;
    /** @brief 是否有轴正在回零 */
    virtual bool anyHomingActive() const noexcept = 0;
    /** @brief 是否全部回零完成 */
    virtual bool allHomingComplete() const noexcept = 0;
    /** @brief 是否存在回零错误 */
    virtual bool anyHomingError() const noexcept = 0;

    /** @brief 全部轴使能（NRT 投递） */
    virtual void enableAll() noexcept = 0;
    /** @brief 全部轴禁能 */
    virtual void disableAll() noexcept = 0;
    /** @brief 全部轴故障复位 */
    virtual void clearFaultAll() noexcept = 0;
    /** @brief 全部轴暂停（Halt） */
    virtual void stopAll() noexcept = 0;
    /** @brief 清除全部轴暂停 */
    virtual void clearStopAll() noexcept = 0;
    /** @brief 全部轴快停 */
    virtual void quickStopAll() noexcept = 0;
    /** @brief 清除全部轴快停 */
    virtual void clearQuickStopAll() noexcept = 0;
    /** @brief 全部轴启动回零 */
    virtual void startHomingAll() noexcept = 0;
    /**
     * @brief 设置全部轴回零超时
     * @param cycles 控制周期数；0 表示不超时
     */
    virtual void setHomingTimeoutCyclesAll(uint32_t cycles) noexcept = 0;

    /**
     * @brief 当前关节角（NRT，可能分配）
     * @return 各轴 [rad]
     */
    virtual std::vector<double> getPositions() const noexcept = 0;

    /**
     * @brief 故障摘要字符串（NRT，可能分配）
     * @return 人类可读文本；无故障时为空或实现定义
     */
    virtual std::string faultSummary() const noexcept = 0;

    /**
     * @brief 读取第 i 轴关节配置（软限位等）
     * @param index 轴索引
     * @return 配置指针；越界返回 nullptr
     */
    virtual const JointAxisConfig* jointConfigAt(size_t index) const noexcept = 0;

    /**
     * @brief 填充各轴诊断
     * @param out 输出缓冲
     * @param n 缓冲长度，须等于 `size()`
     * @return true 成功
     */
    virtual bool fillAxisDiagnostics(AxisDiag* out, size_t n) const noexcept = 0;

    /**
     * @brief 拷贝电机实际位置 [pulse]
     * @param out 输出缓冲
     * @param n 须等于 `size()`
     * @return true 成功
     */
    virtual bool copyMotorActualPositions(int32_t* out, size_t n) const noexcept = 0;
    /**
     * @brief 拷贝电机目标位置 [pulse]
     * @param out 输出缓冲
     * @param n 须等于 `size()`
     * @return true 成功
     */
    virtual bool copyMotorTargetPositions(int32_t* out, size_t n) const noexcept = 0;
    /**
     * @brief 拷贝电机实际速度 [pulse/s]
     * @param out 输出缓冲
     * @param n 须等于 `size()`
     * @return true 成功
     */
    virtual bool copyMotorActualVelocities(int32_t* out, size_t n) const noexcept = 0;
};
