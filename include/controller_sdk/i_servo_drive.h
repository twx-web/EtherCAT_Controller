#pragma once

/**
 * @file i_servo_drive.h
 * @brief 伺服驱动抽象（脉冲 / CiA402 层）
 *
 * ## RT / NRT 契约
 * - **RT**：`realtimeCycle`、`setTargetPositionRT`、`setTargetVelocityRT`，
 *   以及基于 PDO 缓存的状态查询。禁止堆分配、阻塞、日志、SDO。
 * - **NRT**：`enable` / `disable` / `clearFault` 等异步命令投递（有界环）、
 *   切模式、转矩、SDO 配置、字符串诊断。
 *
 * 默认实现：`CiA402Motor`。无硬件：`MockServoDrive`。
 * 未覆盖的虚函数有默认空实现，自定义驱动只需实现周期与 PDO 查询。
 */

#include <cstdint>
#include <cstddef>
#include <string>

/**
 * @brief 运行模式（CiA402 0x6060 / 0x6061 数值）
 *
 * `CiA402Motor::OperationMode` 是本枚举的别名，旧代码不用改。
 */
enum class ServoOperationMode : int8_t {
    PROFILE_POSITION = 1,         ///< PP
    PROFILE_VELOCITY = 3,         ///< PV
    PROFILE_TORQUE = 4,           ///< PT
    HOMING = 6,                   ///< HM
    CYCLIC_SYNC_POSITION = 8,     ///< CSP
    CYCLIC_SYNC_VELOCITY = 9,     ///< CSV
    CYCLIC_SYNC_TORQUE = 10       ///< CST
};

/**
 * @class IServoDrive
 * @brief 单轴伺服驱动接口（单位：脉冲；状态来自过程数据缓存）
 */
class IServoDrive {
public:
    virtual ~IServoDrive() = default;

    /**
     * @brief 实时周期入口：消费命令环、更新状态机、准备写 PDO
     * @param max_commands_per_cycle 本周期最多处理的异步命令数
     */
    virtual void realtimeCycle(std::size_t max_commands_per_cycle = 8) = 0;

    /**
     * @brief CSP 周期直写目标位置（RT，不走命令环）
     * @param position 目标位置 [pulse]
     */
    virtual void setTargetPositionRT(int32_t position) noexcept = 0;

    /**
     * @brief CSV 周期直写目标速度（RT，不走命令环）
     * @param velocity 目标速度 [pulse/s]
     */
    virtual void setTargetVelocityRT(int32_t velocity) noexcept = 0;

    /** @brief 是否已使能（Operation enabled） */
    virtual bool isEnabled() const = 0;
    /** @brief 是否处于故障 */
    virtual bool isFault() const = 0;
    /**
     * @brief 实际位置
     * @return [pulse]
     */
    virtual int32_t getActualPosition() const = 0;
    /**
     * @brief 实际速度
     * @return [pulse/s]
     */
    virtual int32_t getActualVelocity() const = 0;
    /**
     * @brief 状态字（CiA402 0x6041）
     * @return 原始 PDO 缓存值
     */
    virtual uint16_t getStatusWord() const = 0;

    /** @brief 请求使能（NRT 投递，RT 消费） */
    virtual void enable() = 0;
    /** @brief 请求禁能 */
    virtual void disable() = 0;
    /** @brief 请求故障复位 */
    virtual void clearFault() = 0;
    /** @brief 请求快停 */
    virtual void quickStop() = 0;
    /** @brief 请求启动回零（Homing 模式） */
    virtual void startHoming() = 0;

    // ---- 加厚接口：默认空实现，CiA402Motor / MockServoDrive 覆盖 ----

    /** @brief 目标到达（Statusword bit10；默认 false） */
    virtual bool isTargetReached() const { return false; }
    /** @brief 回零完成（成功后应闩锁；默认 false） */
    virtual bool isHomingComplete() const { return false; }
    /** @brief 回零进行中 */
    virtual bool isHomingActive() const { return false; }
    /** @brief 回零错误/超时 */
    virtual bool isHomingError() const { return false; }
    /** @brief 快停激活 */
    virtual bool isQuickStopActive() const { return false; }
    /**
     * @brief 实际转矩
     * @return 千分比（0.1%）；未映射返回 0
     */
    virtual int16_t getActualTorque() const { return 0; }
    /**
     * @brief 模式显示（0x6061）
     * @return 原始值；未映射返回 0
     */
    virtual int8_t getModeDisplay() const { return 0; }
    /**
     * @brief 最近一次驱动错误码（实现自定义，CiA402 为 `CiA402Motor::ErrorCode` 底层值）
     */
    virtual uint8_t lastErrorCode() const { return 0; }
    /** @brief NRT：人类可读错误；RT 禁止调用 */
    virtual std::string lastError() const { return {}; }

    /** @brief 切换运行模式（NRT 投递） */
    virtual void setMode(int8_t /*mode*/) {}
    void setMode(ServoOperationMode mode) { setMode(static_cast<int8_t>(mode)); }

    virtual void moveAbsolute(int32_t /*position*/) {}
    virtual void moveRelative(int32_t /*delta*/) {}
    virtual void moveVelocity(int32_t /*velocity*/) {}
    virtual void moveTorque(int16_t /*torque*/) {}
    virtual void jogPositive(int32_t /*velocity*/) {}
    virtual void jogNegative(int32_t /*velocity*/) {}
    virtual void jogStop() {}
    virtual void stop() {}
    virtual void clearStop() {}
    virtual void clearQuickStop() {}
    virtual void setHomingTimeoutCycles(uint32_t /*cycles*/) noexcept {}

    /** @brief NRT：写 0x6098；不支持返回 false */
    virtual bool setHomingMethod(int8_t /*method*/) { return false; }
    virtual bool setHomingSpeeds(uint32_t /*switch_search*/, uint32_t /*zero_search*/) {
        return false;
    }
    virtual bool setHomingAcceleration(uint32_t /*acceleration*/) { return false; }
    virtual bool setHomeOffset(int32_t /*offset*/) { return false; }
    virtual bool setVendorHomingU32(uint16_t /*index*/, uint8_t /*subindex*/,
                                    uint32_t /*value*/) {
        return false;
    }

    /**
     * @brief NRT SDO 写。禁止 RT。不支持或失败返回 false
     */
    virtual bool writeSdo(uint16_t /*index*/, uint8_t /*subindex*/, const void* /*data*/,
                          size_t /*size*/, uint32_t /*timeout_ms*/ = 500) {
        return false;
    }
    /**
     * @brief NRT SDO 读。禁止 RT。不支持或失败返回 false
     */
    virtual bool readSdo(uint16_t /*index*/, uint8_t /*subindex*/, void* /*data*/,
                         size_t /*size*/, uint32_t /*timeout_ms*/ = 500) {
        return false;
    }
};
