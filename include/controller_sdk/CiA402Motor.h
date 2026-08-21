#pragma once

/**
 * @file CiA402Motor.h
 * @brief CiA402 EtherCAT 伺服电机封装（工业级实时安全）
 *
 * - PDO 读写 + CiA402 状态机（含故障上升沿复位）
 * - 有界命令环（NRT 投递 / RT 消费，避免无界 queue）
 * - CSP 周期直写目标（setTargetPositionRT）
 * - Homing 握手与超时
 * - RT 路径仅写原子错误码，不分配/不打日志
 */

#include "i_servo_drive.h"
#include "sdo_client.h"

#include <ecrt.h>
#include <atomic>
#include <cstdint>
#include <cstddef>
#include <climits>
#include <mutex>
#include <string>
#include <functional>
#include <array>

/**
 * @class CiA402Motor
 * @brief CiA402 伺服实现，满足 IServoDrive 契约。
 *
 * RT：realtimeCycle / setTarget*RT / 缓存查询；NRT：SDO、字符串诊断、配置。
 */
class CiA402Motor : public IServoDrive {
public:
    struct Offsets {
        static constexpr unsigned int UNMAPPED = UINT_MAX;

        unsigned int ctrl_word = UNMAPPED;
        unsigned int target_position = UNMAPPED;
        unsigned int target_velocity = UNMAPPED;
        unsigned int target_torque = UNMAPPED;
        unsigned int operation_mode = UNMAPPED;

        unsigned int status_word = UNMAPPED;
        unsigned int position_actual = UNMAPPED;
        unsigned int velocity_actual = UNMAPPED;
        unsigned int torque_actual = UNMAPPED;
        unsigned int mode_display = UNMAPPED;

        /** 数字量输入/输出（常见 0x60FD / 0x60FE），供 DigitalIo 使用 */
        unsigned int digital_inputs = UNMAPPED;
        unsigned int digital_outputs = UNMAPPED;

        static constexpr bool isMapped(unsigned int offset) noexcept {
            return offset != UNMAPPED;
        }
    };

    using OperationMode = ServoOperationMode;

    enum class CommandType : uint8_t {
        NONE = 0,
        ENABLE,
        DISABLE,
        CLEAR_FAULT,
        SET_MODE,
        MOVE_ABS,
        MOVE_REL,
        MOVE_VEL,
        MOVE_TORQUE,
        JOG_POS,
        JOG_NEG,
        JOG_STOP,
        STOP,
        CLEAR_STOP,
        QUICK_STOP,
        CLEAR_QUICK_STOP,
        START_HOMING
    };

    struct Command {
        CommandType type = CommandType::NONE;
        int32_t param1 = 0;
        int32_t param2 = 0;
    };

    /** RT 安全错误码（不分配字符串） */
    enum class ErrorCode : uint8_t {
        OK = 0,
        DomainNull,
        CmdQueueFull,
        TargetVelUnmapped,
        TargetTorqueUnmapped,
        HomingTimeout,
        HomingError,
        FaultPresent,
        SdoFailed,
        Unknown
    };

    using ErrorCallback = std::function<void(const std::string& error)>;

    static constexpr std::size_t kCmdQueueCapacity = 64;

    CiA402Motor(ec_slave_config_t* slave_config, uint8_t* domain_pd, const Offsets& offsets);
    ~CiA402Motor();

    CiA402Motor(const CiA402Motor&) = delete;
    CiA402Motor& operator=(const CiA402Motor&) = delete;

    void setDomainData(uint8_t* domain_pd);
    void setOffsets(const Offsets& offsets);
    void setErrorCallback(ErrorCallback callback);

    /** @brief NRT SDO 客户端（对象字典配置）；禁止在 RT 周期调用 */
    SdoClient& sdo() noexcept { return sdo_; }
    const SdoClient& sdo() const noexcept { return sdo_; }

    /** 回零超时（周期数，默认 10000 ≈ 10s@1ms）；0 表示不超时 */
    void setHomingTimeoutCycles(uint32_t cycles) noexcept override {
        homing_timeout_cycles_ = cycles;
    }

    void realtimeCycle(std::size_t max_commands_per_cycle = 8) override;

    // ---------- 异步命令（NRT 投递 / RT 消费）----------
    void enable() override;
    void disable() override;
    void clearFault() override;
    void setMode(int8_t mode) override;
    using IServoDrive::setMode;
    void moveAbsolute(int32_t position) override;
    void moveRelative(int32_t delta) override;
    void moveVelocity(int32_t velocity) override;
    void moveTorque(int16_t torque) override;
    void jogPositive(int32_t velocity) override;
    void jogNegative(int32_t velocity) override;
    void jogStop() override;
    void stop() override;
    void clearStop() override;
    void quickStop() override;
    void clearQuickStop() override;
    void startHoming() override;

    /**
     * @brief CSP 周期直写目标位置（RT 安全，无队列）
     * @note 本实现在 realtimeCycle 末尾应用挂起的 RT 目标。
     */
    void setTargetPositionRT(int32_t position) noexcept override;

    /** 挂起的 RT 目标在本周期 write 阶段生效 */
    void setTargetVelocityRT(int32_t velocity) noexcept override;

    // ---------- 状态查询（RT：读原子缓存）----------
    bool isEnabled() const override;
    bool isFault() const override;
    bool isTargetReached() const override;
    /** Homing attained 后闩锁；再次 startHoming 才清。切出 Homing/CSP 后仍为 true */
    bool isHomingComplete() const override;
    bool isHomingActive() const override;
    bool isHomingError() const override;
    bool isQuickStopActive() const override;

    int32_t getActualPosition() const override;
    int32_t getActualVelocity() const override;
    int16_t getActualTorque() const override;
    uint16_t getStatusWord() const override;
    int8_t getModeDisplay() const override;

    uint8_t lastErrorCode() const noexcept override {
        return static_cast<uint8_t>(last_error_code_.load(std::memory_order_relaxed));
    }
    ErrorCode lastErrorEnum() const noexcept {
        return last_error_code_.load(std::memory_order_relaxed);
    }
    uint64_t droppedCommands() const noexcept {
        return dropped_commands_.load(std::memory_order_relaxed);
    }

    bool setProfileVelocity(int32_t velocity);
    bool setProfileAcceleration(uint32_t acceleration);
    bool setProfileDeceleration(uint32_t deceleration);
    bool setHomingMethod(int8_t method) override;
    /**
     * @brief 写回原速度 0x6099（NRT）
     * @param switch_search 寻开关速度（子索引 1，驱动器单位，通常 pulse/s）
     * @param zero_search 寻零速度（子索引 2）
     */
    bool setHomingSpeeds(uint32_t switch_search, uint32_t zero_search) override;
    /** @brief 写回原加速度 0x609A（NRT） */
    bool setHomingAcceleration(uint32_t acceleration) override;
    /** @brief 写原点偏移 0x607C（NRT） */
    bool setHomeOffset(int32_t offset) override;
    /**
     * @brief 厂商扩展回原对象（如 GSHD 0x20AA）。失败返回 false，不阻止标准 6099。
     */
    bool setVendorHomingU32(uint16_t index, uint8_t subindex, uint32_t value) override;
    bool getErrorCode(uint16_t& error_code) const;

    bool writeSdo(uint16_t index, uint8_t subindex, const void* data, size_t size,
                  uint32_t timeout_ms = 500) override;
    bool readSdo(uint16_t index, uint8_t subindex, void* data, size_t size,
                 uint32_t timeout_ms = 500) override;

    /** NRT：将错误码格式化为字符串（可含回调侧日志） */
    std::string lastError() const override;

    void updateCache();
    void processCommands(std::size_t max_commands = 8);
    void updateStateMachine();
    void writeControlWord();

private:
    bool pushCommand(const Command& command) noexcept;
    void executeCommand(const Command& command) noexcept;

    void writeOperationMode(OperationMode mode) noexcept;
    void writeTargetPosition(int32_t position) noexcept;
    void writeTargetVelocity(int32_t velocity) noexcept;
    void writeTargetTorque(int16_t torque) noexcept;

    bool writeSDO(uint16_t index, uint8_t subindex, const void* data, size_t size, uint32_t timeout_ms = 500);
    bool readSDO(uint16_t index, uint8_t subindex, void* data, size_t size, uint32_t timeout_ms = 500);

    void setErrorCode(ErrorCode code) noexcept;
    static const char* errorCodeToCStr(ErrorCode code) noexcept;

    /** 0x6061 已映射则看模式显示；未映射则把 homing_active_ 当作 Homing 上下文 */
    bool homingModeContext() const noexcept;
    void noteHomingBit12(uint16_t status_word) noexcept;
    bool homingAttainedNow(uint16_t status_word) const noexcept;

    ec_slave_config_t* sc_ = nullptr;
    uint8_t* domain_pd_ = nullptr;
    Offsets offsets_;
    SdoClient sdo_;  ///< NRT：对象字典

    // 有界命令环：单生产者(NRT) + 单消费者(RT)
    std::array<Command, kCmdQueueCapacity> cmd_ring_{};
    std::atomic<uint32_t> cmd_head_{0};  // 消费者
    std::atomic<uint32_t> cmd_tail_{0};  // 生产者
    std::atomic<uint64_t> dropped_commands_{0};

    mutable std::mutex error_mutex_;
    std::string last_error_;
    ErrorCallback error_callback_;
    std::atomic<ErrorCode> last_error_code_{ErrorCode::OK};

    uint16_t ctrl_word_ = 0x0006;
    OperationMode target_mode_ = OperationMode::CYCLIC_SYNC_POSITION;

    bool enable_requested_ = false;

    // 故障复位：上升沿状态机
    enum class FaultResetPhase : uint8_t { Idle, EnsureLow, PulseHigh, WaitClear };
    FaultResetPhase fault_reset_phase_ = FaultResetPhase::Idle;
    uint32_t fault_reset_cycles_ = 0;

    // Homing
    bool homing_active_ = false;
    uint32_t homing_cycles_ = 0;
    uint32_t homing_timeout_cycles_ = 10000;
    bool homing_error_latched_ = false;
    /** 0x6061 未映射：须先见到 bit12=0，再上升沿才当 Homing attained（避免 CSP Target reached 误完成） */
    bool homing_attained_armed_ = false;

    // RT 直写挂起
    std::atomic<bool> rt_pos_pending_{false};
    std::atomic<int32_t> rt_pos_value_{0};
    std::atomic<bool> rt_vel_pending_{false};
    std::atomic<int32_t> rt_vel_value_{0};

    std::atomic<int32_t> cached_position_{0};
    std::atomic<int32_t> cached_velocity_{0};
    std::atomic<int16_t> cached_torque_{0};
    std::atomic<uint16_t> cached_status_word_{0};
    std::atomic<int8_t> cached_mode_display_{0};
    std::atomic<bool> cached_enabled_{false};
    std::atomic<bool> cached_homing_active_{false};
    std::atomic<bool> cached_homing_complete_{false};

    static constexpr uint16_t CIA402_STATE_MASK = 0x006F;
    static constexpr uint16_t STATUS_FAULT           = 0x0008;
    static constexpr uint16_t STATUS_TARGET_REACHED  = 1u << 10;
    static constexpr uint16_t STATUS_HOMING_ATTAINED = 1u << 12;
    static constexpr uint16_t STATUS_HOMING_ERROR    = 1u << 13;

    static constexpr uint16_t CONTROL_SHUTDOWN         = 0x0006;
    static constexpr uint16_t CONTROL_SWITCH_ON        = 0x0007;
    static constexpr uint16_t CONTROL_ENABLE_OPERATION = 0x000F;
    static constexpr uint16_t CONTROL_FAULT_RESET      = 0x0080;
    static constexpr uint16_t CONTROL_DISABLE_VOLTAGE  = 0x0000;

    static constexpr uint16_t CONTROL_BIT_QUICK_STOP   = 1u << 2;
    static constexpr uint16_t CONTROL_BIT_NEW_SETPOINT = 1u << 4;
    static constexpr uint16_t CONTROL_BIT_HALT         = 1u << 8;

    static constexpr uint16_t ST_NOT_READY          = 0x0000;
    static constexpr uint16_t ST_SWITCH_ON_DISABLED = 0x0040;
    static constexpr uint16_t ST_READY_TO_SWITCH_ON = 0x0021;
    static constexpr uint16_t ST_SWITCHED_ON        = 0x0023;
    static constexpr uint16_t ST_OPERATION_ENABLED  = 0x0027;
    static constexpr uint16_t ST_QUICK_STOP_ACTIVE  = 0x0007;
    static constexpr uint16_t ST_FAULT_REACTION     = 0x000F;
    static constexpr uint16_t ST_FAULT              = 0x0008;
};
