#pragma once

/**
 * @file mock_servo.h
 * @brief 无硬件仿真伺服（实现 IServoDrive）
 *
 * ## RT / NRT 契约
 * 与 `IServoDrive` 相同。CSP 下目标位置即反馈；不依赖 IgH。
 *
 * 用于开发机 / CI / Studio Mock，也可交给 `Axis` 构造。
 */

#include "i_servo_drive.h"

#include <atomic>
#include <cstdint>
#include <string>

/**
 * @class MockServoDrive
 * @brief 仿真伺服：使能后 `getActualPosition` 跟随上次目标
 */
class MockServoDrive : public IServoDrive {
public:
    MockServoDrive() = default;

    void realtimeCycle(std::size_t max_commands_per_cycle = 8) override;
    void setTargetPositionRT(int32_t position) noexcept override;
    void setTargetVelocityRT(int32_t velocity) noexcept override;

    bool isEnabled() const override;
    bool isFault() const override;
    int32_t getActualPosition() const override;
    int32_t getActualVelocity() const override;
    uint16_t getStatusWord() const override;

    void enable() override;
    void disable() override;
    void clearFault() override;
    void quickStop() override;
    void startHoming() override;

    bool isTargetReached() const override;
    bool isHomingComplete() const override;
    bool isHomingActive() const override { return false; }
    bool isHomingError() const override { return false; }
    bool isQuickStopActive() const override;
    int16_t getActualTorque() const override { return 0; }
    int8_t getModeDisplay() const override;
    uint8_t lastErrorCode() const override { return 0; }
    std::string lastError() const override { return {}; }

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
    void clearQuickStop() override;
    void setHomingTimeoutCycles(uint32_t) noexcept override {}

    /**
     * @brief 注入/清除仿真故障
     * @param fault true 进入故障
     */
    void injectFault(bool fault) noexcept;

    /**
     * @brief 仿真回零是否已完成
     * @return `startHoming` 后为 true
     */
    bool homingDone() const noexcept { return homing_done_.load(std::memory_order_relaxed); }

private:
    std::atomic<bool> enabled_{false};
    std::atomic<bool> fault_{false};
    std::atomic<bool> quick_stop_{false};
    std::atomic<bool> homing_done_{false};
    std::atomic<int8_t> mode_{static_cast<int8_t>(ServoOperationMode::CYCLIC_SYNC_POSITION)};
    std::atomic<int32_t> target_pos_{0};
    std::atomic<int32_t> target_vel_{0};
    std::atomic<int32_t> actual_pos_{0};
    std::atomic<int32_t> actual_vel_{0};
    std::atomic<int16_t> target_torque_{0};
};
