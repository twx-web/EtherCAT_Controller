#pragma once

/**
 * @file MotionCycle.h
 * @brief EtherCAT 标准实时周期模板
 *
 * 固定顺序：
 *   begin: application_time → receive → process → 状态更新 → DC sync
 *   [用户控制逻辑]
 *   end:   queue → send
 *
 * 并提供 WC / 从站 OP 通信门控、连续失败计数与可选粘性闩锁，供上层决定是否下发运动。
 *
 * ## RT 契约
 * `begin`/`end`/`isCommunicationOk` 仅在实时线程调用；回调内禁止阻塞与堆分配。
 * 用户控制逻辑（IK、`setJointRT`）放在 begin 与 end 之间。
 */

#include "EthercatMaster.h"

#include <cstdint>
#include <functional>

/**
 * @class MotionCycle
 * @brief 工业推荐的 EtherCAT 周期编排器（不拥有主站）
 */
class MotionCycle {
public:
    struct Config {
        bool enable_dc = true;                 ///< 是否做 DC 同步
        uint32_t wc_fail_threshold = 3;        ///< 连续失败达到该次数触发 on_comm_fault（0=禁用）
        bool require_slaves_op = true;         ///< 是否要求从站进入 OP
        /** true：总线恢复后仍保持闩锁，须 clearFaultLatch()；false：恢复后自动解除 */
        bool sticky_comm_latch = true;
        /** 通信连续失败达到阈值时回调一次（RT 上下文，须 RT 安全） */
        std::function<void(uint32_t consecutive_failures)> on_comm_fault;
    };

    using ControlFn = std::function<void(bool communication_ok)>;

    explicit MotionCycle(EthercatMaster& master);
    void setConfig(const Config& cfg);

    /**
     * @brief 周期前半段（收包 + 状态 + DC）
     * @param app_time_ns 应用时间；0 表示自动取 CLOCK_MONOTONIC
     */
    void begin(uint64_t app_time_ns = 0);

    /**
     * @brief 当前周期通信是否健康（基于 begin 后的状态）
     */
    bool communicationOk() const noexcept { return communication_ok_; }

    /**
     * @brief 连续通信失败次数
     */
    uint32_t consecutiveFailures() const noexcept { return consecutive_failures_; }

    /**
     * @brief 周期后半段（发包）
     */
    void end();

    /**
     * @brief 一周期完整执行：begin → control(ok) → end
     */
    void run(uint64_t app_time_ns, const ControlFn& control);

    /** @brief 手动清除通信故障闩锁（例如 clearFault 之后） */
    void clearFaultLatch() noexcept;

    /** @brief 通信故障是否已闩锁（触发过 on_comm_fault） */
    bool faultLatched() const noexcept { return fault_latched_; }

    /** @brief 最近一次门控失败原因（短字符串，NRT 可读） */
    const char* lastFaultReason() const noexcept { return last_fault_reason_; }

    EthercatMaster& master() noexcept { return master_; }
    const EthercatMaster& master() const noexcept { return master_; }

private:
    void updateGate();

    EthercatMaster& master_;
    Config config_;
    bool communication_ok_ = false;
    uint32_t consecutive_failures_ = 0;
    bool fault_latched_ = false;
    char last_fault_reason_[48] = {};
};
