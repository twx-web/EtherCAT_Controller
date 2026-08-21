#pragma once

/**
 * @file start_sync_meter.h
 * @brief 多轴「同一拍启动」测量：分别报 Δcmd（指令）与 Δact（实际位置）。
 *
 * 分辨率为一个控制周期（1 ms 周期则 1 ms）。DC 亚微秒对齐测不出，只能判断是否同拍。
 * 不要把编码器 1–2 个计数抖动当成「电机已启动」：Δact 用独立阈值；对拍应用 Δcmd。
 *
 * ## 用法（RT）
 * 1. NRT：reset / setAxisCount / setCycleNs / setActThreshold / setCmdThreshold
 * 2. 每周期 update(实际位置, 目标位置)
 * 3. 与写下目标同一拍调用 markCommand()
 * 4. 继续 update，直到 complete()
 * 5. NRT：cmdSpreadCycles()=0 表示指令同拍；actSpreadCycles() 是反馈滞后差
 */

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <string>

class StartSyncMeter {
public:
    static constexpr std::size_t kMaxAxes = 16;

    StartSyncMeter() noexcept;
    void reset() noexcept;

    void setAxisCount(std::size_t n) noexcept;
    void setCycleNs(uint64_t cycle_ns) noexcept;
    /** 实际位置相对基线超过该增量视为「反馈已动」（编码器计数，默认 32） */
    void setThreshold(int32_t counts) noexcept;
    void setActThreshold(int32_t counts) noexcept { setThreshold(counts); }
    /** 目标位置相对基线超过该增量视为「指令已变」（默认 1） */
    void setCmdThreshold(int32_t counts) noexcept;
    void setTimeoutCycles(uint32_t cycles) noexcept;

    std::size_t axisCount() const noexcept {
        return axis_count_.load(std::memory_order_relaxed);
    }

    /**
     * @brief RT：每周期在读完实际位置后调用（只测 Δact）
     * @param actual 各轴 0x6064（脉冲），长度 n
     */
    void update(const int32_t* actual, std::size_t n) noexcept;

    /**
     * @brief RT：同时测 Δcmd / Δact
     * @param command 各轴目标位置（0x607A）；nullptr 则只更新实际位置
     */
    void update(const int32_t* actual, const int32_t* command, std::size_t n) noexcept;

    /** @brief RT：与写下各轴目标位置同一拍调用 */
    void markCommand() noexcept;

    bool armed() const noexcept { return armed_.load(std::memory_order_acquire); }
    bool complete() const noexcept { return complete_.load(std::memory_order_acquire); }
    bool timedOut() const noexcept { return timed_out_.load(std::memory_order_acquire); }

    /** 该轴相对 markCommand 的实际位置越阈延迟（周期）。未检测到为 -1 */
    int32_t startDelayCycles(std::size_t axis) const noexcept;
    int32_t actDelayCycles(std::size_t axis) const noexcept { return startDelayCycles(axis); }
    /** 该轴指令首次越阈延迟。未测指令或未越阈为 -1 */
    int32_t cmdDelayCycles(std::size_t axis) const noexcept;

    /** 已启动轴中 max(delay)-min(delay)。不足 2 轴返回 -1 */
    int32_t spreadCycles() const noexcept;
    int32_t actSpreadCycles() const noexcept { return spreadCycles(); }
    int32_t cmdSpreadCycles() const noexcept;

    /** spreadCycles * cycle_ns；同拍为 0。不足 2 轴返回 0 */
    uint64_t spreadNs() const noexcept;
    uint64_t cmdSpreadNs() const noexcept;
    std::size_t startedCount() const noexcept;
    std::size_t cmdChangedCount() const noexcept;
    uint32_t commandCycle() const noexcept {
        return command_cycle_.load(std::memory_order_relaxed);
    }
    uint32_t elapsedSinceCommand() const noexcept;

    /** NRT：打印 Δcmd / Δact */
    void printReport() const;
    /** NRT：与 printReport 相同内容，供 GUI 显示 */
    std::string reportText() const;

private:
    int32_t spreadOf(const std::atomic<int32_t>* delays) const noexcept;
    std::size_t countNonNeg(const std::atomic<int32_t>* delays) const noexcept;

    std::atomic<std::size_t> axis_count_{0};
    std::atomic<uint64_t> cycle_ns_{1000000};
    std::atomic<int32_t> threshold_{32};
    std::atomic<int32_t> cmd_threshold_{1};
    std::atomic<uint32_t> timeout_cycles_{1000};

    std::atomic<bool> have_sample_{false};
    std::atomic<bool> have_cmd_sample_{false};
    std::atomic<bool> armed_{false};
    std::atomic<bool> complete_{false};
    std::atomic<bool> timed_out_{false};

    std::atomic<uint32_t> cycle_{0};
    std::atomic<uint32_t> command_cycle_{0};

    int32_t last_pos_[kMaxAxes]{};
    int32_t last_cmd_[kMaxAxes]{};
    int32_t baseline_[kMaxAxes]{};
    int32_t baseline_cmd_[kMaxAxes]{};
    std::atomic<int32_t> delay_cycles_[kMaxAxes]{};
    std::atomic<int32_t> cmd_delay_cycles_[kMaxAxes]{};
};
