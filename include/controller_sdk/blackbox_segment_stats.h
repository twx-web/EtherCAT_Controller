#pragma once

/**
 * @file blackbox_segment_stats.h
 * @brief 黑盒分段统计：匀速 / 加速段的 Δcmd、Δact、Δfe（NRT）
 *
 * 对 `TraceSample` 流分类：
 * - 加速：|cart_acc| 或关节指令加速度超过阈值
 * - 匀速：速度够、加速度低
 * - 静止：速度接近 0
 *
 * Δcmd / Δact 是相邻样本目标/实际位置变化的轴间最大值；
 * 低于阈值的 1–2 计数级抖动不计入（与 `StartSyncMeter` 同一原则）。
 * Δfe 是该样本最大 |跟随误差|。
 *
 * 不改 CSV 列（仍为 8 轴通道）；`reportText()` 供 GUI / 停录后打印。
 * 1.x 与 `TraceSample` 相同：最多 8 轴。
 */

#include "trace_buffer.h"

#include <cstdint>
#include <string>

class BlackboxSegmentStats {
public:
    enum class Phase : uint8_t {
        Dwell = 0,  ///< 静止
        Accel = 1,  ///< 加速/减速
        Cruise = 2  ///< 匀速
    };

    struct Config {
        double accel_thresh_mm_s2 = 50.0;     ///< |cart_acc| 超过则算加速段
        double cruise_speed_mm_s = 5.0;       ///< 路径速度超过则可能是匀速
        double joint_accel_thresh_rad_s2 = 0.5; ///< 无笛卡尔时用关节指令加速度
        double joint_speed_thresh_rad_s = 0.02;
        double cmd_thresh_rad = 1e-6;         ///< |Δcmd| 低于此不算指令变化
        double act_thresh_rad = 1e-5;         ///< |Δact| 低于此不算实际运动
    };

    struct Accumulators {
        uint64_t samples = 0;
        uint32_t segments = 0;
        double max_dcmd = 0.0;  ///< 段内最大 |Δcmd| [rad]
        double max_dact = 0.0;  ///< 段内最大 |Δact| [rad]
        double max_fe = 0.0;    ///< 段内最大 |Δfe| [rad]
        double sum_fe = 0.0;
        double meanFe() const noexcept {
            return samples ? (sum_fe / static_cast<double>(samples)) : 0.0;
        }
    };

    void reset() noexcept;
    void setConfig(const Config& cfg) noexcept { cfg_ = cfg; }
    const Config& config() const noexcept { return cfg_; }

    /** @brief 追加一条样本（NRT） */
    void feed(const TraceSample& s) noexcept;

    const Accumulators& accel() const noexcept { return accel_; }
    const Accumulators& cruise() const noexcept { return cruise_; }
    const Accumulators& dwell() const noexcept { return dwell_; }

    Phase lastPhase() const noexcept { return last_phase_; }
    uint64_t totalSamples() const noexcept { return total_; }

    /** @brief 人类可读报告 */
    std::string reportText() const;

private:
    Phase classify(const TraceSample& s, double dcmd, double dt) const noexcept;
    static void accumulate(Accumulators& a, double dcmd, double dact, double fe,
                           bool new_segment) noexcept;

    Config cfg_{};
    Accumulators accel_{};
    Accumulators cruise_{};
    Accumulators dwell_{};
    bool have_prev_ = false;
    Phase last_phase_ = Phase::Dwell;
    double prev_target_[kTraceMaxAxes]{};
    double prev_actual_[kTraceMaxAxes]{};
    double prev_t_ = 0.0;
    uint8_t prev_n_ = 0;
    uint64_t total_ = 0;
};
