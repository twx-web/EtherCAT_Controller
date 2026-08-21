#pragma once

/**
 * @file axis_profiler.h
 * @brief 单轴 CSP 轨迹执行器：NRT 规划，RT 按周期采样。
 *
 * 不拥有电机、不写 PDO。调用方在实时周期里：
 *   if (profiler.update(dt)) motor.setTargetPositionRT(lround(profiler.commandPosition()));
 *
 * 单位由调用方决定（脉冲、rad、mm 均可）。S / T 用不同接口，由最近一次
 * `moveSCurve` / `moveTCurve` 决定后续 `update` 走哪套插补。
 *
 * - **moveSCurve**：离线 7 段 S 曲线，起止速度有效，按时间采样
 * - **moveTCurve**：在线 T 型（梯形/三角形），运动中再调用会重规划（可折返）
 *
 * T 曲线要求每个控制周期调用一次 `update()`，且 `dt ≈ 1/frequency`。
 *
 * ## RT / NRT
 * - **NRT**：`setLimits` / `moveSCurve` / `moveTCurve` / `abort` / `softStop`
 * - **RT**：`update` 以及指令与状态查询
 */

#include "scurve_planner.h"
#include "tcurve_planner.h"

#include <atomic>
#include <cstdint>

class AxisProfiler {
public:
    enum class ProfileType : std::uint8_t {
        SCurve = 0,  ///< 七段 S 曲线
        TCurve = 1   ///< 梯形/三角形 T 曲线
    };

    struct Limits {
        double max_velocity = 100.0;       ///< 最大速度
        double max_acceleration = 500.0;   ///< 最大加速度
        /** 减速度；≤0 时与 max_acceleration 相同（仅 T 曲线） */
        double max_deceleration = 0.0;
        double max_jerk = 5000.0;          ///< 仅 S 曲线
        double frequency = 1000.0;         ///< 控制频率 [Hz]
        double start_velocity = 0.0;       ///< T 曲线静止起步初速度；S 用 moveSCurve 参数
        double soft_stop_acc = 0.0;        ///< 仅 T 曲线；≤0 则用减速度
        double emergency_stop_acc = 0.0;   ///< 仅 T 曲线；≤0 则为减速度 ×10
    };

    /**
     * @brief 写入速度/加速度限制（NRT）
     * @param limits 限制
     */
    void setLimits(const Limits& limits) noexcept { limits_ = limits; }
    /** @brief 当前限制 */
    const Limits& limits() const noexcept { return limits_; }

    /** @brief 最近一次 moveSCurve / moveTCurve 所选轮廓 */
    ProfileType profileType() const noexcept {
        return static_cast<ProfileType>(type_.load(std::memory_order_acquire));
    }

    /**
     * @brief NRT：S 曲线点到点
     * @param start_velocity 起点速度 (units/s)
     * @param end_velocity 终点速度 (units/s)
     */
    bool moveSCurve(double start, double end,
                    double start_velocity = 0.0,
                    double end_velocity = 0.0) noexcept;

    /** @brief NRT：按完整 S 曲线配置规划 */
    bool moveSCurve(const SCurveConfig& cfg) noexcept;

    /**
     * @brief NRT：T 曲线点到点（终点速度为 0）
     *
     * 静止时从 `start` 起规划；运动中再调用则忽略 `start`，按当前速度重规划到 `end`
     *（刹停距离不够会过冲折返）。
     */
    bool moveTCurve(double start, double end,
                    double start_velocity = 0.0) noexcept;

    /** @brief RT：立即停止吐点，保持最后一次指令位置 */
    void abort() noexcept;

    /** @brief NRT：平滑停止。仅 T 曲线有效；S 曲线等同 abort */
    void softStop() noexcept;

    /** @brief NRT：紧急停止。仅 T 曲线有效；S 曲线等同 abort */
    void emergencyStop() noexcept;

    /**
     * @brief RT：推进 dt 秒并刷新指令
     * @return true 本周期应把 commandPosition() 下发到 CSP
     */
    bool update(double dt) noexcept;

    bool active() const noexcept {
        return running_.load(std::memory_order_acquire);
    }
    /** @brief 轨迹是否已结束 */
    bool finished() const noexcept {
        return finished_.load(std::memory_order_acquire);
    }

    /** @brief 本周期应下发的位置指令 */
    double commandPosition() const noexcept {
        return cmd_pos_.load(std::memory_order_relaxed);
    }
    /** @brief 本周期速度指令 */
    double commandVelocity() const noexcept {
        return cmd_vel_.load(std::memory_order_relaxed);
    }
    /** @brief 本周期加速度指令 */
    double commandAcceleration() const noexcept {
        return cmd_acc_.load(std::memory_order_relaxed);
    }
    /** @brief 已运行时间 [s] */
    double elapsed() const noexcept {
        return t_.load(std::memory_order_relaxed);
    }
    /** @brief 规划总时长 [s] */
    double totalTime() const noexcept {
        return total_time_.load(std::memory_order_relaxed);
    }

private:
    enum class StopReq : std::uint8_t { None = 0, Abort = 1, Soft = 2, Emergency = 3 };

    void publishSample(double pos, double vel, double acc) noexcept;
    void armPendingS(const SCurvePlanner& planned) noexcept;
    TCurveConfig makeTConfig(double start_velocity) const noexcept;
    bool updateS(double dt) noexcept;
    bool updateT(double dt) noexcept;

    Limits limits_{};

    std::atomic<std::uint8_t> type_{static_cast<std::uint8_t>(ProfileType::SCurve)};
    std::atomic<std::uint8_t> stop_req_{0};

    SCurvePlanner pending_s_;
    SCurvePlanner active_s_;
    std::atomic<bool> pending_s_ready_{false};

    TCurvePlanner tcurve_;
    std::atomic<bool> pending_t_ready_{false};
    double pending_t_target_{0.0};
    double pending_t_seed_pos_{0.0};
    bool pending_t_seed_{false};
    TCurveConfig pending_t_cfg_{};

    std::atomic<bool> running_{false};
    std::atomic<bool> finished_{true};

    std::atomic<double> t_{0.0};
    std::atomic<double> total_time_{0.0};
    std::atomic<double> cmd_pos_{0.0};
    std::atomic<double> cmd_vel_{0.0};
    std::atomic<double> cmd_acc_{0.0};
};
