#pragma once

/**
 * @file scurve_planner.h
 * @brief S 曲线加减速规划器（7段式，工业级实时安全）
 *
 * 支持：
 * - 对称/非对称加加速度限制
 * - 自动判断是否达到最大速度/加速度
 * - 实时查询任意时刻的位置、速度、加速度、加加速度
 * - 可选起止速度（可用于多段路径衔接）
 * - 所有实时路径 noexcept
 */

#include <cstdint>
#include <cmath>
#include <vector>

/**
 * @brief S 曲线配置参数
 */
struct SCurveConfig {
    double max_velocity = 100.0;       ///< 最大速度 (units/s)
    double max_acceleration = 500.0;   ///< 最大加速度 (units/s²)
    double max_jerk = 5000.0;          ///< 最大加加速度 (units/s³)

    double start_velocity = 0.0;       ///< 起点速度 (units/s)
    double end_velocity = 0.0;         ///< 终点速度 (units/s)

    double position_start = 0.0;       ///< 起点位置 (units)
    double position_end = 100.0;       ///< 终点位置 (units)

    double frequency = 1000.0;         ///< 控制频率 (Hz)，用于周期计算
};

/**
 * @brief S 曲线7段时长
 */
struct SCurvePhases {
    double T1 = 0.0;  ///< 加加速段 (s)
    double T2 = 0.0;  ///< 匀加速段 (s)
    double T3 = 0.0;  ///< 减加速段 (s)
    double T4 = 0.0;  ///< 匀速段 (s)
    double T5 = 0.0;  ///< 加减速段 (s)
    double T6 = 0.0;  ///< 匀减速段 (s)
    double T7 = 0.0;  ///< 减减速段 (s)

    double total() const { return T1 + T2 + T3 + T4 + T5 + T6 + T7; }
    bool isValid() const {
        return T1 >= 0 && T2 >= 0 && T3 >= 0 && T4 >= 0 &&
               T5 >= 0 && T6 >= 0 && T7 >= 0;
    }
};

/**
 * @brief S 曲线在某时刻的采样值
 */
struct SCurveSample {
    double position = 0.0;     ///< 位置
    double velocity = 0.0;     ///< 速度
    double acceleration = 0.0; ///< 加速度
    double jerk = 0.0;         ///< 加加速度
    double progress = 0.0;     ///< 进度 [0,1]
    bool finished = false;     ///< 是否已完成
};

/**
 * @class SCurvePlanner
 * @brief S 曲线加减速规划器
 *
 * 使用示例：
 * @code
 * SCurveConfig cfg;
 * cfg.max_velocity = 100;
 * cfg.max_acceleration = 500;
 * cfg.max_jerk = 5000;
 * cfg.position_start = 0;
 * cfg.position_end = 200;
 * cfg.frequency = 1000;
 *
 * SCurvePlanner planner;
 * bool ok = planner.plan(cfg);
 * // 实时循环中：
 * auto sample = planner.sample(t);
 * @endcode
 */
class SCurvePlanner {
public:
    SCurvePlanner() = default;

    /**
     * @brief 执行 S 曲线规划
     * @param cfg 配置参数
     * @return true 规划成功，false 参数无效或无法到达
     */
    bool plan(const SCurveConfig& cfg) noexcept;

    /**
     * @brief 对给定时间 t 采样
     * @param t 时间 (s)，从 0 开始
     * @return 采样结果
     */
    SCurveSample sample(double t) const noexcept;

    /**
     * @brief 获取总时长
     * @return 总时长 (s)，规划失败返回 0
     */
    double totalTime() const noexcept { return phases_.total(); }

    /**
     * @brief 获取各阶段时长（只读）
     */
    const SCurvePhases& phases() const noexcept { return phases_; }

    /**
     * @brief 获取配置（只读）
     */
    const SCurveConfig& config() const noexcept { return config_; }

    /**
     * @brief 检查规划是否有效
     */
    bool isValid() const noexcept { return valid_; }

    /**
     * @brief 清除规划状态
     */
    void reset() noexcept;

private:
    // 计算各阶段时长（核心算法）；ok=false 表示起止速度相对位移不可达
    SCurvePhases computePhases(const SCurveConfig& cfg, bool& ok) const noexcept;

    // 根据时间 t 计算所在阶段的位置
    double positionAtTime(double t, const SCurvePhases& phases,
                          const SCurveConfig& cfg) const noexcept;

    // 根据时间 t 计算速度
    double velocityAtTime(double t, const SCurvePhases& phases,
                          const SCurveConfig& cfg) const noexcept;

    // 根据时间 t 计算加速度
    double accelerationAtTime(double t, const SCurvePhases& phases,
                              const SCurveConfig& cfg) const noexcept;

    // 根据时间 t 计算加加速度
    double jerkAtTime(double t, const SCurvePhases& phases,
                      const SCurveConfig& cfg) const noexcept;

    SCurveConfig config_;
    SCurvePhases phases_;
    bool valid_ = false;
};

/**
 * @brief 辅助：生成多段连续 S 曲线规划结果的时间轴
 */
struct MultiSegmentSCurve {
    struct Segment {
        SCurvePlanner planner;
        double global_start_time = 0.0;  // 全局起始时间
    };

    std::vector<Segment> segments;

    /**
     * @brief 在全局时间 t 下采样
     */
    SCurveSample sample(double t) const noexcept {
        for (const auto& seg : segments) {
            double local_t = t - seg.global_start_time;
            if (local_t >= 0 && local_t <= seg.planner.totalTime()) {
                auto s = seg.planner.sample(local_t);
                s.finished = false;
                return s;
            }
        }
        if (!segments.empty()) {
            auto s = segments.back().planner.sample(
                segments.back().planner.totalTime());
            s.finished = true;
            return s;
        }
        return SCurveSample{};
    }

    double totalTime() const noexcept {
        if (segments.empty()) return 0.0;
        const auto& last = segments.back();
        return last.global_start_time + last.planner.totalTime();
    }
};
