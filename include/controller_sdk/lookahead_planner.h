#pragma once

/**
 * @file lookahead_planner.h
 * @brief 轨迹前瞻规划器（实现 ITrajectoryPlanner）
 *
 * 管理多段连续路径，自动计算拐角约束速度，执行双向速度规划，
 * 拐角处插入过渡段，段内用 S 曲线消费 entry/exit 速度，输出统一时间轴采样。
 *
 * plan/clear/compute = NRT；sampler() 回调 = RT。
 */

#include "i_trajectory_planner.h"
#include "scurve_planner.h"

#include <vector>
#include <functional>

/**
 * @brief 单段路径描述
 */
struct PathSegment {
    TrajectoryType type = TrajectoryType::LINEAR;  ///< 段类型
    CartesianPose start{};                         ///< 起点
    CartesianPose end{};                           ///< 终点
    double max_speed = 50.0;                       ///< 段内最大速度
    double max_accel = 200.0;                      ///< 段内最大加速度
    double duration = 0.0;                         ///< 规划后时长 [s]
    CartesianPose center{};                        ///< 圆弧圆心
    bool clockwise = false;                        ///< 圆弧方向
};

/**
 * @brief 带速度约束的轨迹段（前瞻内部）
 */
struct ConstrainedSegment {
    PathSegment seg;
    double entry_speed = 0.0;
    double exit_speed  = 0.0;
    double max_speed = 50.0;
    double max_accel = 200.0;
    double length = 0.0;
};

/**
 * @class LookaheadPlanner
 * @brief 多段笛卡尔路径前瞻规划器
 */
class LookaheadPlanner : public ITrajectoryPlanner {
public:
    /**
     * @brief 追加一段路径（NRT；之后须 `computeLookahead` 或 `planPath`）
     * @param seg 路径段
     */
    void addSegment(const PathSegment& seg);
    /** @brief 清空路径（NRT） */
    void clear() override;

    /**
     * @brief 执行前瞻规划
     * @param corner_error_max 允许轮廓误差 (mm)
     * @param blend_radius     >0 时在拐角插入弦线过渡（倒角）；<=0 不插过渡
     */
    void computeLookahead(double corner_error_max = 0.05, double blend_radius = 5.0);

    /** @brief ITrajectoryPlanner：由路径点序列规划 */
    bool planPath(const std::vector<CartesianPose>& waypoints,
                  double max_speed,
                  double max_accel,
                  double corner_error_max = 0.5,
                  double blend_radius = 5.0) override;

    /** @brief 采样函数 f(t) -> TrajectorySample，t 为全局时间 (s) */
    std::function<TrajectorySample(double)> getSampler() const;
    TrajectorySampler sampler() const override { return getSampler(); }

    double totalTime() const noexcept override { return total_time_; }
    bool isReady() const noexcept override { return static_cast<bool>(sampler_); }

private:
    double computeCornerSpeed(const PathSegment& seg1, const PathSegment& seg2,
                              double max_accel, double corner_error_max) const;
    void backwardSpeedPlan(std::vector<ConstrainedSegment>& segments) const;
    void insertBlendSegments(std::vector<ConstrainedSegment>& segments,
                             double blend_radius, double corner_error_max);

    std::vector<PathSegment> segments_;
    std::function<TrajectorySample(double)> sampler_;
    double total_time_ = 0.0;
};
