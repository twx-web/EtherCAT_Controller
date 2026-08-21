#pragma once

/**
 * @file i_trajectory_planner.h
 * @brief 笛卡尔路径规划抽象
 *
 * ## RT / NRT 契约
 * - **NRT**：`clear` / `planPath` / 配置——允许分配与求解。
 * - **RT**：`sampler()` 返回的函数对象——仅查表/插值，禁止分配与阻塞。
 *
 * 热路径统一为 `TrajectorySampler`（`f(t) -> TrajectorySample`）。
 * 默认多段实现：`LookaheadPlanner`。
 */

#include "trajectory_planner.h"

#include <functional>
#include <vector>

/** @brief 时间轴采样器：输入绝对/局部时间 t(s)，输出位姿采样（RT 可调用） */
using TrajectorySampler = std::function<TrajectorySample(double)>;

/**
 * @class ITrajectoryPlanner
 * @brief 笛卡尔多段路径规划接口（NRT 求解，RT 采样）
 */
class ITrajectoryPlanner {
public:
    virtual ~ITrajectoryPlanner() = default;

    /** @brief 清空已规划轨迹（NRT） */
    virtual void clear() = 0;

    /**
     * @brief 规划多段连续路径（NRT）
     * @param waypoints 路径点（至少 2 个；调用方负责插入当前起点）
     * @param max_speed 最大路径速度
     * @param max_accel 最大路径加速度
     * @param corner_error_max 拐角允许误差（默认 0.5）
     * @param blend_radius 交融半径（默认 5.0，单位与位姿位置一致，通常 mm）
     * @return true 规划成功，false 失败（点数不足或求解失败）
     */
    virtual bool planPath(const std::vector<CartesianPose>& waypoints,
                          double max_speed,
                          double max_accel,
                          double corner_error_max = 0.5,
                          double blend_radius = 5.0) = 0;

    /**
     * @brief 取得轨迹采样器（RT 可调用返回的函数对象）
     * @return `f(t) -> TrajectorySample`；未规划时行为由实现定义
     */
    virtual TrajectorySampler sampler() const = 0;

    /**
     * @brief 整条路径预计时长
     * @return 秒；未规划时通常为 0
     */
    virtual double totalTime() const noexcept = 0;

    /**
     * @brief 是否已有可用轨迹
     * @return true 可对 `sampler()` 采样
     */
    virtual bool isReady() const noexcept = 0;
};
