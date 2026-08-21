#pragma once

/**
 * @file pvt_interpolator.h
 * @brief PVT / PT 插补器（实现见 pvt_interpolator.cpp）
 *
 * ## RT / NRT 契约
 * - **NRT**：`loadPoints` / `loadPT` / `clear`（分配表）。
 * - **RT**：`update` / `velocity` / `totalTime`（查表，无分配）。
 */

#include <vector>
#include <cstdint>
#include <cstddef>

/** @brief PVT 节点 */
struct PVTNode {
    double position = 0.0;        ///< 位置
    double velocity = 0.0;        ///< 速度
    double time = 0.0;            ///< 段时间或相对时间 [s]
    double absolute_time = 0.0;   ///< 绝对时间 [s]
};

/** @brief 一段三次多项式（内部） */
struct PVTSegment {
    double t0 = 0.0;   ///< 段起始绝对时间
    double t1 = 0.0;   ///< 段结束绝对时间
    double dt = 0.0;   ///< 段时长
    double p0 = 0.0;   ///< 起点位置
    double p1 = 0.0;   ///< 终点位置
    double v0 = 0.0;   ///< 起点速度
    double v1 = 0.0;   ///< 终点速度
    double a = 0.0, b = 0.0, c = 0.0, d = 0.0;  ///< 多项式系数
};

/**
 * @class PVTInterpolator
 * @brief 单轴 PVT/PT 插补
 */
class PVTInterpolator {
public:
    PVTInterpolator() = default;

    /**
     * @brief 加载 PVT 点列（NRT）
     * @param nodes 节点（时间须递增）
     * @return true 成功
     */
    bool loadPoints(const std::vector<PVTNode>& nodes) noexcept;
    /**
     * @brief 加载 PT 点列（速度由实现估计）
     * @param positions 位置
     * @param times 对应时间 [s]
     * @return true 成功
     */
    bool loadPT(const std::vector<double>& positions,
                const std::vector<double>& times) noexcept;
    /**
     * @brief 在时刻 t 采样位置（RT）
     * @param t 时间 [s]
     * @return 位置
     */
    double update(double t) const noexcept;
    /**
     * @brief 在时刻 t 采样速度（RT）
     * @param t 时间 [s]
     * @return 速度
     */
    double velocity(double t) const noexcept;
    /** @brief 总时长 [s] */
    double totalTime() const noexcept;
    /** @brief 清空（NRT） */
    void clear() noexcept;
    /** @brief 节点数 */
    size_t size() const noexcept;
    /** @brief 只读节点表 */
    const std::vector<PVTNode>& nodes() const noexcept;

private:
    std::vector<PVTNode> nodes_;
    std::vector<PVTSegment> segments_;
    double total_time_ = 0.0;
};

/**
 * @class MultiAxisPVT
 * @brief 多轴共享时间轴的 PVT
 */
class MultiAxisPVT {
public:
    MultiAxisPVT() = default;

    /**
     * @brief 加载各轴位置（及可选速度）
     * @param axis_positions 每轴一条位置序列
     * @param times 共享时间戳
     * @param velocities 可选每轴速度；空则按 PT 估计
     * @return true 成功
     */
    bool loadPoints(const std::vector<std::vector<double>>& axis_positions,
                    const std::vector<double>& times,
                    const std::vector<std::vector<double>>& velocities = {}) noexcept;
    /**
     * @brief 在时刻 t 采样各轴位置（RT，返回 vector 可能分配）
     * @param t 时间 [s]
     * @return 各轴位置
     */
    std::vector<double> update(double t) const noexcept;
    /** @brief 总时长 [s] */
    double totalTime() const noexcept;
    /** @brief 清空 */
    void clear() noexcept;
    /** @brief 轴数 */
    size_t numAxes() const noexcept;

private:
    std::vector<PVTInterpolator> interpolators_;
};
