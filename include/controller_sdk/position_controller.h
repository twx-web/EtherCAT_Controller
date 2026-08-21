#pragma once

/**
 * @file position_controller.h
 * @brief 主站侧位置环 PID（实现见 position_controller.cpp）
 *
 * ## RT / NRT 契约
 * - **NRT**：`configure` / `reset`。
 * - **RT**：`update`（无堆分配）。
 */

#include <cstdint>

/** @brief PID + 前馈参数 */
struct PIDConfig {
    double kp = 1.0;                 ///< 比例
    double ki = 0.0;                 ///< 积分
    double kd = 0.0;                 ///< 微分
    double kff_velocity = 0.0;       ///< 速度前馈
    double kff_acceleration = 0.0;   ///< 加速度前馈

    double output_max = 1e6;         ///< 输出上限
    double output_min = -1e6;        ///< 输出下限

    double integral_limit = 1e6;     ///< 积分限幅
    double deadband = 0.0;           ///< 误差死区

    double df_cutoff = 0.0;          ///< 微分滤波截止 [Hz]；0=不滤波
    double sample_freq = 1000.0;     ///< 采样频率 [Hz]
};

/**
 * @class PositionController
 * @brief 位置 PID：目标/实际位置 → 控制量（单位由调用方约定）
 */
class PositionController {
public:
    PositionController() = default;

    /**
     * @brief 写入 PID 参数（NRT）
     * @param cfg 配置
     */
    void configure(const PIDConfig& cfg) noexcept;
    /**
     * @brief 计算本周期输出（RT）
     * @param target_position 目标位置
     * @param actual_position 实际位置
     * @param target_velocity 目标速度（前馈）
     * @param target_acceleration 目标加速度（前馈）
     * @param dt 周期 [s]
     * @return 限幅后的控制量
     */
    double update(double target_position, double actual_position,
                  double target_velocity = 0.0,
                  double target_acceleration = 0.0,
                  double dt = 0.001) noexcept;
    /** @brief 清零积分与微分状态 */
    void reset() noexcept;
    /** @brief 当前配置 */
    const PIDConfig& config() const noexcept;
    /** @brief 当前积分项 */
    double integral() const noexcept;

private:
    PIDConfig cfg_;
    double integral_ = 0.0;
    double last_error_ = 0.0;
    double filtered_derivative_ = 0.0;
    double alpha_df_ = 1.0;
};
