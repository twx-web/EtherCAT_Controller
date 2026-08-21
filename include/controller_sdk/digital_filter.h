#pragma once

/**
 * @file digital_filter.h
 * @brief 数字滤波器（实现见 digital_filter.cpp）
 *
 * ## RT / NRT 契约
 * - **NRT**：`configure` / `reset`（`MovingAverageFilter::configure` 会改 deque）。
 * - **RT**：`update` / `value`（移动平均窗口须事先 configure，RT 不再扩容）。
 */

#include <cstdint>
#include <deque>
#include <cstddef>

/** @brief 一阶低通 */
class LowPassFilter {
public:
    LowPassFilter() = default;
    /**
     * @brief 配置时间常数
     * @param time_constant 时间常数 [s]
     * @param sample_freq 采样频率 [Hz]
     */
    void configure(double time_constant, double sample_freq) noexcept;
    /**
     * @brief 推入一个样本
     * @param input 输入
     * @return 滤波输出
     */
    double update(double input) noexcept;
    /**
     * @brief 复位输出
     * @param initial 初值
     */
    void reset(double initial = 0.0) noexcept;
    /** @brief 当前输出 */
    double value() const noexcept;

private:
    double alpha_ = 1.0;
    double output_ = 0.0;
};

/** @brief 二阶 Butterworth 低通 */
class ButterworthLPF {
public:
    ButterworthLPF() = default;
    /**
     * @brief 配置截止频率
     * @param cutoff_freq 截止频率 [Hz]
     * @param sample_freq 采样频率 [Hz]
     */
    void configure(double cutoff_freq, double sample_freq) noexcept;
    /**
     * @brief 推入一个样本
     * @param input 输入
     * @return 滤波输出
     */
    double update(double input) noexcept;
    /** @brief 复位状态 */
    void reset(double initial = 0.0) noexcept;

private:
    bool bypass_ = false;
    double b0_ = 1.0, b1_ = 0.0, b2_ = 0.0;
    double a1_ = 0.0, a2_ = 0.0;
    double x1_ = 0.0, x2_ = 0.0;
    double y1_ = 0.0, y2_ = 0.0;
};

/** @brief 陷波滤波 */
class NotchFilter {
public:
    NotchFilter() = default;
    /**
     * @brief 配置陷波点
     * @param notch_freq 陷波频率 [Hz]
     * @param q 品质因数
     * @param sample_freq 采样频率 [Hz]
     */
    void configure(double notch_freq, double q, double sample_freq) noexcept;
    /**
     * @brief 推入一个样本
     * @param input 输入
     * @return 滤波输出
     */
    double update(double input) noexcept;
    /** @brief 复位状态 */
    void reset(double initial = 0.0) noexcept;

private:
    bool bypass_ = false;
    double b0_ = 1.0, b1_ = 0.0, b2_ = 0.0;
    double a1_ = 0.0, a2_ = 0.0;
    double x1_ = 0.0, x2_ = 0.0;
    double y1_ = 0.0, y2_ = 0.0;
};

/** @brief 滑动平均（窗口在 configure 时确定） */
class MovingAverageFilter {
public:
    MovingAverageFilter() = default;
    /**
     * @brief 设置窗口长度（NRT，可能分配）
     * @param window_size 样本数
     */
    void configure(size_t window_size) noexcept;
    /**
     * @brief 推入一个样本
     * @param input 输入
     * @return 窗口均值
     */
    double update(double input) noexcept;
    /** @brief 复位窗口 */
    void reset(double initial = 0.0) noexcept;
    /** @brief 当前均值 */
    double value() const noexcept;

private:
    std::deque<double> buffer_;
    double sum_ = 0.0;
    size_t max_samples_ = 10;
};

/** @brief 位置差分 + 低通的速度估计 */
class VelocityEstimator {
public:
    VelocityEstimator() = default;
    /**
     * @brief 配置低通时间常数
     * @param lpf_tc 低通时间常数 [s]
     * @param sample_freq 采样频率 [Hz]
     */
    void configure(double lpf_tc, double sample_freq) noexcept;
    /**
     * @brief 由位置差分估计速度
     * @param position 位置
     * @param dt 周期 [s]
     * @return 滤波后速度
     */
    double update(double position, double dt) noexcept;
    /**
     * @brief 复位
     * @param position 当前位置
     * @param velocity 当前速度
     */
    void reset(double position = 0.0, double velocity = 0.0) noexcept;
    /** @brief 当前速度估计 */
    double value() const noexcept;

private:
    double last_position_ = 0.0;
    double last_velocity_ = 0.0;
    LowPassFilter lpf_;
};

/** @brief 一阶高通 */
class HighPassFilter {
public:
    HighPassFilter() = default;
    /**
     * @brief 配置时间常数
     * @param time_constant 时间常数 [s]
     * @param sample_freq 采样频率 [Hz]
     */
    void configure(double time_constant, double sample_freq) noexcept;
    /**
     * @brief 推入一个样本
     * @param input 输入
     * @return 滤波输出
     */
    double update(double input) noexcept;
    /** @brief 复位状态 */
    void reset(double initial = 0.0) noexcept;

private:
    bool bypass_ = false;
    double alpha_ = 0.5;
    double last_input_ = 0.0;
    double last_output_ = 0.0;
};
