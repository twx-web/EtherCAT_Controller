#pragma once

/**
 * @file trace_buffer.h
 * @brief RT 安全 SPSC 环形录波缓冲（生产者 1：实时线程；消费者 1：UI/日志）
 *
 * 样本为 POD；容量为 2 的幂。满时覆盖最旧数据（overwritable）。
 *
 * **1.x 录波最多 8 轴**（`kTraceMaxAxes`）。超过静默截断，不报错。
 * `StartSyncMeter` / `ecs_*` 是 16 轴；扩数组留给 2.0。
 */

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <vector>

/** 1.x 录波关节通道上限；超过静默截断 */
constexpr uint8_t kTraceMaxAxes = 8;

/** @brief 单周期录波样本（固定布局，便于跨线程拷贝） */
struct TraceSample {
    double t = 0.0;                 ///< 时间 (s)
    uint64_t exec_ns = 0;           ///< 本周期执行耗时
    uint8_t axis_count = 0;         ///< 有效轴数（最多 `kTraceMaxAxes`）
    double actual_rad[kTraceMaxAxes]{};          ///< 实际关节角 [rad]
    double target_rad[kTraceMaxAxes]{};          ///< 目标关节角 [rad]
    double follow_err_rad[kTraceMaxAxes]{};      ///< 跟随误差 [rad]
    double actual_vel_rad_s[kTraceMaxAxes]{};   ///< 实际关节速度 (rad/s)，差分或编码器
    double target_vel_rad_s[kTraceMaxAxes]{};   ///< 目标关节速度 (rad/s)
    double actual_acc_rad_s2[kTraceMaxAxes]{};  ///< 实际关节加速度 (rad/s²)
    double cart_x = 0;              ///< 笛卡尔 X [mm]
    double cart_y = 0;              ///< 笛卡尔 Y [mm]
    double cart_z = 0;              ///< 笛卡尔 Z [mm]
    double cart_vx = 0, cart_vy = 0, cart_vz = 0;  ///< 笛卡尔速度 (mm/s)
    double cart_speed = 0;          ///< 路径速度 |v| (mm/s)
    double cart_acc = 0;            ///< 路径加速度 (mm/s²)
    double cart_err_mm = 0;         ///< 笛卡尔位置误差 [mm]
    uint32_t flags = 0;             ///< bit0 enabled, bit1 fault, bit2 comm_ok
    uint32_t working_counter = 0;   ///< EtherCAT Domain 实际 WC（Mock=0）
    uint32_t expected_working_counter = 0; ///< 期望 WKC（0=未知）
};

/** @brief 录波差分状态（会话内持有，用于速度/加速度） */
struct TraceDerivState {
    bool valid = false;                 ///< 是否已有上一拍
    double actual_rad[kTraceMaxAxes]{};             ///< 上一拍实际角
    double target_rad[kTraceMaxAxes]{};             ///< 上一拍目标角
    double actual_vel_rad_s[kTraceMaxAxes]{};       ///< 上一拍实际速度
    double cart_x = 0;                  ///< 上一拍 X
    double cart_y = 0;                  ///< 上一拍 Y
    double cart_z = 0;                  ///< 上一拍 Z
    double cart_speed = 0;              ///< 上一拍路径速度
};

class TraceBuffer {
public:
    /**
     * @brief 构造环形缓冲
     * @param capacity_pow2 容量，须为 2 的幂；否则实现会调整
     */
    explicit TraceBuffer(std::size_t capacity_pow2 = 16384);

    /** @brief 清空读写指针（不释放存储） */
    void reset() noexcept;

    /**
     * @brief RT：写入一条；满则覆盖最旧
     * @param s 样本
     */
    void push(const TraceSample& s) noexcept;

    /**
     * @brief NRT：拷出最多 max_out 条新样本
     * @param out 输出缓冲
     * @param max_out 最大条数
     * @return 实际条数
     */
    std::size_t pop(TraceSample* out, std::size_t max_out) noexcept;

    /** @brief 容量 */
    std::size_t capacity() const noexcept { return capacity_; }
    /** @brief 大约可读条数（无锁近似） */
    std::size_t sizeApprox() const noexcept;

private:
    std::size_t capacity_ = 0;
    std::size_t mask_ = 0;
    std::vector<TraceSample> buf_;
    std::atomic<uint64_t> write_{0};
    std::atomic<uint64_t> read_{0};
};
