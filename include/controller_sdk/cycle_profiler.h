#pragma once

/**
 * @file cycle_profiler.h
 * @brief 实时循环性能分析器（实现见 cycle_profiler.cpp）
 *
 * ## RT / NRT 契约
 * 内部有 `vector` 分配，**硬 RT 热路径不要用**，改用 `RealtimeLoop::lastExecNs()`。
 * 软实时 / 调试周期可以 `begin`/`end`。
 *
 * 多轴启动同步：`start_sync`（Δcmd / Δact，分辨率 = 一个控制周期）。
 */

#include <cstdint>
#include <vector>
#include <string>
#include <ctime>
#include <cstddef>

#include "start_sync_meter.h"

/**
 * @class CycleProfiler
 * @brief 统计任务执行时间、周期抖动与分阶段耗时（调试用）
 */
class CycleProfiler {
public:
    CycleProfiler();

    /** @brief 周期开始：记录起点并统计与上一拍间隔 */
    void begin();
    /**
     * @brief 相对本周期 `begin` 的耗时
     * @return 纳秒
     */
    uint64_t mark();
    /** @brief 周期结束：累计执行时间；每 5000 拍打印报告 */
    void end();
    /**
     * @brief 累加一个命名阶段的耗时
     * @param name 阶段名
     * @param elapsed_ns 耗时 [ns]
     */
    void addPhase(const std::string& name, uint64_t elapsed_ns);
    /** @brief 清零统计与 `start_sync` */
    void reset();

    /** @brief 打印执行时间 / 周期抖动 / 分阶段 / 启动同步（NRT 或调试周期） */
    void printReport();
    /** @brief 与 printReport 相同内容，供 GUI 日志 */
    std::string reportText() const;

    /** @brief 多轴同一拍启动时间差（RT：`update` / `markCommand`） */
    StartSyncMeter start_sync;

private:
    double calcStddev(const std::vector<uint64_t>& data, double mean) const;
    uint64_t diff_ns(const struct timespec& start, const struct timespec& end) const;

    struct timespec start_{};
    struct timespec last_start_{};

    std::vector<uint64_t> exec_history_;
    uint64_t min_exec_ns_ = UINT64_MAX;
    uint64_t max_exec_ns_ = 0;
    uint64_t total_exec_ns_ = 0;
    size_t exec_count_ = 0;

    std::vector<uint64_t> period_history_;
    uint64_t min_period_ns_ = UINT64_MAX;
    uint64_t max_period_ns_ = 0;
    uint64_t total_period_ns_ = 0;
    size_t period_count_ = 0;

    std::vector<std::string> phase_names_;
    std::vector<uint64_t> phase_totals_;
    std::vector<size_t> phase_counts_;
};
