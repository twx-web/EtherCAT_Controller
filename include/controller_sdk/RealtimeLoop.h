#pragma once

/**
 * @file RealtimeLoop.h
 * @brief 定周期实时线程：SCHED_FIFO + 周期补偿
 *
 * ## RT / NRT 契约
 * - **NRT**：`setConfig` / `addTask` / `start` / `stop` / `lastError`。
 * - **RT**：任务回调内只做实时工作；`overrunCount` / `lastExecNs` 可读。
 */

#include <functional>
#include <atomic>
#include <thread>
#include <cstdint>
#include <string>
#include <mutex>
#include <vector>
#include <condition_variable>

/**
 * @brief 实时循环执行器
 *
 * 创建一个高优先级实时线程，以指定周期重复执行用户注册的任务。
 * 特性：
 * - 使用 SCHED_FIFO 实时调度
 * - 内存锁定 (mlockall)
 * - 可绑定到指定 CPU 核心
 * - CLOCK_MONOTONIC 周期补偿，无累积误差
 * - start() 会等待线程完成实时环境初始化，失败时返回 false
 * - 支持多个任务回调
 */
class RealtimeLoop {
public:
    /** @brief 任务回调类型 */
    using Task = std::function<void()>;

    /** @brief 配置参数 */
    struct Config {
        uint64_t cycle_ns = 1000000;  ///< 周期 [ns]，默认 1 ms
        int priority = 99;            ///< 实时优先级 (1–99)
        int cpu_affinity = -1;        ///< 绑定 CPU 核心；-1 不绑定
        /**
         * SCHED_FIFO / mlockall 失败时是否降级为普通线程周期循环。
         * 开发机 / 无实时权限时置 true；硬实时控机保持 false。
         */
        bool allow_soft_realtime = false;
    };

    RealtimeLoop() = default;
    ~RealtimeLoop();

    // 禁止拷贝
    RealtimeLoop(const RealtimeLoop&) = delete;
    RealtimeLoop& operator=(const RealtimeLoop&) = delete;

    /**
     * @brief 设置配置
     * @param cfg 配置参数
     */
    void setConfig(const Config& cfg);

    /**
     * @brief 添加一个周期性任务（线程安全，需在 start 前调用）
     * @param task 无参可调用对象
     */
    void addTask(Task task);

    /**
     * @brief 启动实时线程
     * @return true 实时环境就绪且循环已开始；false 调度/锁内存失败（此时不会空转）
     */
    bool start();

    /**
     * @brief 请求停止并等待线程退出
     */
    void stop();

    /**
     * @brief 检查是否正在运行
     */
    bool isRunning() const;

    /**
     * @brief 获取最后一次错误信息
     */
    std::string lastError() const;

    /**
     * @brief 累计周期过载次数（任务执行超过周期）
     */
    uint64_t overrunCount() const noexcept { return overrun_count_.load(std::memory_order_relaxed); }

    /** @brief 最近一周期任务执行耗时 (ns)，轻量统计无堆分配 */
    uint64_t lastExecNs() const noexcept { return last_exec_ns_.load(std::memory_order_relaxed); }
    /** @brief 历史最大执行耗时 (ns) */
    uint64_t maxExecNs() const noexcept { return max_exec_ns_.load(std::memory_order_relaxed); }
    /** @brief 本周期计划唤醒时刻 (CLOCK_MONOTONIC ns)，供 DC application_time */
    uint64_t lastWakeupNs() const noexcept { return wakeup_ns_.load(std::memory_order_relaxed); }

private:
    void threadFunc();
    void setError(const std::string& error);

    Config config_;
    std::vector<Task> tasks_;
    std::atomic<bool> running_{false};
    std::thread worker_;
    mutable std::mutex error_mutex_;
    std::string last_error_;

    // start() 与工作线程的握手：就绪 / 失败
    std::mutex start_mutex_;
    std::condition_variable start_cv_;
    bool start_ready_ = false;
    bool start_ok_ = false;

    std::atomic<uint64_t> overrun_count_{0};
    std::atomic<uint64_t> last_exec_ns_{0};
    std::atomic<uint64_t> max_exec_ns_{0};
    std::atomic<uint64_t> wakeup_ns_{0};
};
