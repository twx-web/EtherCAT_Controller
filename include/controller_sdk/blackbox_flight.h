#pragma once

/**
 * @file blackbox_flight.h
 * @brief 故障飞行记录：始终保留最近 N 秒环形预缓冲；闩锁上升沿自动落盘并续录事后段
 *
 * RT：仅 push()；NRT：poll() 检测上升沿并写文件。
 */

#include "blackbox_recorder.h"
#include "trace_buffer.h"

#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>

class BlackboxFlightRecorder {
public:
    struct Config {
        std::string dump_dir = "/tmp";               ///< 落盘目录
        std::string file_prefix = "fault_blackbox";  ///< 文件名前缀
        std::size_t prebuffer_capacity = 16384;      ///< 预缓冲容量（约 16s@1kHz）
        uint64_t post_fault_samples = 5000;          ///< 故障后再录样本数
        bool enabled = true;                         ///< 总开关
    };

    explicit BlackboxFlightRecorder(std::size_t prebuffer_capacity = 16384);

    /**
     * @brief 更新配置（NRT）
     * @param cfg 配置
     */
    void setConfig(const Config& cfg);
    /** @brief 当前配置副本 */
    Config config() const;

    /**
     * @brief 启用/关闭故障自动落盘
     * @param on true 启用
     */
    void setEnabled(bool on) noexcept {
        cfg_enabled_.store(on, std::memory_order_release);
    }
    /** @brief 是否启用 */
    bool enabled() const noexcept {
        return cfg_enabled_.load(std::memory_order_acquire);
    }

    /** @brief RT：写入预缓冲（满则覆盖最旧） */
    void push(const TraceSample& s) noexcept;

    /**
     * @brief NRT：根据 fault_latched 上升沿触发落盘；续写事后样本
     * @return 若本周期新开了故障文件则返回路径，否则空串
     */
    std::string poll(bool fault_latched, std::string& err);

    /** @brief 是否正在录故障事后段 */
    bool isPostCapturing() const noexcept {
        return post_remaining_.load(std::memory_order_acquire) > 0;
    }
    /** @brief 最近一次落盘路径 */
    std::string lastDumpPath() const;
    /** @brief 累计落盘次数 */
    uint64_t dumpCount() const noexcept {
        return dump_count_.load(std::memory_order_relaxed);
    }

private:
    bool beginDumpUnlocked(std::string& err);
    void drainPrebufferToDump();

    mutable std::mutex mu_;
    Config cfg_;
    TraceBuffer pre_;
    BlackboxRecorder dump_;
    std::atomic<bool> cfg_enabled_{true};
    bool was_latched_ = false;
    std::atomic<uint64_t> post_remaining_{0};
    std::atomic<uint64_t> dump_count_{0};
    std::string last_path_;
};
