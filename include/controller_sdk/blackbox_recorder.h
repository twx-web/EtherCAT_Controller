#pragma once

/**
 * @file blackbox_recorder.h
 * @brief 黑盒录波落盘（NRT）：将 TraceSample 写成 CSV，支持滚动截断
 *
 * RT 线程只应 push 到独立 TraceBuffer；由 NRT 线程 pop 后调用本类。
 * 禁止在实时上下文调用 start/stop/writeSamples。
 */

#include "trace_buffer.h"

#include <atomic>
#include <cstdint>
#include <fstream>
#include <mutex>
#include <string>

/** @brief 黑盒录波器（线程安全；文件 IO 仅 NRT） */
class BlackboxRecorder {
public:
    struct Config {
        std::string path;                 ///< 输出 CSV 路径
        uint64_t max_samples = 120000;    ///< 单文件最大样本（约 120s@1kHz）；0=不限
        bool write_header = true;
    };

    BlackboxRecorder() = default;
    ~BlackboxRecorder();

    BlackboxRecorder(const BlackboxRecorder&) = delete;
    BlackboxRecorder& operator=(const BlackboxRecorder&) = delete;

    /** @brief 开始录制；失败写 err */
    bool start(const Config& cfg, std::string& err);

    /** @brief 停止并关闭文件 */
    void stop();

    bool isRecording() const noexcept {
        return recording_.load(std::memory_order_acquire);
    }

    /**
     * @brief 追加样本；达到 max_samples 时截断文件重写头（滚动窗口）
     * @return 实际写入条数
     */
    std::size_t writeSamples(const TraceSample* samples, std::size_t n);

    uint64_t samplesWritten() const noexcept {
        return samples_written_.load(std::memory_order_relaxed);
    }

    std::string path() const;

private:
    void writeHeaderUnlocked();
    void writeOneUnlocked(const TraceSample& s);
    bool reopenTruncatedUnlocked(std::string& err);

    mutable std::mutex mu_;
    Config cfg_;
    std::ofstream out_;
    std::atomic<bool> recording_{false};
    std::atomic<uint64_t> samples_written_{0};
    uint64_t samples_in_file_ = 0;
};
