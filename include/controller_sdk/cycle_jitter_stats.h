#pragma once

/**
 * @file cycle_jitter_stats.h
 * @brief 周期耗时滑动窗口统计（NRT）：p50 / p95 / p99 / max
 *
 * 供 Studio 总线页等使用。内部会排序拷贝，**不要在 RT 线程调用**。
 */

#include <algorithm>
#include <cstdint>
#include <vector>

/**
 * @class CycleJitterStats
 * @brief 固定窗口内执行耗时百分位
 */
class CycleJitterStats {
public:
    /**
     * @brief 构造
     * @param window 窗口长度（样本数）；0 视为 2048
     */
    explicit CycleJitterStats(std::size_t window = 2048) : window_(window ? window : 2048) {
        buf_.reserve(window_);
    }

    /** @brief 清空窗口与最大值 */
    void clear() {
        buf_.clear();
        write_ = 0;
        max_ns_ = 0;
        count_ = 0;
        dirty_ = true;
    }

    /**
     * @brief 追加一条执行耗时
     * @param exec_ns 本周期耗时 [ns]
     */
    void push(uint64_t exec_ns) {
        if (buf_.size() < window_)
            buf_.push_back(exec_ns);
        else
            buf_[write_ % window_] = exec_ns;
        ++write_;
        if (exec_ns > max_ns_)
            max_ns_ = exec_ns;
        ++count_;
        dirty_ = true;
    }

    /** @brief 累计推入次数（含被覆盖的） */
    uint64_t sampleCount() const { return count_; }
    /** @brief 窗口出现过的最大耗时 [ns] */
    uint64_t maxNs() const { return max_ns_; }

    /** @brief 50 分位耗时 [ns] */
    uint64_t p50Ns() const { return percentile(0.50); }
    /** @brief 95 分位耗时 [ns] */
    uint64_t p95Ns() const { return percentile(0.95); }
    /** @brief 99 分位耗时 [ns] */
    uint64_t p99Ns() const { return percentile(0.99); }

    /**
     * @brief 最近一条耗时
     * @return [ns]；无样本为 0
     */
    uint64_t lastNs() const {
        if (buf_.empty())
            return 0;
        if (buf_.size() < window_)
            return buf_.back();
        return buf_[(write_ + window_ - 1) % window_];
    }

private:
    uint64_t percentile(double p) const {
        if (buf_.empty())
            return 0;
        if (dirty_) {
            sorted_ = buf_;
            std::sort(sorted_.begin(), sorted_.end());
            dirty_ = false;
        }
        const auto n = sorted_.size();
        const auto i = static_cast<std::size_t>(p * static_cast<double>(n - 1));
        return sorted_[i];
    }

    std::size_t window_ = 2048;
    std::size_t write_ = 0;
    std::vector<uint64_t> buf_;
    mutable std::vector<uint64_t> sorted_;
    mutable bool dirty_ = true;
    uint64_t max_ns_ = 0;
    uint64_t count_ = 0;
};
