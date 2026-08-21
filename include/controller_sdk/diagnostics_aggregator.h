#pragma once

/**
 * @file diagnostics_aggregator.h
 * @brief 诊断聚合（纯逻辑，无 EtherCAT 依赖，便于单测）
 *
 * NRT 调用：从 MotionCycle / RealtimeLoop / RobotFeedback / 各轴缓存收集 Inputs，
 * 生成 DiagnosticsSnapshot。不在 RT 路径内调用（会拼字符串）。
 */

#include "diagnostics_snapshot.h"

#include <cstddef>
#include <cstdint>
#include <vector>

class DiagnosticsAggregator {
public:
    /** @brief 聚合输入（由 collector 或测试填充） */
    struct Inputs {
        bool communication_ok = false;               ///< 通信门控
        bool fault_latched = false;                  ///< 通信闩锁
        bool wc_complete = false;                    ///< WKC 完整
        uint32_t working_counter = 0;                ///< 实际 WKC
        uint32_t expected_working_counter = 0;       ///< 期望 WKC；0=未知
        uint32_t consecutive_comm_failures = 0;      ///< 连续失败次数
        uint64_t rt_overrun_count = 0;               ///< RT 超时次数
        uint32_t dc_diff_ns = 0;                     ///< DC 差上界 [ns]
        bool sm2_sync_error = false;                 ///< 0x1C32:32
        bool sm3_sync_error = false;                 ///< 0x1C33:32

        bool servo_enabled = false;                  ///< 全使能
        bool any_axis_fault = false;                 ///< 轴故障
        double max_joint_following_error_rad = 0.0;  ///< 最大关节跟随误差
        double cartesian_following_error_mm = 0.0;   ///< 笛卡尔跟随误差

        std::vector<AxisDiag> axes;                  ///< 各轴
    };

    /**
     * @brief 由输入构造快照并生成 summary（NRT，拼字符串）
     * @param in 聚合输入
     * @return 完整快照
     */
    static DiagnosticsSnapshot build(const Inputs& in);
};
