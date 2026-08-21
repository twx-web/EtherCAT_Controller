#pragma once

/**
 * @file diagnostics_snapshot.h
 * @brief 统一诊断快照（纯数据，可跨线程拷贝后给 GUI/日志）
 *
 * ## RT / NRT 契约
 * 结构体本身可拷贝；`summary` 与 `axes` 含堆分配，在 **NRT** 填充（见
 * `DiagnosticsAggregator` / `collectDiagnostics`）。
 */

#include <cstdint>
#include <string>
#include <vector>

/** @brief 单轴诊断（来自 PDO 缓存 / 仿真） */
struct AxisDiag {
    uint8_t error_code = 0;          ///< CiA402Motor::ErrorCode 底层值
    uint16_t status_word = 0;        ///< 状态字 0x6041
    bool enabled = false;            ///< 已使能
    bool fault = false;              ///< 故障
    bool homing_active = false;      ///< 回零进行中
    bool homing_complete = false;    ///< 回零完成
    bool homing_error = false;       ///< 回零错误/超时
    double following_error_rad = 0.0;///< 跟随误差 [rad]
};

/** @brief 整机诊断快照：通信、实时、运动与各轴 */
struct DiagnosticsSnapshot {
    bool communication_ok = false;                 ///< 通信门控通过
    bool fault_latched = false;                    ///< 通信故障闩锁
    bool wc_complete = false;                      ///< Domain WKC 完整
    uint32_t working_counter = 0;                  ///< 实际 WKC
    uint32_t expected_working_counter = 0;         ///< 期望 WKC；0=未知
    uint32_t consecutive_comm_failures = 0;        ///< 连续通信失败次数
    uint64_t rt_overrun_count = 0;                 ///< RT 周期超时次数
    uint32_t dc_diff_ns = 0;                       ///< DC 同步差上界 [ns]；0=未知/未开 DC
    bool sm2_sync_error = false;                   ///< 0x1C32:32（NRT 采样）
    bool sm3_sync_error = false;                   ///< 0x1C33:32（NRT 采样）

    bool servo_enabled = false;                    ///< 伺服是否全使能
    bool any_axis_fault = false;                   ///< 是否存在轴故障
    double max_joint_following_error_rad = 0.0;    ///< 最大关节跟随误差 [rad]
    double cartesian_following_error_mm = 0.0;     ///< 笛卡尔跟随误差 [mm]

    std::vector<AxisDiag> axes;                    ///< 各轴诊断
    std::string summary;                           ///< 人类可读摘要（NRT 填充）
};
