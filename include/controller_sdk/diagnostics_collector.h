#pragma once

/**
 * @file diagnostics_collector.h
 * @brief 从 MotionCycle / RealtimeLoop / IAxisGroup / RobotFeedback 一键采集诊断
 *
 * NRT 调用。空指针参数表示该项不可用（填默认值）。
 */

#include "diagnostics_aggregator.h"
#include "i_axis_group.h"
#include "robot_feedback.h"

class MotionCycle;
class RealtimeLoop;

/**
 * @brief 汇总通信、实时、反馈与轴诊断（NRT）
 * @param cycle 运动周期；nullptr 则通信视为正常、WKC 视为完整
 * @param loop 实时循环；nullptr 则 overrun 为 0
 * @param feedback 运动反馈
 * @param axes 轴组；nullptr 则不填各轴诊断
 * @param sm2_sync_error NRT 采样的 0x1C32:32；RT 路径传上次缓存
 * @param sm3_sync_error NRT 采样的 0x1C33:32
 * @return 诊断快照（含 summary）
 */
DiagnosticsSnapshot collectDiagnostics(const MotionCycle* cycle,
                                       const RealtimeLoop* loop,
                                       const RobotFeedback& feedback,
                                       const IAxisGroup* axes = nullptr,
                                       bool sm2_sync_error = false,
                                       bool sm3_sync_error = false);
