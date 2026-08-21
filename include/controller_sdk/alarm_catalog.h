#pragma once

/**
 * @file alarm_catalog.h
 * @brief 产线报警目录：统一 Alarm ID → 中文标题 / 建议动作
 *
 * NRT 使用。由 DiagnosticsSnapshot 生成活动报警列表，供 GUI / MES / 日志。
 */

#include "diagnostics_snapshot.h"

#include <cstdint>
#include <string>
#include <vector>

/** @brief 统一报警编号（工业可读） */
enum class AlarmId : uint32_t {
    None = 0,

    // 1xxx 通信 / 实时
    CommLost = 1001,
    CommFaultLatched = 1002,
    WcIncomplete = 1003,
    RtOverrun = 1004,

    // 2xxx 轴 / 驱动
    AxisFault = 2001,
    AxisFollowingError = 2002,
    AxisHomingTimeout = 2003,
    AxisHomingError = 2004,
    AxisCmdQueueFull = 2005,
    AxisDomainNull = 2006,
    AxisMappingMissing = 2007,
    AxisSdoFailed = 2008,
    AxisDriveError = 2099,  ///< 其它 CiA402 ErrorCode

    // 3xxx 运动 / 总控
    CartFollowingError = 3001,
};

enum class AlarmSeverity : uint8_t {
    Info = 0,     ///< 提示
    Warning = 1,  ///< 警告
    Error = 2,    ///< 故障
    Fatal = 3     ///< 致命（通信丢失等）
};

/** @brief 单条活动报警 */
struct AlarmEntry {
    AlarmId id = AlarmId::None;
    AlarmSeverity severity = AlarmSeverity::Info;
    std::string code;       ///< 如 "E1001"
    std::string title;      ///< 中文短标题
    std::string advice;     ///< 建议动作
    int axis_index = -1;    ///< -1=系统级
    uint32_t detail = 0;    ///< 附加：status_word / error_code / overrun 等
};

/** @brief 目录项（静态描述，不含活动轴号） */
struct AlarmDescriptor {
    AlarmId id = AlarmId::None;
    AlarmSeverity severity = AlarmSeverity::Info;
    const char* title = "";
    const char* advice = "";
};

/** @brief 查目录；未知 ID 返回通用描述 */
AlarmDescriptor lookupAlarm(AlarmId id) noexcept;

/** @brief 格式化为 E#### */
std::string alarmCodeString(AlarmId id);

/** @brief 严重级别中文 */
const char* alarmSeverityText(AlarmSeverity s) noexcept;

/**
 * @brief 将底层轴 error_code（CiA402Motor::ErrorCode 数值）映射为 AlarmId
 * @note 不依赖 EtherCAT 头文件，数值与 CiA402Motor::ErrorCode 对齐
 */
AlarmId alarmIdFromAxisErrorCode(uint8_t error_code) noexcept;

struct AlarmCollectOptions {
    double joint_follow_warn_rad = 0.05;
    double cart_follow_warn_mm = 2.0;
    /** @brief 未使能是否产生 Info 报警（默认否，避免刷屏） */
    bool report_not_enabled = false;
};

/**
 * @brief 从诊断快照生成当前活动报警（NRT）
 * 同一类系统报警只出一条；轴报警按轴展开。
 */
std::vector<AlarmEntry> collectAlarms(const DiagnosticsSnapshot& snap,
                                      const AlarmCollectOptions& opt = {});
