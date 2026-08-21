#pragma once

/**
 * @file machine_config.h
 * @brief 机型配置（JSON）：轴参数、运动学、EtherCAT、安全限位
 *
 * ## RT / NRT 契约
 * - **NRT**：`loadMachineConfig*` / `saveMachineConfigFile` / `createKinematics`。
 * - 周期内不要解析 JSON。
 *
 * 工业换型入口：现场只改 JSON / 从站 XML，不改代码。
 */

#include "joint_unit_converter.h"
#include "safety_limiter.h"
#include "electronic_gear.h"

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

class RobotKinematics;

/** @brief 运动学类型 */
enum class KinematicsType {
    None = 0,   ///< 无运动学（仅关节）
    Delta,      ///< Delta 并联
    Scara,      ///< SCARA
    Abb,        ///< ABB 6R
    Hebot       ///< 通用 6R DH
};

/** @brief 轴对应的 EtherCAT 从站引用 */
struct MachineAxisSlaveRef {
    uint16_t alias = 0;       ///< 从站别名
    uint16_t position = 0;    ///< 物理位置
    std::string xml_path;     ///< 相对机型文件或绝对路径；空则走自动 PDO
};

/**
 * @brief 可选回原对象字典（NRT SDO）。JSON 无 `homing` 对象则全部不写。
 * 写 0x6098/6099/609A/607C 以及可选厂商 U32（如 GSHD 0x20AA）。
 */
struct MachineAxisHomingConfig {
    bool present = false;              ///< JSON 含 homing 对象
    bool write_method = false;
    int8_t method = 0;                 ///< 0x6098
    bool write_speeds = false;
    uint32_t switch_search = 0;        ///< 0x6099:01
    uint32_t zero_search = 0;          ///< 0x6099:02
    bool write_acceleration = false;
    uint32_t acceleration = 0;         ///< 0x609A
    bool write_offset = false;
    int32_t offset = 0;                ///< 0x607C
    uint16_t vendor_index = 0;         ///< 非 0 则写厂商 U32
    uint8_t vendor_subindex = 0;
    uint32_t vendor_value = 0;
};

/** @brief 单轴：关节参数 + 从站 */
struct MachineAxisConfig {
    JointAxisConfig joint;        ///< 脉冲↔rad 与软限位
    MachineAxisSlaveRef slave;    ///< 从站
    MachineAxisHomingConfig homing; ///< 可选回原 SDO；缺省不写
};

class IServoDrive;

/**
 * @brief NRT：按机型把回原参数写入驱动。`homing.present==false` 时直接成功。
 * @return 全部写入成功为 true；任一项失败为 false（已写的不回滚）
 */
bool applyMachineAxisHoming(IServoDrive& drive, const MachineAxisHomingConfig& h);

/** @brief 运动学几何参数（按 type 取用） */
struct MachineKinematicsConfig {
    KinematicsType type = KinematicsType::Delta;
    double L1 = 200;          ///< Delta 主动臂 [mm]
    double L2 = 400;          ///< Delta 从动臂 [mm]
    double R = 100;           ///< Delta 基座半径 [mm]
    double r = 40;            ///< Delta 动平台半径 [mm]
    double d1 = 300;          ///< SCARA d1 [mm]
    double a1 = 250;          ///< SCARA a1 [mm]
    double a2 = 200;          ///< SCARA a2 [mm]
    double d4_offset = 0;     ///< SCARA d4 偏置 [mm]
};

struct MachineEthercatConfig {
    unsigned master_index = 0;
    unsigned auto_configure_max_slaves = 16;
    bool prefer_auto_pdo = true;  ///< true：调试用，扫描自动 PDO；量产应提交 XML，不要每次扫总线
    /**
     * 从站过程数据看门狗超时（ms）。
     * 仅当 watchdog_timeout_cycles==0 时生效；0=不改写从站默认。须在 activate 前写入。
     */
    uint32_t watchdog_timeout_ms = 100;
    /**
     * 看门狗超时（控制周期数）。>0 时优先于 ms。
     * 1 kHz 下建议 2–10；默认 0 表示沿用 watchdog_timeout_ms（兼容旧 JSON）。
     */
    uint32_t watchdog_timeout_cycles = 0;
    /** MotionCycle：连续 WC/通信失败次数阈值 */
    uint32_t wc_fail_threshold = 3;
    /** true：掉线闩锁须清故障后解除 */
    bool sticky_comm_latch = true;
    /** 通信故障闩锁时 RT 回调内 Quick Stop 全部轴 */
    bool quick_stop_on_comm_fault = true;
    /** MotionCycle 是否做 DC 同步（application_time + sync） */
    bool enable_dc = true;
    /**
     * 从站 DC AssignActivate。0=不调用 ecrt_slave_config_dc。
     * 要 DC：enable_dc=true 且本字段=768（0x0300 SYNC0）。
     * 不要 DC：enable_dc=false。true+0 是陷阱（主站 sync、从站无 SYNC0），activate 会 WARN。
     */
    uint16_t dc_assign_activate = 0;

    /** enable_dc 且未写 AssignActivate：主站在 sync，从站没配 SYNC0 */
    bool dcMasterSlaveMismatch() const noexcept {
        return enable_dc && dc_assign_activate == 0;
    }
    /** Sync0 偏移 [ns]；cycle 取 `MachineConfig::dcSync0CycleNs()` */
    int32_t dc_sync0_shift_ns = 0;

    /**
     * 解析实际写入从站的看门狗毫秒数。
     * cycles>0：cycles × cycle_ms（四舍五入，至少 1 ms）；否则用 watchdog_timeout_ms。
     */
    uint32_t effectiveTimeoutMs(double cycle_ms) const noexcept {
        if (watchdog_timeout_cycles == 0)
            return watchdog_timeout_ms;
        if (!(cycle_ms > 0.0))
            cycle_ms = 1.0;
        const double ms = static_cast<double>(watchdog_timeout_cycles) * cycle_ms;
        if (ms < 1.0)
            return 1u;
        if (ms > 6553500.0)
            return 6553500u;
        return static_cast<uint32_t>(ms + 0.5);
    }
};

/** @brief 安全与跟随误差、回原门闩 */
struct MachineSafetyConfig {
    bool enable_following_limits = true;                 ///< 是否启用跟随误差检查
    double max_joint_following_error_rad = 0.05;         ///< 关节跟随误差阈值 [rad]
    double max_cartesian_following_error_mm = 2.0;       ///< 笛卡尔跟随误差阈值 [mm]
    SafetyLimiter::CartesianBounds cartesian_aabb{};     ///< 笛卡尔 AABB
    /**
     * 为 true 时，未回原完成则拒绝 CSP/CSV/CST/点动。
     * 默认 false：调试软件多数没有完整回原，打开会挡住日常 CSP。
     */
    bool require_homing_complete = false;
};

/** @brief 电子齿轮链路（JSON `sync.gears[]`） */
struct MachineGearLink {
    size_t master = 0;     ///< 主轴索引
    size_t slave = 1;      ///< 从轴索引
    GearConfig cfg{};      ///< 速比 / 限幅 / 滤波
};

/** @brief 电子凸轮链路（JSON `sync.cams[]`；表在加载时读入） */
struct MachineCamLink {
    size_t master = 0;
    size_t slave = 1;
    CamInterpMode interp = CamInterpMode::CUBIC;
    std::vector<CamPoint> table;  ///< master_pos 为度（与 ElectronicCAM 一致）
};

/** @brief 龙门双轴纠偏（JSON `sync.gantries[]`） */
struct MachineGantryLink {
    size_t axis_a = 0;     ///< 参考轴
    size_t axis_b = 1;     ///< 被纠偏轴（命令叠加 correction）
    GantryConfig cfg{};
};

/**
 * @brief 可选同步段。JSON 无 `sync` 对象则全部为空，周期行为与 1.2 相同。
 * 应用顺序：齿轮 → 凸轮 → 龙门。
 */
struct MachineSyncConfig {
    std::vector<MachineGearLink> gears;
    std::vector<MachineCamLink> cams;
    std::vector<MachineGantryLink> gantries;
    bool empty() const noexcept {
        return gears.empty() && cams.empty() && gantries.empty();
    }
};

/** @brief 完整机型：周期、运动学、轴、总线、安全 */
struct MachineConfig {
    int version = 1;                  ///< JSON schema 版本
    std::string name = "unnamed";     ///< 机型名
    std::string description;          ///< 说明
    double cycle_ms = 1.0;            ///< 控制周期 [ms]
    bool soft_realtime = true;        ///< 无 RT 权限时是否允许软实时

    MachineKinematicsConfig kinematics;  ///< 运动学
    std::vector<MachineAxisConfig> axes; ///< 轴列表
    MachineEthercatConfig ethercat;      ///< 总线
    MachineSafetyConfig safety;          ///< 安全
    MachineSyncConfig sync;              ///< 可选齿轮/凸轮/龙门；缺省空

    /** @brief 配置文件所在目录（加载后填充，用于解析相对 xml） */
    std::string base_dir;

    bool empty() const noexcept { return axes.empty(); }
    size_t axisCount() const noexcept { return axes.size(); }

    std::vector<JointAxisConfig> jointConfigs() const;

    /** @brief 按 kinematics 字段创建运动学；失败返回 nullptr 并写 err */
    std::shared_ptr<RobotKinematics> createKinematics(std::string& err) const;

    /** @brief 由关节 + safety 生成 SafetyLimiter */
    SafetyLimiter makeSafetyLimiter() const;

    /** @brief 看门狗超时（ms），cycles 优先 */
    uint32_t effectiveWatchdogTimeoutMs() const noexcept {
        return ethercat.effectiveTimeoutMs(cycle_ms);
    }

    /** @brief DC Sync0 周期 [ns]，由 cycle_ms 换算 */
    uint32_t dcSync0CycleNs() const noexcept {
        double ns = cycle_ms * 1e6;
        if (!(ns > 0.0)) ns = 1e6;
        if (ns > 4000000000.0) ns = 4000000000.0;
        return static_cast<uint32_t>(ns + 0.5);
    }

    /** @brief 解析轴 XML 绝对路径 */
    std::string resolvePath(const std::string& maybe_relative) const;
};

/**
 * @brief 从 JSON 文件加载
 * @return true 成功；失败时 err 含原因
 */
bool loadMachineConfigFile(const std::string& path, MachineConfig& out, std::string& err);

/** @brief 从 JSON 字符串加载 */
bool loadMachineConfigJson(const std::string& json_text, MachineConfig& out, std::string& err);

/** @brief 保存为 JSON 文件 */
bool saveMachineConfigFile(const std::string& path, const MachineConfig& cfg, std::string& err);

/** @brief 内置默认：3 轴 Delta Mock（与历史 Studio 行为一致） */
MachineConfig makeDefaultDeltaMockConfig(int axis_count = 3);

/** @brief 运动学类型 <-> 字符串 */
std::string kinematicsTypeToString(KinematicsType t);
bool kinematicsTypeFromString(const std::string& s, KinematicsType& out);
