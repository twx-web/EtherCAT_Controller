#pragma once

/**
 * @file electronic_gear.h
 * @brief 电子齿轮、电子凸轮与龙门同步（算法模块）
 *
 * ## RT / NRT 契约
 * - **NRT**：`configure` / `loadTable` / `clear` / `reset`。
 * - **RT**：`update`（无堆分配；凸轮表须事先 `loadTable`）。
 *
 * 实现位于 `electronic_gear.cpp`。
 */

#include <cstdint>
#include <vector>
#include <cstddef>

/** @brief 电子齿轮工作模式 */
enum class GearMode : uint8_t {
    MASTER_SLAVE,   ///< 从轴跟随主轴
    FIXED_RATIO,    ///< 固定速比
    PHASE_SHIFT     ///< 带相位偏移
};

/** @brief 电子齿轮参数 */
struct GearConfig {
    double ratio = 1.0;             ///< 从/主 速比
    double master_offset = 0.0;     ///< 主轴位置偏置
    double slave_offset = 0.0;      ///< 从轴位置偏置
    double max_accel = 1e6;         ///< 输出加速度限幅
    double max_velocity = 1e6;      ///< 输出速度限幅
    double filter_tc = 0.0;         ///< 一阶滤波时间常数 [s]；0=关闭
    bool enable_soft_start = true;  ///< 软启动
};

/**
 * @class ElectronicGear
 * @brief 电子齿轮：主轴位置 → 从轴位置
 */
class ElectronicGear {
public:
    ElectronicGear() = default;

    /**
     * @brief 写入齿轮参数（NRT）
     * @param cfg 配置
     */
    void configure(const GearConfig& cfg) noexcept;
    /**
     * @brief 按主轴位置更新从轴指令（RT）
     * @param master_pos 主轴位置
     * @param dt 周期 [s]
     * @return 从轴位置
     */
    double update(double master_pos, double dt) noexcept;
    /**
     * @brief 复位内部状态
     * @param initial_pos 从轴初始位置
     */
    void reset(double initial_pos = 0.0) noexcept;
    /** @brief 当前从轴输出位置 */
    double currentPosition() const noexcept;
    /** @brief 当前配置 */
    const GearConfig& config() const noexcept;

private:
    GearConfig cfg_;
    double filtered_pos_ = 0.0;
    double last_output_ = 0.0;
    double alpha_ = 1.0;
};

/** @brief 凸轮表一点（主轴位置 → 从轴位置/速度） */
struct CamPoint {
    double master_pos = 0.0;  ///< 主轴位置
    double slave_pos = 0.0;   ///< 从轴位置
    double slave_vel = 0.0;   ///< 从轴速度（三次/样条用）
};

/** @brief 凸轮插值方式 */
enum class CamInterpMode : uint8_t {
    LINEAR,  ///< 线性
    CUBIC,   ///< 三次
    SPLINE   ///< 样条
};

/**
 * @class ElectronicCAM
 * @brief 电子凸轮：按主轴位置查表得到从轴位置
 */
class ElectronicCAM {
public:
    ElectronicCAM() = default;

    /**
     * @brief 加载凸轮表（NRT，可能分配）
     * @param table 按 master_pos 递增
     * @param mode 插值方式
     * @return true 成功
     */
    bool loadTable(const std::vector<CamPoint>& table,
                   CamInterpMode mode = CamInterpMode::CUBIC) noexcept;
    /**
     * @brief 按主轴位置插值（RT）
     * @param master_pos 主轴位置
     * @return 从轴位置
     */
    double update(double master_pos) noexcept;
    /** @brief 清空凸轮表（NRT） */
    void clear() noexcept;
    /** @brief 表点数 */
    size_t size() const noexcept;
    /** @brief 只读凸轮表 */
    const std::vector<CamPoint>& table() const noexcept;

private:
    void computeSplineCoeffs() noexcept;

    struct SplineCoeff {
        double a = 0, b = 0, c = 0, d = 0;
    };

    std::vector<CamPoint> table_;
    std::vector<SplineCoeff> spline_coeffs_;
    CamInterpMode mode_ = CamInterpMode::CUBIC;
};

/** @brief 龙门双轴纠偏参数 */
struct GantryConfig {
    double position_gain = 1.0;     ///< 位置差增益
    double velocity_gain = 0.1;     ///< 速度差增益（预留）
    double max_correction = 5.0;    ///< 纠偏限幅
};

/**
 * @class GantrySync
 * @brief 龙门同步：根据两轴位置差给出纠偏量
 */
class GantrySync {
public:
    GantrySync() = default;
    /**
     * @brief 写入纠偏参数（NRT）
     * @param cfg 配置
     */
    void configure(const GantryConfig& cfg) noexcept;
    /**
     * @brief 计算纠偏（RT）
     * @param pos1 轴 1 位置
     * @param pos2 轴 2 位置
     * @return 纠偏量（已限幅）
     */
    double update(double pos1, double pos2) noexcept;

private:
    GantryConfig cfg_;
};

/** @brief PVT 点（位置-速度-时间） */
struct PVTPoint {
    double position = 0.0;  ///< 位置
    double velocity = 0.0;  ///< 速度
    double time = 0.0;      ///< 时间 [s]
};

/** @brief 主从同步输出快照 */
struct SyncOutput {
    double master_position = 0.0;  ///< 主轴位置
    double slave_position = 0.0;   ///< 从轴位置
    double correction = 0.0;       ///< 纠偏
};
