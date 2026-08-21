#pragma once

/**
 * @file motion_sync.h
 * @brief 机型 JSON 配置的电子齿轮 / 凸轮 / 龙门，接到周期关节指令上
 *
 * ## RT / NRT 契约
 * - **NRT**：`configure` / `reset`（凸轮 `loadTable` 可能分配）。
 * - **RT**：`apply`（改写 `joints[]`；无堆分配）。
 *
 * 缺省（空配置）不改关节，行为与 1.2 相同。
 * 应用顺序：齿轮 → 凸轮 → 龙门纠偏。
 */

#include "electronic_gear.h"
#include "machine_config.h"

#include <cstddef>
#include <string>

/**
 * @class MotionSyncChain
 * @brief 将 `MachineSyncConfig` 落到 `ElectronicGear` / `ElectronicCAM` / `GantrySync`
 */
class MotionSyncChain {
public:
    static constexpr size_t kMaxGears = 8;
    static constexpr size_t kMaxCams = 4;
    static constexpr size_t kMaxGantries = 4;

    MotionSyncChain() = default;

    /**
     * @brief 按机型同步段配置内部算法（NRT）
     * @param cfg 齿轮/凸轮/龙门列表；空则 clear
     * @param axis_count 轴数，用于校验 master/slave 下标
     * @param err 失败原因
     * @return true 成功（空配置也成功）
     */
    bool configure(const MachineSyncConfig& cfg, size_t axis_count, std::string& err) noexcept;

    /** @brief 复位齿轮内部状态（NRT / 使能后） */
    void reset() noexcept;

    /**
     * @brief 把齿轮输出对齐到当前从轴位置，避免使能后第一拍跳变
     * @param joints 当前关节 [rad]
     * @param n 轴数
     */
    void resetTo(const double* joints, size_t n) noexcept;

    /** @brief 是否没有任何同步项 */
    bool empty() const noexcept { return n_gears_ == 0 && n_cams_ == 0 && n_gantries_ == 0; }

    size_t gearCount() const noexcept { return n_gears_; }
    size_t camCount() const noexcept { return n_cams_; }
    size_t gantryCount() const noexcept { return n_gantries_; }

    /**
     * @brief RT：按主轴指令改写从轴目标
     * @param joints 本周期 IK/指令关节 [rad]，就地修改从轴
     * @param n 轴数
     * @param actual 实际关节 [rad]（龙门用；nullptr 则用 joints）
     * @param dt 周期 [s]
     */
    void apply(double* joints, size_t n, const double* actual, double dt) noexcept;

private:
    struct GearSlot {
        size_t master = 0;
        size_t slave = 0;
        ElectronicGear gear;
    };
    struct CamSlot {
        size_t master = 0;
        size_t slave = 0;
        ElectronicCAM cam;
    };
    struct GantrySlot {
        size_t axis_a = 0;
        size_t axis_b = 0;
        GantrySync gantry;
    };

    GearSlot gears_[kMaxGears]{};
    CamSlot cams_[kMaxCams]{};
    GantrySlot gantries_[kMaxGantries]{};
    size_t n_gears_ = 0;
    size_t n_cams_ = 0;
    size_t n_gantries_ = 0;
};
