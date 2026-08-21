#pragma once

/**
 * @file axis_group.h
 * @brief 多轴同步控制封装（工业级实时安全版本）
 *
 * 作用：
 * 1. 管理一组 Axis 指针。
 * 2. 在同一 EtherCAT 周期内批量下发关节位置/速度命令。
 * 3. 查询多轴状态（全部使能、故障汇总、到位情况）。
 * 4. 提供统一错误码，不抛异常，适配实时路径。
 *
 * 实现 IAxisGroup，供 RobotController 依赖倒置。
 */

#include "axis.h"
#include "i_axis_group.h"

#include <vector>
#include <string>
#include <cstdint>

/**
 * @class AxisGroup
 * @brief 多轴同步控制类
 *
 * 通过裸指针引用 Axis，不拥有所有权，由上层管理生命周期。
 *
 * ## RT / NRT
 * - RT：`realtimeCycleAll`、`setJointRT`、`copyPositions`、布尔查询
 * - NRT：轴管理、`getPositions`/`faultSummary`、异步批量命令
 */
class AxisGroup : public IAxisGroup {
public:
    AxisGroup() = default;

    /**
     * @brief 使用轴列表构造
     * @param axes 轴指针列表（不能包含空指针）
     */
    explicit AxisGroup(const std::vector<Axis*>& axes);

    /** @brief 轴组名称 */
    const std::string& name() const noexcept { return name_; }
    /**
     * @brief 设置轴组名称（NRT）
     * @param name 显示名
     */
    void setName(const std::string& name) noexcept { name_ = name; }

    /**
     * @brief 追加一根轴（不获得所有权）
     * @param axis 非空指针；空指针被忽略
     */
    void addAxis(Axis* axis) noexcept;
    /**
     * @brief 按名称移除轴（不释放对象）
     * @param axis_name 轴名
     */
    void removeAxis(const std::string& axis_name) noexcept;
    /** @brief 清空轴列表（不释放对象） */
    void clear() noexcept;
    /** @brief 当前轴数 */
    size_t size() const noexcept override { return axes_.size(); }
    /**
     * @brief 按索引取轴
     * @param index 从 0 起
     * @return 轴指针；越界返回 nullptr
     */
    Axis* axis(size_t index) const noexcept;
    /**
     * @brief 按名称取轴
     * @param name 轴名
     * @return 轴指针；未找到返回 nullptr
     */
    Axis* axis(const std::string& name) const noexcept;

    // ---------- IAxisGroup / 实时 ----------
    void realtimeCycleAll(std::size_t max_commands_per_axis = 8) noexcept override;

    void enableAll() noexcept override;
    void disableAll() noexcept override;
    void clearFaultAll() noexcept override;
    void stopAll() noexcept override;
    void clearStopAll() noexcept override;
    void quickStopAll() noexcept override;
    void clearQuickStopAll() noexcept override;
    void startHomingAll() noexcept override;
    void setHomingTimeoutCyclesAll(uint32_t cycles) noexcept override;

    /**
     * @brief 全部轴绝对位置运动（NRT 投递，CSP）
     * @param positions 各轴目标 [rad]
     * @return 错误码
     */
    AxisGroupError moveJoint(const std::vector<double>& positions) noexcept;
    AxisGroupError setJointRT(const std::vector<double>& positions) noexcept override;
    AxisGroupError setJointRT(const double* positions, size_t n) noexcept override;
    /**
     * @brief 全部轴速度运动（NRT 投递，CSV）
     * @param velocities 各轴目标 [rad/s]
     * @return 错误码
     */
    AxisGroupError moveVelocity(const std::vector<double>& velocities) noexcept;
    /**
     * @brief 全部轴转矩运动（NRT 投递，CST）
     * @param torques_percent 各轴目标转矩 [%]
     * @return 错误码
     */
    AxisGroupError moveTorque(const std::vector<double>& torques_percent) noexcept;

    /**
     * @brief 全部轴切换运行模式（NRT 投递）
     * @param mode CiA402 模式
     */
    void setMode(ServoOperationMode mode) noexcept;

    bool allEnabled() const noexcept override;
    bool anyFault() const noexcept override;
    /** @brief 是否全部目标到达（Statusword bit10） */
    bool allTargetReached() const noexcept;
    bool anyHomingActive() const noexcept override;
    bool allHomingComplete() const noexcept override;
    bool anyHomingError() const noexcept override;

    bool copyPositions(double* out, size_t n) const noexcept override;
    std::vector<double> getPositions() const noexcept override;
    /**
     * @brief 当前关节速度（NRT，可能分配）
     * @return 各轴 [rad/s]
     */
    std::vector<double> getVelocities() const noexcept;
    std::string faultSummary() const noexcept override;
    const JointAxisConfig* jointConfigAt(size_t index) const noexcept override;

    bool fillAxisDiagnostics(AxisDiag* out, size_t n) const noexcept override;
    bool copyMotorActualPositions(int32_t* out, size_t n) const noexcept override;
    bool copyMotorTargetPositions(int32_t* out, size_t n) const noexcept override;
    bool copyMotorActualVelocities(int32_t* out, size_t n) const noexcept override;

private:
    std::string name_ = "default_group";
    std::vector<Axis*> axes_;
};
