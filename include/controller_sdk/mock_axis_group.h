#pragma once

/**
 * @file mock_axis_group.h
 * @brief 基于 MockServoDrive 的 IAxisGroup，供无硬件调试 / Studio Mock 后端
 *
 * ## RT / NRT 契约
 * 与 `IAxisGroup` 相同。不依赖 IgH。
 */

#include "i_axis_group.h"
#include "joint_unit_converter.h"
#include "mock_servo.h"

#include <memory>
#include <string>
#include <vector>

/**
 * @class MockAxisGroup
 * @brief 仿真轴组：每轴一个 `MockServoDrive` + 单位转换
 */
class MockAxisGroup : public IAxisGroup {
public:
    /** @brief 单轴规格：名称 + 关节转换/限位 */
    struct AxisSpec {
        std::string name;           ///< 轴名
        JointAxisConfig config;     ///< 脉冲↔rad 与软限位
    };

    /**
     * @brief 按规格构造仿真轴
     * @param specs 轴列表（不可为空）
     */
    explicit MockAxisGroup(const std::vector<AxisSpec>& specs);

    size_t size() const noexcept override { return drives_.size(); }

    void realtimeCycleAll(std::size_t max_commands_per_axis = 8) noexcept override;
    AxisGroupError setJointRT(const std::vector<double>& positions) noexcept override;
    AxisGroupError setJointRT(const double* positions, size_t n) noexcept override;
    bool copyPositions(double* out, size_t n) const noexcept override;

    bool allEnabled() const noexcept override;
    bool anyFault() const noexcept override;
    bool anyHomingActive() const noexcept override { return false; }
    bool allHomingComplete() const noexcept override;
    bool anyHomingError() const noexcept override { return false; }

    void enableAll() noexcept override;
    void disableAll() noexcept override;
    void clearFaultAll() noexcept override;
    void stopAll() noexcept override;
    void clearStopAll() noexcept override {}
    void quickStopAll() noexcept override;
    void clearQuickStopAll() noexcept override {}
    void startHomingAll() noexcept override;
    void setHomingTimeoutCyclesAll(uint32_t) noexcept override {}

    std::vector<double> getPositions() const noexcept override;
    std::string faultSummary() const noexcept override;
    const JointAxisConfig* jointConfigAt(size_t index) const noexcept override;
    bool fillAxisDiagnostics(AxisDiag* out, size_t n) const noexcept override;
    bool copyMotorActualPositions(int32_t* out, size_t n) const noexcept override;
    bool copyMotorTargetPositions(int32_t* out, size_t n) const noexcept override;
    bool copyMotorActualVelocities(int32_t* out, size_t n) const noexcept override;

    /**
     * @brief 单轴绝对运动（NRT，Studio 轴面板用）
     * @param index 轴索引
     * @param rad 目标 [rad]
     * @return 错误码
     */
    AxisGroupError moveAbsolute(size_t index, double rad) noexcept;
    /**
     * @brief 单轴点动速度（NRT）
     * @param index 轴索引
     * @param rad_per_s 速度 [rad/s]
     * @return 错误码
     */
    AxisGroupError jog(size_t index, double rad_per_s) noexcept;
    /**
     * @brief 向指定轴注入仿真故障
     * @param index 轴索引
     * @param fault true 进入故障
     */
    void injectFault(size_t index, bool fault) noexcept;

    /**
     * @brief 取得仿真驱动指针
     * @param i 轴索引
     * @return 驱动；越界返回 nullptr
     */
    MockServoDrive* driveAt(size_t i) noexcept {
        return i < drives_.size() ? drives_[i].get() : nullptr;
    }

private:
    std::vector<std::unique_ptr<MockServoDrive>> drives_;
    std::vector<JointUnitConverter> converters_;
    std::vector<JointAxisConfig> configs_;
    std::vector<std::string> names_;
    std::vector<int32_t> last_target_pulse_;
};
