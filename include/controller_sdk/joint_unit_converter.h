#pragma once

/**
 * @file joint_unit_converter.h
 * @brief 关节单位转换层（工业级、实时安全版本）。
 *
 * 作用：
 * 1. 实现关节弧度 (rad) 与电机脉冲 (pulse) 的双向转换。
 * 2. 实现速度 rad/s 与 pulse/s 的双向转换。
 * 3. 实现加速度 rad/s² 与 pulse/s² 的双向转换（保留符号）。
 * 4. 提供软限位、速度/加速度限制的查询接口。
 * 5. 所有实时路径函数均为 noexcept，使用错误码替代异常。
 *
 * 设计原则：
 * - 运动学层使用 rad。
 * - 轨迹层使用 rad / rad/s / rad/s²。
 * - 电机层使用 pulse。
 * - 本类作为单位和安全限位的唯一转换入口。
 */

#include <cstdint>
#include <string>
#include <utility>

/**
 * @brief 单个关节轴的单位转换配置。
 */
struct JointAxisConfig {
    /** @brief 轴名称，用于错误信息标识 */
    std::string name = "axis";

    /** @brief 编码器每电机圈脉冲数（例如 17bit：131072，20bit：1048576） */
    double encoder_resolution = 131072.0;

    /** @brief 减速比（电机转 gear_ratio 圈，关节转 1 圈） */
    double gear_ratio = 1.0;

    /** @brief 电机方向因子（+1.0 或 -1.0） */
    double direction = 1.0;

    /** @brief 关节零点偏移（单位 rad），当关节处于 zero_offset_rad 时对应 pulse_offset */
    double zero_offset_rad = 0.0;

    /** @brief 电机脉冲零点偏移 */
    int32_t pulse_offset = 0;

    /** @brief 关节最小软限位（单位 rad） */
    double min_position_rad = -3.14159265358979323846;

    /** @brief 关节最大软限位（单位 rad） */
    double max_position_rad = 3.14159265358979323846;

    /** @brief 关节最大速度（单位 rad/s） */
    double max_velocity_rad_s = 10.0;

    /** @brief 关节最大加速度（单位 rad/s²） */
    double max_acceleration_rad_s2 = 100.0;
};

/**
 * @brief 转换错误码枚举（用于实时安全路径）。
 */
enum class ConversionError : uint8_t {
    OK = 0,            /**< 转换成功 */
    Overflow,          /**< 结果超出 int32_t 范围 */
    OutOfLimit,        /**< 输入超出软限位或速度/加速度限制 */
    InvalidArgument    /**< 输入非有限值或非法值 */
};

/**
 * @class JointUnitConverter
 * @brief 单关节单位转换器（实时安全，无异常）。
 *
 * 每个 Axis 对应一个转换器实例。
 */
class JointUnitConverter {
public:
    /**
     * @brief 构造函数
     * @param config 初始配置
     */
    explicit JointUnitConverter(const JointAxisConfig& config);

    /** @brief 获取当前配置（只读） */
    const JointAxisConfig& config() const noexcept;

    /**
     * @brief 更新配置（非实时安全，必须在实时循环停止后调用）
     * @param config 新配置
     * @note 本函数会进行严格校验，若非法可能抛异常（仅用于初始化阶段）
     */
    void setConfig(const JointAxisConfig& config);

    // ==================== 实时安全转换接口 (noexcept) ====================

    /**
     * @brief 关节位置 rad → 电机脉冲 pulse（带限位检查）
     * @param joint_rad 输入关节角 (rad)
     * @param pulse 输出脉冲值
     * @return 错误码，OK 表示成功
     */
    ConversionError positionRadToPulse(double joint_rad, int32_t& pulse) const noexcept;

    /**
     * @brief 电机脉冲 pulse → 关节位置 rad
     * @param pulse 电机脉冲
     * @param joint_rad 输出关节角 (rad)
     * @return 错误码（仅可能返回 Overflow，正常不会）
     */
    ConversionError positionPulseToRad(int32_t pulse, double& joint_rad) const noexcept;

    /**
     * @brief 关节速度 rad/s → 电机速度 pulse/s（带限速检查）
     * @param joint_velocity_rad_s 关节速度
     * @param pulse_velocity 输出电机速度
     * @return 错误码
     */
    ConversionError velocityRadToPulse(double joint_velocity_rad_s, int32_t& pulse_velocity) const noexcept;

    /**
     * @brief 电机速度 pulse/s → 关节速度 rad/s
     * @param pulse_velocity 电机速度
     * @param joint_velocity_rad_s 输出关节速度
     * @return 错误码
     */
    ConversionError velocityPulseToRad(int32_t pulse_velocity, double& joint_velocity_rad_s) const noexcept;

    /**
     * @brief 关节加速度 rad/s² → 电机加速度 pulse/s²（保留符号，带限幅检查）
     * @param joint_acceleration_rad_s2 关节加速度
     * @param pulse_acceleration 输出电机加速度（有符号）
     * @return 错误码
     */
    ConversionError accelerationRadToPulse(double joint_acceleration_rad_s2, int32_t& pulse_acceleration) const noexcept;

    /**
     * @brief 电机加速度 pulse/s² → 关节加速度 rad/s²
     * @param pulse_acceleration 电机加速度
     * @param joint_acceleration_rad_s2 输出关节加速度
     * @return 错误码
     */
    ConversionError accelerationPulseToRad(int32_t pulse_acceleration, double& joint_acceleration_rad_s2) const noexcept;

    // ==================== 限位与速度查询 ====================

    /** @brief 关节位置是否在软限位内 */
    bool isPositionWithinLimit(double joint_rad) const noexcept;

    /** @brief 关节速度是否在限制内 */
    bool isVelocityWithinLimit(double joint_velocity_rad_s) const noexcept;

    /** @brief 关节加速度是否在限制内 */
    bool isAccelerationWithinLimit(double joint_acceleration_rad_s2) const noexcept;

    /** @brief 获取每弧度对应的脉冲数 */
    double pulsesPerRad() const noexcept;

    /** @brief 获取每脉冲对应的弧度数 */
    double radPerPulse() const noexcept;

    // ==================== 便捷工具（供初始化阶段使用，可抛异常） ====================
    // 注意：以下函数允许抛异常，仅用于非实时路径（如调试、脚本配置）

    int32_t positionRadToPulse(double joint_rad) const;       /**< 可抛异常版本 */
    double positionPulseToRad(int32_t pulse) const;
    int32_t velocityRadToPulse(double joint_velocity_rad_s) const;
    double velocityPulseToRad(int32_t pulse_velocity) const;
    int32_t accelerationRadToPulse(double joint_acceleration_rad_s2) const;
    double accelerationPulseToRad(int32_t pulse_acceleration) const;

private:
    void validateConfigOrThrow(const JointAxisConfig& config) const;
    ConversionError checkOverflowInt32(double value) const noexcept;

    JointAxisConfig config_;
};