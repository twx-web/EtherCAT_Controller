#pragma once

/**
 * @file delta_kinematics.h
 * @brief Delta 并联机器人运动学类。
 *
 * 本类直接继承 RobotKinematics 基类，用于实现 Delta 并联机器人
 * 的正运动学、逆运动学和构型名称查询。
 *
 * 单位约定：
 * 1. 长度单位：mm。
 * 2. 角度单位：rad。
 * 3. CartesianPose 中 a、b、c 当前不参与 Delta 计算，保留为 0。
 */

#include "robot_kinematics.h"

#include <Eigen/Dense>

#include <array>
#include <cstddef>
#include <string>
#include <vector>

/**
 * @class DeltaKinematics
 * @brief Delta 并联机器人运动学实现类。
 *
 * 本类继承 RobotKinematics，适用于 3 自由度平动 Delta 并联机器人。
 *
 * 机构参数含义：
 * - L1：主动臂长度。
 * - L2：从动臂长度。
 * - R ：基座电机铰点分布半径。
 * - r ：动平台铰点分布半径。
 *
 * 三条机械臂默认均匀分布在 0°、120°、240° 方向。
 */
class DeltaKinematics : public RobotKinematics {
public:
    /**
     * @brief 构造函数。
     *
     * 输入参数：
     * @param L1 主动臂长度，单位 mm。
     * @param L2 从动臂长度，单位 mm。
     * @param R 基座电机铰点分布半径，单位 mm。
     * @param r 动平台铰点分布半径，单位 mm。
     *
     * 输出参数：无。
     *
     * 异常：
     * 当机构参数非法时抛出 std::invalid_argument。
     */
    DeltaKinematics(double L1, double L2, double R, double r);

    /**
     * @brief 析构函数。
     *
     * 输入参数：无。
     * 输出参数：无。
     */
    ~DeltaKinematics() override = default;

    /**
     * @brief 返回 Delta 机构轴数。
     *
     * 输入参数：无。
     *
     * 输出：
     * @return size_t Delta 机构固定返回 3。
     */
    size_t numAxes() const override;

    /**
     * @brief 正向运动学：关节空间到笛卡尔空间。
     *
     * 输入参数：
     * @param joints 三个主动关节角，单位 rad。
     *
     * 输出：
     * @return CartesianPose 末端笛卡尔位姿。
     *
     * 异常：
     * 1. joints 数量不为 3 时抛出 std::invalid_argument。
     * 2. 正解迭代失败时抛出 std::runtime_error。
     *
     * 说明：
     * 当前实现使用数值迭代正解。
     */
    CartesianPose forward(const std::vector<double>& joints) const override;

    /**
     * @brief 逆向运动学：笛卡尔空间到关节空间。
     *
     * @param last_joints 上一周期关节值；在肘上/肘下两解中选最近解。
     * 不可达时返回空向量。
     */
    std::vector<double> inverse(
        const CartesianPose& pose,
        const std::vector<double>& last_joints
    ) const override;

    bool inverseTo(const CartesianPose& pose,
                   const double* last_joints,
                   size_t nlast,
                   double* out,
                   size_t n) const override;

    /** @brief 肘下 + 肘上全部可达逆解 */
    std::vector<std::vector<double>> inverseAll(const CartesianPose& pose) const override;

    /**
     * @brief 获取机器人构型名称。
     *
     * 输入参数：无。
     *
     * 输出：
     * @return std::string 返回 "Delta"。
     */
    std::string typeName() const override;

    /**
     * @brief 几何法逆运动学。
     *
     * 输入参数：
     * @param pose 目标末端位姿。
     *
     * 输出：
     * @return std::vector<double> 三个主动关节角，单位 rad。
     *
     * 异常：
     * 目标点不可达或机构几何退化时抛出 std::runtime_error。
     *
     * 说明：
     * 这是推荐使用的 Delta 逆解实现。
     */
    std::vector<double> inverseGeometric(const CartesianPose& pose) const;

    /**
     * @brief 代数法逆运动学。
     *
     * 输入参数：
     * @param pose 目标末端位姿。
     *
     * 输出：
     * @return std::vector<double> 三个主动关节角，单位 rad。
     *
     * 异常：
     * 目标点不可达或方程退化时抛出 std::runtime_error。
     *
     * 说明：
     * 该函数主要用于对照验证几何逆解。
     */
    std::vector<double> inverseAlgebraic(const CartesianPose& pose) const;

    /**
     * @brief 设置正运动学最大迭代次数。
     *
     * 输入参数：
     * @param max_iterations 最大迭代次数，必须大于 0。
     *
     * 输出参数：无。
     *
     * 异常：
     * max_iterations 小于等于 0 时抛出 std::invalid_argument。
     */
    void setForwardMaxIterations(int max_iterations);

    /**
     * @brief 设置正运动学收敛阈值。
     *
     * 输入参数：
     * @param tolerance 收敛阈值，必须大于 0。
     *
     * 输出参数：无。
     *
     * 异常：
     * tolerance 小于等于 0 时抛出 std::invalid_argument。
     */
    void setForwardTolerance(double tolerance);

private:
    /**
     * @brief 校验机构参数。
     *
     * 输入参数：无。
     * 输出参数：无。
     *
     * 异常：
     * 参数非法时抛出 std::invalid_argument。
     */
    void validateParameters() const;

    /**
     * @brief 校验关节数量。
     *
     * 输入参数：
     * @param joints 关节数组。
     *
     * 输出参数：无。
     *
     * 异常：
     * joints 数量不为 3 时抛出 std::invalid_argument。
     */
    void validateJointSize(const std::vector<double>& joints) const;

    /**
     * @brief 数值限幅。
     *
     * 输入参数：
     * @param value 原始值。
     * @param lower 下限。
     * @param upper 上限。
     *
     * 输出：
     * @return double 限幅后的值。
     */
    static double clamp(double value, double lower, double upper);

    /**
     * @brief 几何逆解单分支
     * @param elbow_sign -1 → theta=gamma-beta（肘下），+1 → gamma+beta（肘上）
     */
    std::vector<double> inverseGeometricBranch(const CartesianPose& pose,
                                               int elbow_sign) const;

    /**
     * @brief 几何逆解单分支写入 `out[3]`（失败返回 false，不抛异常）
     */
    bool inverseGeometricBranchTo(const CartesianPose& pose,
                                  int elbow_sign,
                                  double out[3]) const noexcept;

private:
    double L1_ = 0.0;   /**< 主动臂长度 */
    double L2_ = 0.0;   /**< 从动臂长度 */
    double R_ = 0.0;    /**< 基座半径 */
    double r_ = 0.0;    /**< 动平台半径 */

    std::array<Eigen::Vector2d, 3> motor_xy_;  /**< 三个电机铰点在基座平面的坐标 */

    int forward_max_iterations_ = 80;          /**< 正运动学最大迭代次数 */
    double forward_tolerance_ = 1e-9;          /**< 正运动学收敛阈值 */
};