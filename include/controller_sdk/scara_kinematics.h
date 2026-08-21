#pragma once

/**
 * @file scara_kinematics.h
 * @brief SCARA 四轴工业机器人运动学类。
 *
 * 本类继承 RobotKinematics，实现 SCARA 构型机器人的正运动学与解析逆运动学。
 * SCARA 构型具有两个旋转关节（肩、肘）、一个移动关节（Z 轴）和一个末端旋转关节。
 *
 * 单位约定：
 * - 长度：mm
 * - 角度：rad（内部），界面接口使用度
 */

#include "robot_kinematics.h"
#include <Eigen/Dense>
#include <array>
#include <vector>
#include <string>

/**
 * @class SCARAKinematics
 * @brief SCARA 四轴机器人运动学实现类。
 *
 * 默认参数示例：d1=300, a1=250, a2=200, d4_offset=0。
 * 正运动学：关节值 -> 末端位姿 (x, y, z, yaw)。
 * 逆运动学：末端位姿 -> 两组关节解（肘上/肘下），自动选最接近上一组关节角的解。
 */
class SCARAKinematics : public RobotKinematics {
public:
    /**
     * @brief 默认构造函数。
     *
     * 使用预设的 SCARA 参数。
     */
    SCARAKinematics();

    /**
     * @brief 自定义 DH 参数构造函数。
     *
     * @param d1 基座高度（关节 1 原点到关节 2 原点的 Z 向距离） (mm)
     * @param a1 连杆 1 长度 (mm)
     * @param a2 连杆 2 长度 (mm)
     * @param d4_offset 末端工具 Z 轴偏移（零位时关节 3 的初始位置） (mm)
     */
    SCARAKinematics(double d1, double a1, double a2, double d4_offset = 0.0);

    ~SCARAKinematics() override = default;

    /** @brief 返回轴数 */
    size_t numAxes() const override { return 4; }

    /**
     * @brief 正向运动学：关节值 → 末端位姿。
     * @param joints 四个关节值：[θ1, θ2, d3, θ4]
     *               θ1, θ2, θ4 单位 rad；d3 单位 mm
     * @return CartesianPose 末端位姿 (x,y,z 单位 mm)，姿态仅绕 Z 轴旋转，角度存入 c 字段 (度)
     */
    CartesianPose forward(const std::vector<double>& joints) const override;

    /**
     * @brief 逆向运动学：末端位姿 → 关节值。
     * @param pose 目标末端位姿 (x,y,z 单位 mm)，绕 Z 轴旋转角存入 c 字段 (度)
     * @param last_joints 上一组关节值，用于从肘上/肘下两解中选最近解
     * @return 四个关节值 [θ1, θ2, d3, θ4] (rad, mm)，若无解返回空向量
     */
    std::vector<double> inverse(const CartesianPose& pose,
                                const std::vector<double>& last_joints) const override;

    /** @brief 肘上/肘下全部可达逆解 */
    std::vector<std::vector<double>> inverseAll(const CartesianPose& pose) const override;

    /** @brief 获取构型名称 */
    std::string typeName() const override { return "SCARA"; }

    // ================== 参数设置 ==================
    /** @brief 设置 DH 参数 */
    void setDHParameters(double d1, double a1, double a2, double d4_offset = 0.0);

private:
    /** @brief 标准化关节角到 [-pi, pi] */
    static std::vector<double> normalizeJoints(const std::vector<double>& joints);

    /** @brief 计算两组关节值的带权重距离（考虑旋转对称性） */
    static double jointDistance(const std::vector<double>& a,
                                const std::vector<double>& b);

    double d1_;        ///< 基座高度 (mm)
    double a1_;        ///< 连杆 1 长度 (mm)
    double a2_;        ///< 连杆 2 长度 (mm)
    double d4_offset_; ///< 末端工具 Z 轴偏移 (mm)
};