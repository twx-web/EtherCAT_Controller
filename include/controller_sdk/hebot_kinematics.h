#pragma once

/**
 * @file hebot_kinematics.h
 * @brief 通用六轴机械臂运动学框架。
 *
 * 本类继承 RobotKinematics，适用于任何基于 DH 参数建模的六轴串联机械臂。
 * 提供 DH 参数配置、正运动学与逆运动学接口。
 * 正解：DH 连乘；逆解：阻尼最小二乘数值 IK（以 last_joints 为初值）。
 * 复杂机型仍可继承重写解析逆解以提升速度/鲁棒性。
 *
 * 单位约定：
 * - 长度：mm
 * - 角度：rad
 */

#include "robot_kinematics.h"
#include <Eigen/Dense>
#include <array>
#include <vector>
#include <string>

/**
 * @class HebotKinematics
 * @brief 通用六轴串联机械臂运动学框架。
 *
 * 使用示例：
 * @code
 * HebotKinematics robot;
 * robot.setDHParameters({0, 0, 0, 0, 0, 0},   // a (mm)
 *                       {0, 0, 0, 0, 0, 0},   // d (mm)
 *                       {0, 0, 0, 0, 0, 0});  // alpha (rad)
 * robot.setThetaOffset({0,0,0,0,0,0});        // 零位补偿 (rad)
 * @endcode
 *
 */
class HebotKinematics : public RobotKinematics {
public:
    /** @brief 默认构造函数。DH 参数和零位偏移均初始化为 0。 */
    HebotKinematics();

    /** @brief 析构函数 */
    ~HebotKinematics() override = default;

    /** @brief 返回轴数 */
    size_t numAxes() const override { return 6; }

    /** @brief 获取构型名称 */
    std::string typeName() const override { return "Hebot_6R"; }

    // ================== DH 参数配置 ==================

    /**
     * @brief 设置 DH 参数。
     * @param a     连杆长度 a_i (mm)，6 个元素
     * @param d     连杆偏距 d_i (mm)，6 个元素
     * @param alpha 连杆扭转角 alpha_i (rad)，6 个元素
     */
    void setDHParameters(const std::array<double, 6>& a,
                         const std::array<double, 6>& d,
                         const std::array<double, 6>& alpha);

    /**
     * @brief 设置关节零位偏移。
     * @param theta0 各关节的零位偏移量 (rad)，6 个元素。
     */
    void setThetaOffset(const std::array<double, 6>& theta0);

    // ================== 运动学接口 ==================

    /**
     * @brief 正向运动学：关节角 → 末端位姿（DH 连乘）。
     * @param joints 六个关节角 (rad)。
     * @return CartesianPose 末端位姿（位置 mm，姿态 a/b/c = ZYX 度）。
     */
    CartesianPose forward(const std::vector<double>& joints) const override;

    /**
     * @brief 逆向运动学：数值阻尼最小二乘（初值 = last_joints）。
     * @return 六个关节角 (rad)；不收敛返回空向量。
     */
    std::vector<double> inverse(const CartesianPose& pose,
                                const std::vector<double>& last_joints) const override;

    // ================== 正运动学辅助工具 ==================

    /**
     * @brief 计算从基座到末端的齐次变换矩阵 (4x4)。
     * @param joints 六个关节角 (rad)。
     * @return Eigen::Matrix4d 齐次变换矩阵。
     *
     * 该函数已利用 DH 参数实现，用户可在自己的正解实现中直接调用。
     */
    Eigen::Matrix4d forwardTransform(const std::vector<double>& joints) const;

    /**
     * @brief 计算单个连杆的 DH 变换矩阵。
     * @param a     连杆长度 (mm)
     * @param d     连杆偏距 (mm)
     * @param alpha 扭转角 (rad)
     * @param theta 关节角 (rad)
     * @return 4x4 齐次变换矩阵
     */
    static Eigen::Matrix4d dhTransform(double a, double d,
                                       double alpha, double theta);

protected:
    /**
     * @brief 校验关节数组长度是否为 6。
     * @param joints 关节值数组
     * @throw std::invalid_argument 长度不为 6 时
     */
    void checkJointSize(const std::vector<double>& joints) const;

    /** @brief 将旋转矩阵转换为 ZYX 顺序的欧拉角 (rad) */
    static Eigen::Vector3d rotToZYX(const Eigen::Matrix3d& R);

    /** @brief 将关节角标准化到 [-pi, pi] */
    static std::vector<double> normalizeJoints(const std::vector<double>& joints);

    /** @brief 计算两组关节角之间的欧氏距离（考虑旋转对称性） */
    static double jointDistance(const std::vector<double>& a,
                                const std::vector<double>& b);

    // DH 参数
    std::array<double, 6> a_{};       ///< 连杆长度 (mm)
    std::array<double, 6> d_{};       ///< 连杆偏距 (mm)
    std::array<double, 6> alpha_{};   ///< 扭转角 (rad)
    std::array<double, 6> theta0_{};  ///< 关节零位偏移 (rad)
};