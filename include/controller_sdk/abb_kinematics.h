#pragma once

/**
 * @file abb_kinematics.h
 * @brief ABB 构型六轴工业机器人运动学（标准 DH 参数，解析逆解）。
 *
 * 默认参数为 IRB120，用户可通过 setDHParameters() 修改。
 * 正解输入：6 个关节角 (rad)
 * 逆解输入：末端位姿（位置 mm，姿态 ZYX 欧拉角 deg）
 * 返回关节角 (rad)，自动从多解中选取最接近上一组关节角的解。
 */

#include "robot_kinematics.h"
#include <Eigen/Dense>
#include <vector>
#include <string>
#include <array>

class ABBKinematics : public RobotKinematics {
public:
    /**
     * @brief 默认构造函数，使用 IRB120 的 DH 参数。
     */
    ABBKinematics();

    ~ABBKinematics() override = default;

    /** @brief 轴数 */
    size_t numAxes() const override { return 6; }

    /** @brief 构型名称 */
    std::string typeName() const override { return "ABB_6R"; }

    /**
     * @brief 设置 DH 参数（覆盖默认值）。
     *
     * 输入参数（单位均为 rad 或 mm）：
     * @param a      连杆长度 a_i (mm)，6 个元素
     * @param d      连杆偏距 d_i (mm)，6 个元素
     * @param alpha  连杆扭转角 alpha_i (rad)，6 个元素
     * @param theta0 关节零位偏移 (rad)，6 个元素
     *
     * 说明：
     * - 参数数组均按关节 1~6 的顺序提供。
     * - 调用此函数后，逆解仍假设后三轴轴线交于一点（球腕构型），
     *   若实际机器人不满足此条件，逆解可能失效或精度下降。
     */
    void setDHParameters(const std::array<double, 6>& a,
                         const std::array<double, 6>& d,
                         const std::array<double, 6>& alpha,
                         const std::array<double, 6>& theta0 = {0,0,0,0,0,0});

    // ---------- 运动学接口 ----------
    /**
     * @brief 正向运动学：关节角 (rad) → 末端位姿（位置 mm，姿态 ZYX 欧拉角 deg）
     */
    CartesianPose forward(const std::vector<double>& joints) const override;

    /**
     * @brief 逆向运动学：末端位姿 → 关节角 (rad)，若无解返回空向量
     * @param pose 目标位姿（位置 mm，姿态 ZYX 欧拉角 deg）
     * @param last_joints 上一组关节角 (rad)，用于选解
     */
    std::vector<double> inverse(const CartesianPose& pose,
                                const std::vector<double>& last_joints) const override;

private:
    /** DH 参数（内部全部以 rad 存储） */
    struct DHParam {
        double a;       ///< 连杆长度 (mm)
        double alpha;   ///< 扭转角 (rad)
        double d;       ///< 偏距 (mm)
        double theta;   ///< 关节零位偏移 (rad)
    };

    // 工具函数
    static double normalizeAngle(double rad);
    static double angleDistance(double a, double b);

    // DH 变换矩阵
    static Eigen::Matrix4d dhTransform(const DHParam& dh, double joint_rad);

    // 正解矩阵（只算前 n 个关节）
    Eigen::Matrix4d forwardMatrix(const std::vector<double>& joints) const;
    Eigen::Matrix4d forwardMatrix03(double q1, double q2, double q3) const;

    // 位姿 ↔ 矩阵
    static CartesianPose matrixToPose(const Eigen::Matrix4d& T);
    static Eigen::Matrix4d poseToMatrix(const CartesianPose& pose);

    // 多解选择（所有内部解均为弧度）
    std::vector<double> chooseBestSolution(
        const std::vector<std::vector<double>>& solutions,
        const std::vector<double>& last_joints) const;

    std::vector<DHParam> dh_params_;
};