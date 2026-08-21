#pragma once

/**
 * @file pose_math.h
 * @brief 笛卡尔姿态数学：ZYX 欧拉角(度) ↔ 四元数，位置 lerp + 姿态 slerp
 *
 * 约定与 ABB/Hebot 一致：CartesianPose.a = Rz(yaw)，.b = Ry(pitch)，.c = Rx(roll)，单位度。
 * 对外仍保持 a/b/c 欧拉接口，插补在内部经四元数完成，避免大角度线性插值畸变。
 */

#include "robot_kinematics.h"

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <vector>

namespace pose_math {

/**
 * @brief ZYX 欧拉角（度）转四元数
 * @param a_deg Rz [deg]
 * @param b_deg Ry [deg]
 * @param c_deg Rx [deg]
 * @return 单位四元数
 */
inline Eigen::Quaterniond eulerZYXDegToQuat(double a_deg, double b_deg, double c_deg) noexcept {
    const double rz = a_deg * M_PI / 180.0;
    const double ry = b_deg * M_PI / 180.0;
    const double rx = c_deg * M_PI / 180.0;
    return Eigen::AngleAxisd(rz, Eigen::Vector3d::UnitZ()) *
           Eigen::AngleAxisd(ry, Eigen::Vector3d::UnitY()) *
           Eigen::AngleAxisd(rx, Eigen::Vector3d::UnitX());
}

/**
 * @brief 四元数转 ZYX 欧拉角（度）
 * @param q_in 四元数（将归一化）
 * @param[out] a_deg Rz [deg]
 * @param[out] b_deg Ry [deg]
 * @param[out] c_deg Rx [deg]
 */
inline void quatToEulerZYXDeg(const Eigen::Quaterniond& q_in,
                              double& a_deg, double& b_deg, double& c_deg) noexcept {
    Eigen::Quaterniond q = q_in.normalized();
    const Eigen::Matrix3d R = q.toRotationMatrix();
    // ZYX: R = Rz * Ry * Rx
    const double sy = -R(2, 0);
    const double cy = std::sqrt(std::max(0.0, 1.0 - sy * sy));
    double rz, ry, rx;
    if (cy > 1e-8) {
        ry = std::atan2(sy, cy);
        rz = std::atan2(R(1, 0), R(0, 0));
        rx = std::atan2(R(2, 1), R(2, 2));
    } else {
        // 万向锁：取 rz=0
        ry = std::atan2(sy, cy);
        rz = 0.0;
        rx = std::atan2(-R(0, 1), R(1, 1));
    }
    a_deg = rz * 180.0 / M_PI;
    b_deg = ry * 180.0 / M_PI;
    c_deg = rx * 180.0 / M_PI;
}

/**
 * @brief 四元数球面线性插值（最短弧）
 * @param q0 起点
 * @param q1 终点
 * @param t 参数 [0,1]
 * @return 插值结果
 */
inline Eigen::Quaterniond slerp(const Eigen::Quaterniond& q0,
                                const Eigen::Quaterniond& q1,
                                double t) noexcept {
    t = std::max(0.0, std::min(1.0, t));
    Eigen::Quaterniond a = q0.normalized();
    Eigen::Quaterniond b = q1.normalized();
    if (a.dot(b) < 0.0) {
        b.coeffs() = -b.coeffs();  // 最短弧
    }
    return a.slerp(t, b);
}

/**
 * @brief 位置线性插值 + 姿态四元数 slerp
 * @param start 起点位姿
 * @param end 终点位姿
 * @param u 参数 [0,1]
 * @return 插值位姿（a/b/c 仍为欧拉角度）
 */
inline CartesianPose interpolatePose(const CartesianPose& start,
                                     const CartesianPose& end,
                                     double u) noexcept {
    u = std::max(0.0, std::min(1.0, u));
    CartesianPose p;
    p.x = start.x + (end.x - start.x) * u;
    p.y = start.y + (end.y - start.y) * u;
    p.z = start.z + (end.z - start.z) * u;

    const Eigen::Quaterniond q0 = eulerZYXDegToQuat(start.a, start.b, start.c);
    const Eigen::Quaterniond q1 = eulerZYXDegToQuat(end.a, end.b, end.c);
    quatToEulerZYXDeg(slerp(q0, q1, u), p.a, p.b, p.c);
    return p;
}

/**
 * @brief 笛卡尔位置距离
 * @param a 位姿 A
 * @param b 位姿 B
 * @return |Δxyz| [mm]
 */
inline double positionDistance(const CartesianPose& a, const CartesianPose& b) noexcept {
    const double dx = b.x - a.x, dy = b.y - a.y, dz = b.z - a.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

/**
 * @brief 关节空间欧氏距离（用于 IK 多解择近）
 * @param a 关节向量
 * @param b 关节向量
 * @return 距离；长度不符返回很大的数
 */
inline double jointDistance(const std::vector<double>& a,
                            const std::vector<double>& b) noexcept {
    if (a.size() != b.size() || a.empty()) return 1e300;
    double sum = 0.0;
    for (size_t i = 0; i < a.size(); ++i) {
        const double d = a[i] - b[i];
        sum += d * d;
    }
    return std::sqrt(sum);
}

/**
 * @brief 在多组逆解中选最接近 seed 的一组
 * @param solutions 逆解列表
 * @param seed 参考关节角
 * @return 最近解；列表空则返回空向量
 */
inline std::vector<double> chooseNearestJoints(
    const std::vector<std::vector<double>>& solutions,
    const std::vector<double>& seed) noexcept {
    if (solutions.empty()) return {};
    if (seed.size() != solutions[0].size()) return solutions[0];
    double best = 1e300;
    size_t best_i = 0;
    for (size_t i = 0; i < solutions.size(); ++i) {
        if (solutions[i].size() != seed.size()) continue;
        const double d = jointDistance(solutions[i], seed);
        if (d < best) {
            best = d;
            best_i = i;
        }
    }
    return solutions[best_i];
}

}  // namespace pose_math
