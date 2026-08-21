#pragma once

/**
 * @file safety_limiter.h
 * @brief 安全限制器（工业级，实时安全）
 *
 * 提供关节位置、速度、加速度的软限位检查，以及可选笛卡尔工作空间 AABB。
 * 所有检查函数均为 noexcept，适合在实时循环中调用。
 *
 * 策略：未配置关节限位时，关节检查失败（fail-closed），避免“空限位放行”。
 */

#include "joint_unit_converter.h"
#include "robot_kinematics.h"

#include <vector>
#include <cmath>
#include <limits>

class SafetyLimiter {
public:
    /** @brief 笛卡尔工作空间轴对齐包围盒（单位与 CartesianPose 一致：mm） */
    struct CartesianBounds {
        bool enabled = false;
        double min_x = -std::numeric_limits<double>::infinity();
        double max_x =  std::numeric_limits<double>::infinity();
        double min_y = -std::numeric_limits<double>::infinity();
        double max_y =  std::numeric_limits<double>::infinity();
        double min_z = -std::numeric_limits<double>::infinity();
        double max_z =  std::numeric_limits<double>::infinity();
    };

    SafetyLimiter() = default;

    /**
     * @brief 使用关节配置列表构造
     * @param configs 各轴的限位配置（通常与 Axis 的 converter 配置一致）
     */
    explicit SafetyLimiter(const std::vector<JointAxisConfig>& configs)
        : configs_(configs) {}

    /**
     * @brief 更新关节配置
     */
    void setJointConfigs(const std::vector<JointAxisConfig>& configs) {
        configs_ = configs;
    }

    /**
     * @brief 设置笛卡尔工作空间限制（可选）
     */
    void setCartesianBounds(const CartesianBounds& bounds) {
        cartesian_bounds_ = bounds;
    }

    const CartesianBounds& cartesianBounds() const noexcept {
        return cartesian_bounds_;
    }

    /** 推荐默认：关节跟随误差 ~2.9°；<=0 关闭检查 */
    static constexpr double kRecommendedFollowingErrorRad = 0.05;
    /** 推荐默认：笛卡尔位置跟随误差 2mm */
    static constexpr double kRecommendedCartesianFollowingErrorMm = 2.0;

    /**
     * @brief 关节跟随误差阈值 [rad]；<=0 表示不启用超差检查
     */
    void setMaxFollowingErrorRad(double max_error_rad) noexcept {
        max_following_error_rad_ = max_error_rad;
    }

    double maxFollowingErrorRad() const noexcept { return max_following_error_rad_; }

    /**
     * @brief 笛卡尔位置跟随误差阈值 [mm]；<=0 表示不启用
     */
    void setMaxCartesianFollowingErrorMm(double max_error_mm) noexcept {
        max_cartesian_following_error_mm_ = max_error_mm;
    }

    double maxCartesianFollowingErrorMm() const noexcept {
        return max_cartesian_following_error_mm_;
    }

    /** @brief 启用推荐跟随误差阈值（可按机型再调） */
    void applyRecommendedFollowingLimits() noexcept {
        max_following_error_rad_ = kRecommendedFollowingErrorRad;
        max_cartesian_following_error_mm_ = kRecommendedCartesianFollowingErrorMm;
    }

    /**
     * @brief 是否已配置关节限位（非空）
     */
    bool hasJointConfigs() const noexcept { return !configs_.empty(); }

    /**
     * @brief 获取指定轴的配置（只读）
     */
    const JointAxisConfig* getConfig(size_t index) const {
        return (index < configs_.size()) ? &configs_[index] : nullptr;
    }

    // ---------- 关节位置检查 ----------

    /**
     * @brief 检查单个关节位置是否在软限位内
     * @return true 合法；未配置或超限返回 false（fail-closed）
     */
    bool isJointPositionValid(size_t axis_index, double joint_rad) const noexcept {
        if (configs_.empty()) return false;
        if (axis_index >= configs_.size()) return false;
        if (!std::isfinite(joint_rad)) return false;
        const auto& cfg = configs_[axis_index];
        return (joint_rad >= cfg.min_position_rad && joint_rad <= cfg.max_position_rad);
    }

    /**
     * @brief 批量检查关节位置
     */
    bool areJointPositionsValid(const std::vector<double>& joints) const noexcept {
        return areJointPositionsValid(joints.data(), joints.size());
    }

    /**
     * @brief 批量检查关节位置（RT，无堆分配）
     * @param joints 各轴 [rad]
     * @param n 长度，须等于已配置轴数
     */
    bool areJointPositionsValid(const double* joints, size_t n) const noexcept {
        if (!joints || configs_.empty()) return false;
        if (n != configs_.size()) return false;
        for (size_t i = 0; i < n; ++i) {
            if (!isJointPositionValid(i, joints[i]))
                return false;
        }
        return true;
    }

    // ---------- 关节速度检查 ----------

    bool isJointVelocityValid(size_t axis_index, double vel_rad_s) const noexcept {
        if (configs_.empty()) return false;
        if (axis_index >= configs_.size()) return false;
        if (!std::isfinite(vel_rad_s)) return false;
        return std::abs(vel_rad_s) <= configs_[axis_index].max_velocity_rad_s;
    }

    bool areJointVelocitiesValid(const std::vector<double>& velocities) const noexcept {
        if (configs_.empty()) return false;
        if (velocities.size() != configs_.size()) return false;
        for (size_t i = 0; i < velocities.size(); ++i) {
            if (!isJointVelocityValid(i, velocities[i])) return false;
        }
        return true;
    }

    // ---------- 关节加速度检查 ----------

    bool isJointAccelerationValid(size_t axis_index, double acc_rad_s2) const noexcept {
        if (configs_.empty()) return false;
        if (axis_index >= configs_.size()) return false;
        if (!std::isfinite(acc_rad_s2)) return false;
        return std::abs(acc_rad_s2) <= configs_[axis_index].max_acceleration_rad_s2;
    }

    // ---------- 笛卡尔空间检查 ----------

    /**
     * @brief 检查笛卡尔位姿是否在工作空间内
     *
     * 未启用 cartesian bounds 时返回 true（依赖关节限位）。
     * 启用后检查 x/y/z AABB；姿态角暂不限制。
     */
    bool isCartesianPoseValid(const CartesianPose& pose) const noexcept {
        if (!cartesian_bounds_.enabled) return true;
        if (!std::isfinite(pose.x) || !std::isfinite(pose.y) || !std::isfinite(pose.z)) {
            return false;
        }
        return pose.x >= cartesian_bounds_.min_x && pose.x <= cartesian_bounds_.max_x &&
               pose.y >= cartesian_bounds_.min_y && pose.y <= cartesian_bounds_.max_y &&
               pose.z >= cartesian_bounds_.min_z && pose.z <= cartesian_bounds_.max_z;
    }

    /**
     * @brief 检查关节跟随误差是否在阈值内
     * @return true 合法；未启用阈值时始终 true；超差或尺寸不匹配返回 false
     */
    bool isJointFollowingErrorValid(const std::vector<double>& target,
                                    const std::vector<double>& actual) const noexcept {
        if (max_following_error_rad_ <= 0.0) return true;
        if (target.size() != actual.size() || target.empty()) return false;
        for (size_t i = 0; i < target.size(); ++i) {
            if (!std::isfinite(target[i]) || !std::isfinite(actual[i])) return false;
            if (std::abs(target[i] - actual[i]) > max_following_error_rad_) return false;
        }
        return true;
    }

    bool isCartesianFollowingErrorValid(const CartesianPose& target,
                                        const CartesianPose& actual) const noexcept {
        if (max_cartesian_following_error_mm_ <= 0.0) return true;
        if (!std::isfinite(target.x) || !std::isfinite(actual.x)) return false;
        const double dx = target.x - actual.x;
        const double dy = target.y - actual.y;
        const double dz = target.z - actual.z;
        const double err = std::sqrt(dx * dx + dy * dy + dz * dz);
        return err <= max_cartesian_following_error_mm_;
    }

private:
    std::vector<JointAxisConfig> configs_;
    CartesianBounds cartesian_bounds_;
    double max_following_error_rad_ = 0.0;
    double max_cartesian_following_error_mm_ = 0.0;
};
