#pragma once

/**
 * @file robot_feedback.h
 * @brief 机器人反馈快照（NRT 拷贝后给 GUI / 诊断）
 *
 * 由 `RobotController::feedback()` 等填充。含 `std::vector` / `std::string`，
 * 不要在硬 RT 热路径里构造或拷贝本结构。
 */

#include <vector>
#include <string>
#include "robot_kinematics.h"

/** @brief 一拍运动反馈：关节、电机脉冲、笛卡尔与跟随误差 */
struct RobotFeedback {
    std::vector<double> actual_joints_rad;           ///< 实际关节角 [rad]
    std::vector<double> target_joints_rad;           ///< 目标关节角 [rad]
    std::vector<double> joint_following_error_rad;   ///< 目标 − 实际 [rad]

    std::vector<int32_t> actual_motor_pos;           ///< 电机实际位置 [pulse]
    std::vector<int32_t> target_motor_pos;           ///< 电机目标位置 [pulse]
    std::vector<int32_t> actual_motor_vel;           ///< 电机实际速度 [pulse/s]

    CartesianPose actual_pose;                       ///< 实际笛卡尔位姿（正解）
    CartesianPose target_pose;                       ///< 当前指令笛卡尔位姿
    double max_joint_following_error_rad = 0.0;      ///< 各轴跟随误差绝对值最大值
    double cartesian_following_error_mm = 0.0;       ///< 笛卡尔位置误差 [mm]
    bool servo_enabled = false;                      ///< 是否全部使能
    bool fault = false;                              ///< 是否存在故障
    std::string message;                             ///< 诊断信息（NRT）
};
