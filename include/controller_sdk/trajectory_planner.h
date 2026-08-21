#pragma once

/**
 * @file trajectory_planner.h
 * @brief 笛卡尔空间轨迹规划器（工业级，优化版）
 *
 * 本文件定义 TrajectoryPlanner 类，用于生成笛卡尔空间的一次性轨迹和 Jog 点动轨迹。
 *
 * 设计原则：
 * 1. 本类只负责笛卡尔空间轨迹规划，不处理运动学逆解。
 * 2. 本类不处理电机脉冲、关节单位换算、EtherCAT 通信。
 * 3. 输出统一为 CartesianPose，后续由 RobotKinematics 完成逆解。
 * 4. 一次性轨迹输入时间 t，输出对应时刻末端位姿。
 * 5. Jog 轨迹通过共享状态 JogState 修改目标速度，实现实时点动控制。
 * 6. 自动时长估算严格保证速度/加速度不超过配置值。
 */

#include "robot_kinematics.h"

#include <Eigen/Dense>

#include <functional>
#include <memory>

/** @brief 一次性轨迹类型 */
enum class TrajectoryType {
    LINEAR,   /**< 直线匀速轨迹 */
    QUINTIC,  /**< 五次多项式轨迹，平滑启停，起止速度和加速度为 0 */
    CUBIC     /**< 三次多项式轨迹，平滑启停，起止速度为 0 */
    // 注意：圆弧插补通过专门的 createCircular 方法创建，不使用此枚举
};

/** @brief 一次性轨迹配置参数（用于直线和多项式轨迹） */
struct TrajectoryConfig {
    CartesianPose start;          /**< 起点位姿 */
    CartesianPose end;            /**< 终点位姿 */

    double max_speed = 50.0;      /**< 最大线速度，单位 mm/s */
    double max_accel = 200.0;     /**< 最大线加速度，单位 mm/s² */

    /**
     * @brief 指定轨迹总时间，单位 s。
     *
     * 若 duration > 0，则优先使用该时间作为轨迹总时间。
     * 若 duration <= 0，则根据 max_speed 和 max_accel 自动估算轨迹时间。
     */
    double duration = 0.0;
};

/** @brief 轨迹采样结果，包含位姿、进度和结束标志 */
struct TrajectorySample {
    CartesianPose pose;       /**< 当前时刻位姿 */
    double progress = 0.0;    /**< 当前轨迹进度，范围 [0, 1] */
    bool finished = false;    /**< 当前轨迹是否已经结束 */
};

/**
 * @note 规划工厂（create*）为 NRT；返回的函数对象在 RT 中调用时应只做插值。
 * 多段路径抽象见 i_trajectory_planner.h（TrajectorySampler / ITrajectoryPlanner）。
 */

/**
 * @brief Jog 点动控制器状态
 *
 * 该结构体可由外部持有并修改。
 * 实时循环中调用 JogHandle::func(t) 时，会根据 target_vel 平滑更新 pose。
 */
struct JogState {
    CartesianPose pose;       /**< 当前末端位姿 */

    Eigen::Vector3d actual_vel = Eigen::Vector3d::Zero();  /**< 当前实际速度，单位 mm/s */
    Eigen::Vector3d target_vel = Eigen::Vector3d::Zero();  /**< 目标速度，单位 mm/s */

    double max_speed = 50.0;   /**< 最大速度限幅，单位 mm/s */
    double max_accel = 200.0;  /**< 最大加速度限幅，单位 mm/s² */

    double last_time = 0.0;    /**< 上一次调用时间，单位 s */
    bool initialized = false;  /**< 是否已经完成首次时间初始化 */

    /**
     * @brief 软限位（可选保护）
     *
     * 默认值极大，相当于不限制。
     * 用户可设置 min_pos / max_pos，积分后会自动钳位到范围内。
     */
    Eigen::Vector3d min_pos = Eigen::Vector3d::Constant(-1e12);
    Eigen::Vector3d max_pos = Eigen::Vector3d::Constant(1e12);
};

/** @brief Jog 点动句柄，包含点动轨迹函数和状态指针 */
struct JogHandle {
    /** @brief 点动轨迹函数，输入时间 t(s)，返回当前位姿 */
    std::function<CartesianPose(double)> func;

    /** @brief 点动状态指针，外部可通过该指针修改 target_vel 等 */
    std::shared_ptr<JogState> state;
};

/**
 * @class TrajectoryPlanner
 * @brief 笛卡尔空间轨迹规划器（静态方法集）
 *
 * 该类提供静态方法，用于创建一次性轨迹和 Jog 点动轨迹。
 */
class TrajectoryPlanner {
public:
    /**
     * @brief 创建一次性轨迹函数（直线/多项式）
     * @param type 轨迹类型
     * @param cfg 轨迹配置
     * @return 轨迹函数 f(t) -> CartesianPose
     */
    static std::function<CartesianPose(double)> create(
        TrajectoryType type,
        const TrajectoryConfig& cfg
    );

    /**
     * @brief 创建带进度和结束标志的采样函数
     * @param type 轨迹类型
     * @param cfg 轨迹配置
     * @return 采样函数 f(t) -> TrajectorySample
     */
    static std::function<TrajectorySample(double)> createSampler(
        TrajectoryType type,
        const TrajectoryConfig& cfg
    );

    /**
     * @brief 创建 Jog 点动控制器
     * @param initial_pose 初始末端位姿
     * @param max_speed 最大线速度，单位 mm/s
     * @param max_accel 最大线加速度，单位 mm/s²
     * @return JogHandle 点动句柄
     */
    static JogHandle createJog(
        const CartesianPose& initial_pose,
        double max_speed = 50.0,
        double max_accel = 200.0
    );

    /**
     * @brief 估算一次性轨迹总时长
     * @param type 轨迹类型
     * @param cfg 轨迹配置
     * @return 轨迹总时长，单位 s
     */
    static double estimateDuration(
        TrajectoryType type,
        const TrajectoryConfig& cfg
    );

    /**
     * @brief 创建圆弧插补轨迹
     *
     * 从起点沿圆弧运动到终点，圆心由 center 指定。
     * 当前实现采用匀速角度变化（与 LINE 类型类似），
     * 未来可扩展为使用多项式进度实现平滑启停。
     *
     * @param start     起点笛卡尔位姿
     * @param end       终点笛卡尔位姿
     * @param center    圆心位置（只使用 x,y,z 分量）
     * @param clockwise 是否顺时针（从起点到终点）
     * @param max_speed 最大线速度（圆弧路径上的线速度），单位 mm/s
     * @param max_accel 最大线加速度，单位 mm/s²（当前仅保留，未实际使用）
     * @param duration  指定总时长（<=0 时根据弧长和 max_speed 自动计算）
     * @return 轨迹函数 f(t) -> CartesianPose（姿态为线性插值）
     *
     * @throw std::invalid_argument 起点/终点到圆心距离不一致、共线等异常
     */
    static std::function<CartesianPose(double)> createCircular(
        const CartesianPose& start,
        const CartesianPose& end,
        const CartesianPose& center,
        bool clockwise,
        double max_speed,
        double max_accel = 200.0,
        double duration = 0.0
    );

private:
    static std::function<CartesianPose(double)> createLinear(const TrajectoryConfig& cfg);
    static std::function<CartesianPose(double)> createQuintic(const TrajectoryConfig& cfg);
    static std::function<CartesianPose(double)> createCubic(const TrajectoryConfig& cfg);

    static CartesianPose interpolatePose(const CartesianPose& start,
                                         const CartesianPose& end, double s);
    static double positionDistance(const CartesianPose& start, const CartesianPose& end);
    static double clamp(double value, double lower, double upper);
    static void validateConfig(const TrajectoryConfig& cfg);
};