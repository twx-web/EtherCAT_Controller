#pragma once

/**
 * @file robot_controller.h
 * @brief 机器人总控层。
 *
 * RobotController 是上层运动控制的唯一入口。
 *
 * 职责：
 * 1. 管理机器人状态机（IDLE / MOVING / JOGGING / HOMING / STOPPING / ERROR）。
 * 2. 调用轨迹规划器 / 前瞻 / S 曲线生成笛卡尔目标位姿。
 * 3. 调用运动学逆解，将 CartesianPose 转换为关节角 (rad)。
 * 4. 调用 SafetyLimiter 做安全检查。
 * 5. 通过 AxisGroup 将关节目标同步下发到各个 Axis。
 * 6. 汇总机器人反馈信息。
 */

#include "diagnostics_snapshot.h"
#include "i_axis_group.h"
#include "i_trajectory_planner.h"
#include "lookahead_planner.h"
#include "motion_state.h"
#include "motion_sync.h"
#include "robot_feedback.h"
#include "robot_kinematics.h"
#include "safety_limiter.h"
#include "trajectory_planner.h"

class MotionCycle;
class RealtimeLoop;

#include <Eigen/Dense>

#include <functional>
#include <memory>
#include <string>
#include <vector>

/**
 * @class RobotController
 * @brief 机器人总控：状态机 + IK + 安全 + 轴组下发
 *
 * ## RT / NRT 契约
 * - **RT**：`update`（采样轨迹、逆解、`setJointRT`）；禁止在此做 SDO / 堆规划。
 * - **NRT**：`moveTo*` / `movePath` / `startJog` / `stop` 规划与入队。
 * - **1.x**：周期内笛卡尔 IK 只保证 `DeltaKinematics`。SCARA/ABB/Hebot 的 `inverseTo`
 *   默认会分配，硬实时笛卡尔请用 Delta，或只走关节 `setJointRT`。
 */
class RobotController {
public:
    /**
     * @brief 构造总控
     * @param kinematics 运动学
     * @param axis_group 轴组抽象（可传 `shared_ptr<AxisGroup>`）
     */
    RobotController(std::shared_ptr<RobotKinematics> kinematics,
                    std::shared_ptr<IAxisGroup> axis_group);

    /**
     * @brief 设置安全限制器（覆盖从轴配置生成的默认）
     * @param limiter 限位与跟随误差阈值
     */
    void setSafetyLimiter(const SafetyLimiter& limiter);

    /**
     * @brief 装入机型 `sync` 段（齿轮/凸轮/龙门）。空配置为无操作。
     * @param cfg 同步配置
     * @param err 失败原因
     * @return true 成功
     */
    bool configureSync(const MachineSyncConfig& cfg, std::string& err);

    /**
     * @brief 控制周期 [s]，供齿轮限速/限加速度。默认 0.001。
     */
    void setCycleDt(double dt) noexcept;

    /**
     * @brief RT：对已有关节指令套齿轮/凸轮/龙门（无 IK）
     * @param joints 就地改写
     * @param n 须等于轴数
     */
    void applySyncRT(double* joints, size_t n) noexcept;

    /** @brief 全部轴使能 */
    bool enable();
    /** @brief 全部轴禁能 */
    bool disable();
    /** @brief 全部轴故障复位 */
    bool clearFault();

    /**
     * @brief 单段笛卡尔运动（多项式/直线，兼容原接口）
     */
    bool moveTo(const CartesianPose& target,
                TrajectoryType type,
                double max_speed,
                double max_accel,
                double current_time);

    /**
     * @brief 单段笛卡尔直线运动，路径参数用 S 曲线加减速
     * @param max_jerk 加加速度；<=0 时取 max_accel*10
     */
    bool moveToSCurve(const CartesianPose& target,
                      double max_speed,
                      double max_accel,
                      double current_time,
                      double max_jerk = 0.0);

    /**
     * @brief 多段连续路径（内部 LookaheadPlanner）
     * @param waypoints 路径点序列（不含当前点，会自动插入当前实际位姿为起点）
     */
    bool movePath(const std::vector<CartesianPose>& waypoints,
                  double max_speed,
                  double max_accel,
                  double current_time,
                  double corner_error_max = 0.5,
                  double blend_radius = 5.0);

    /**
     * @brief 开始笛卡尔点动
     * @param initial_pose 起点位姿
     * @param max_speed 最大速度
     * @param max_accel 最大加速度
     * @param current_time 当前时间 [s]
     * @return true 已进入 JOGGING
     */
    bool startJog(const CartesianPose& initial_pose,
                  double max_speed,
                  double max_accel,
                  double current_time);

    /**
     * @brief 设置点动速度矢量
     * @param velocity_mm_s 笛卡尔速度 [mm/s]
     * @return true 成功
     */
    bool setJogVelocity(const Eigen::Vector3d& velocity_mm_s);

    /**
     * @brief 受控减速停止：沿当前笛卡尔速度方向 S 曲线刹停，经 STOPPING → IDLE
     * @param current_time 当前时间 (s)，与 update() 同源
     * @param max_decel 最大减速度 (mm/s²)
     * @param max_jerk 加加速度；<=0 时取 max_decel*10
     */
    bool stop(double current_time, double max_decel = 200.0, double max_jerk = 0.0);
    /** @brief 急停：Quick Stop 全部轴并进入 ERROR */
    bool emergencyStop();

    /** @brief 全部关节启动回零（CiA402 Homing） */
    void startHoming();
    /**
     * @brief 回零超时
     * @param cycles 控制周期数；0=不超时
     */
    void setHomingTimeoutCycles(uint32_t cycles);
    /** @brief 是否有轴正在回零 */
    bool isHomingActive() const;
    /** @brief 是否全部回零完成 */
    bool isHomingComplete() const;
    /** @brief 是否存在回零错误 */
    bool isHomingError() const;

    /**
     * @brief 周期推进（RT）：采样轨迹、逆解、安全检查、下发关节
     * @param current_time 当前时间 [s]
     */
    void update(double current_time);

    /** @brief 当前运动状态 */
    MotionState state() const;
    /** @brief 反馈快照（NRT 拷贝，含 vector） */
    RobotFeedback feedback() const;
    /** @brief 当前指令笛卡尔位姿 */
    CartesianPose currentTargetPose() const;
    /** @brief 最近一次逆解关节角 [rad] */
    std::vector<double> lastTargetJointsRad() const;
    /**
     * @brief 拷贝最近一次逆解关节角（无堆分配）
     * @return true 当 n == 轴数
     */
    bool copyLastTargetJoints(double* out, size_t n) const noexcept;
    /** @brief 最近一次错误字符串（NRT） */
    std::string lastError() const;

    /** @brief 启用推荐跟随误差阈值（默认构造后可选调用） */
    void enableRecommendedFollowingLimits();

    /**
     * @brief 一键诊断快照（NRT）
     * @param cycle / loop 可为 nullptr
     */
    DiagnosticsSnapshot collectDiagnostics(const MotionCycle* cycle = nullptr,
                                           const RealtimeLoop* loop = nullptr) const;

    /** @brief 当前活动轨迹剩余/总时长估计（无活动轨迹返回 0） */
    double trajectoryDuration() const noexcept { return trajectory_duration_; }

private:
    bool commandPose(const CartesianPose& pose);
    void setFault(const std::string& error);
    void setLastError(const std::string& error);
    bool syncPoseFromFeedback();
    void initSafetyFromAxes();
    void clearMotionRuntime();
    bool canStartMotion(std::string* reason = nullptr) const;
    bool validateTarget(const CartesianPose& target, std::string* reason) const;
    bool startSamplerMotion(std::function<TrajectorySample(double)> sampler,
                            double duration,
                            double current_time,
                            MotionState next_state = MotionState::MOVING);
    void checkFollowingErrorOrFault();
    Eigen::Vector3d estimateCartesianVelocity(double current_time) const;

    std::shared_ptr<RobotKinematics> kinematics_;
    std::shared_ptr<IAxisGroup> axis_group_;

    SafetyLimiter safety_limiter_;
    LookaheadPlanner lookahead_;  // 亦满足 ITrajectoryPlanner
    MotionSyncChain sync_;
    double cycle_dt_ = 0.001;

    static constexpr size_t kMaxAxes = 16;

    MotionState state_ = MotionState::IDLE;

    CartesianPose current_target_pose_{0, 0, 0, 0, 0, 0};
    std::vector<double> last_target_joints_rad_;

    /** 旧式位姿轨迹 f(t)->pose（多项式） */
    std::function<CartesianPose(double)> active_trajectory_;
    /** 新式采样轨迹（S曲线/前瞻）；类型同 TrajectorySampler */
    TrajectorySampler active_sampler_;

    double trajectory_start_time_ = 0.0;
    double trajectory_duration_ = 0.0;

    /** 指令笛卡尔速度估计 (mm/s)，供受控停止使用 */
    Eigen::Vector3d last_cmd_cart_vel_{0.0, 0.0, 0.0};
    CartesianPose last_cmd_pose_{0, 0, 0, 0, 0, 0};
    double last_cmd_time_ = -1.0;
    bool last_cmd_pose_valid_ = false;

    JogHandle jog_handle_;
    std::string last_error_;
};
