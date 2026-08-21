#pragma once

/**
 * @file tcurve_planner.h
 * @brief T 型加减速规划器（梯形/三角形，支持运动中重规划）
 *
 * 按控制周期离散插补：每次 `update()` 推进 1 个周期。速度/加速度在内部
 * 换算为「单位/周期」，与固件 PPT 点到点规则一致：
 * - 静止起步：三角形或梯形
 * - 运动中同向改目标：加速/匀速/减速衔接
 * - 刹停距离不足或反向：先减速到 0，过冲后折返
 * - 平滑停止 / 紧急停止 / 立即停止
 *
 * 单位由调用方决定（脉冲、rad、mm 均可）。对外配置用单位/秒，与
 * `SCurvePlanner` 相同；`frequency` 仅用于周期换算。
 *
 * ## RT / NRT
 * - **NRT**：`setConfig` / `setTarget` / `reset` / 停止命令
 * - **RT**：`update` 以及位置、速度、状态查询（无堆分配）
 *
 * 使用示例：
 * @code
 * TCurveConfig cfg;
 * cfg.max_velocity = 100000.0;      // pulse/s
 * cfg.acceleration = 1e6;
 * cfg.deceleration = 1e6;
 * cfg.frequency = 1000.0;
 *
 * TCurvePlanner planner;
 * planner.setConfig(cfg);
 * planner.setTarget(4000.0);
 * while (planner.update()) {
 *     motor.setTargetPositionRT(lround(planner.position()));
 * }
 * @endcode
 */

#include <cstdint>

/**
 * @brief T 型规划配置
 *
 * 加速度、减速度允许不对称。`start_velocity` 对应固件 `StarVel`，
 * 静止起步时作为第一拍速度（0 表示从零加速）。
 */
struct TCurveConfig {
    double max_velocity = 100.0;       ///< 最大速度 (units/s)
    double acceleration = 500.0;       ///< 加速度 (units/s²)
    double deceleration = 500.0;       ///< 减速度 (units/s²)
    double start_velocity = 0.0;       ///< 静止起步初速度 (units/s)
    double frequency = 1000.0;         ///< 控制频率 (Hz)，须与 `update()` 调用频率一致
    double soft_stop_acc = 500.0;      ///< 平滑停止减速度 (units/s²)
    double emergency_stop_acc = 5000.0;///< 紧急停止减速度 (units/s²)
};

/**
 * @brief 停止方式
 */
enum class TCurveStop : std::uint8_t {
    None = 0,        ///< 正常跟踪目标
    Soft = 1,        ///< 按 soft_stop_acc 与减速上限中的较大值刹停
    Emergency = 2,   ///< 按 emergency_stop_acc 与减速上限中的较大值刹停
    Immediate = 3    ///< 立即速度清零，位置保持当前值
};

/**
 * @class TCurvePlanner
 * @brief 单轴 T 型点到点规划器（在线重规划）
 *
 * 与 `SCurvePlanner` 不同：本类是有状态插补器，不是「一次 plan、事后 sample」。
 * 新目标调用 `setTarget`；每个控制周期调用一次 `update()`。
 */
class TCurvePlanner {
public:
    TCurvePlanner() = default;

    /**
     * @brief 设置限速与加减速（NRT）
     * @param cfg 配置；非法参数（频率/加速度 ≤ 0）会被拒绝并保持旧配置
     * @return true 已接受
     */
    bool setConfig(const TCurveConfig& cfg) noexcept;

    /** @brief 当前配置（只读） */
    const TCurveConfig& config() const noexcept { return config_; }

    /**
     * @brief 规划到绝对目标位置（NRT）
     *
     * 可在静止或运动中调用。运动中若剩余距离不够以当前减速度停在目标上，
     * 会先刹停（可能过冲）再反向折返。
     *
     * @param target_position 目标位置 (units)
     * @return false 配置非法
     */
    bool setTarget(double target_position) noexcept;

    /** @brief 平滑停止（NRT，下一拍 `update` 生效） */
    void softStop() noexcept { stop_ = TCurveStop::Soft; }

    /** @brief 紧急停止（NRT，下一拍 `update` 生效） */
    void emergencyStop() noexcept { stop_ = TCurveStop::Emergency; }

    /** @brief 立即停止：速度清零，目标改为当前位置（NRT） */
    void abort() noexcept;

    /**
     * @brief 静止时设定当前位置（NRT）。运动中调用会被忽略
     * @return true 已写入
     */
    bool setPosition(double position) noexcept;

    /**
     * @brief 复位到静止（NRT）。位置保持，速度清零，规划丢弃
     */
    void reset() noexcept;

    /** @brief 本次规划总时长 (s)，未规划为 0 */
    double totalTime() const noexcept;

    /** @brief 本次规划已运行时间 (s) */
    double elapsed() const noexcept;

    /**
     * @brief RT：推进一个控制周期并刷新指令位置
     * @return true 本周期应把 `position()` 下发（运动中或刚结束的一拍）
     */
    bool update() noexcept;

    double position() const noexcept { return act_pos_; }
    /** @brief 有符号速度 (units/s) */
    double velocity() const noexcept;
    /** @brief 有符号加速度 (units/s²)，匀速段为 0 */
    double acceleration() const noexcept;

    bool standingStill() const noexcept { return standing_still_; }
    bool nearEnd() const noexcept { return near_end_; }
    bool active() const noexcept { return !standing_still_; }
    TCurveStop stopMode() const noexcept { return stop_; }

    /** @brief 当前目标位置 (units) */
    double targetPosition() const noexcept { return cmd_pos_; }

private:
    void syncCycleLimits() noexcept;
    bool poscale(short step) noexcept;
    void compensatePrevious(int up_to_step) noexcept;
    void planFromStandstill(double distance, int dir_offset) noexcept;
    void planDuringMotion(double distance, int dir_offset) noexcept;
    void planReverse(double distance, int dir_offset, bool same_dir) noexcept;
    void planTriangle(double distance, double v0, double v_peak,
                      int idx_acc, int idx_dec, int dir_offset) noexcept;
    void planTrapezoid(double distance, double v0, double v_const,
                       int dir_offset) noexcept;
    int directionSign() const noexcept;
    bool updateStop() noexcept;

    TCurveConfig config_{};
    TCurveStop stop_ = TCurveStop::None;

    double acc_c_ = 0.0;      ///< 加速度 (units/cycle²)，本次规划快照
    double dec_c_ = 0.0;      ///< 减速度 (units/cycle²)，本次规划快照
    double vmax_c_ = 0.0;
    double star_c_ = 0.0;
    double smstop_c_ = 0.0;
    double emstop_c_ = 0.0;

    double act_pos_ = 0.0;
    double cmd_pos_ = 0.0;
    double act_vel_ = 0.0;    ///< 速度绝对值 (units/cycle)
    double pos0_ = 0.0;
    double runt_ = 0.0;

    double t1_ = 0.0, t2_ = 0.0, t3_ = 0.0, t4_ = 0.0;
    double rtim_[4]{};
    double pos_[4]{};
    double vel_[4]{};
    short act_[4]{};

    short step_num_ = 0;
    short rstep_ = 0;
    short cur_sts_ = 0;
    bool standing_still_ = true;
    bool near_end_ = true;
    bool update_flag_ = false;
};
