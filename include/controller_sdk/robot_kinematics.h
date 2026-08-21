#pragma once

/**
 * @file robot_kinematics.h
 * @brief 机器人运动学抽象：正逆解与笛卡尔位姿
 *
 * ## RT / NRT 契约
 * - **NRT**：创建具体构型、改结构参数。
 * - **RT**：`forwardTo` / `inverseTo`（失败返回 false，不构造空向量）。
 * - **1.x 硬实时笛卡尔只保证 Delta**：`DeltaKinematics::inverseTo` 走栈缓冲。
 *   默认 `inverseTo` / `forwardTo` 仍调用 `inverse` / `forward`（会 `new vector`）。
 *   SCARA / ABB / Hebot 未覆盖定长路径，1 ms 周期里不要当硬 RT。
 *
 * 实现：`DeltaKinematics` / `SCARAKinematics` / `ABBKinematics` / `HebotKinematics`。
 */

#include <vector>
#include <array>
#include <cstddef>
#include <string>

/**
 * @brief 笛卡尔位姿（位置 mm，姿态 ZYX 欧拉角 度）
 *
 * 约定与 ABB/Hebot 一致：`a` = Rz(yaw)，`b` = Ry(pitch)，`c` = Rx(roll)。
 */
struct CartesianPose {
    double x = 0;  ///< X [mm]
    double y = 0;  ///< Y [mm]
    double z = 0;  ///< Z [mm]
    double a = 0;  ///< Rz 偏航 [deg]
    double b = 0;  ///< Ry 俯仰 [deg]
    double c = 0;  ///< Rx 滚转 [deg]
};

/**
 * @class RobotKinematics
 * @brief 正逆运动学接口（关节单位由具体构型决定，Delta/SCARA/ABB 关节为 rad）
 */
class RobotKinematics {
public:
    virtual ~RobotKinematics() = default;

    /**
     * @brief 关节轴数
     * @return 轴数
     */
    virtual size_t numAxes() const = 0;

    /**
     * @brief 正向运动学：关节空间 → 笛卡尔空间
     * @param joints 关节值（通常 rad；长度须等于 `numAxes()`）
     * @return 末端位姿
     */
    virtual CartesianPose forward(const std::vector<double>& joints) const = 0;

    /**
     * @brief 逆向运动学：笛卡尔空间 → 关节空间（单解，按 last_joints 择近）
     * @param pose 目标位姿
     * @param last_joints 上一周期关节值（用于多解择优）
     * @return 关节值；无法到达则返回空向量
     * @note NRT / 兼容接口。硬 RT 请用 `inverseTo`（失败返回 false，不构造空向量）。
     */
    virtual std::vector<double> inverse(const CartesianPose& pose,
                                        const std::vector<double>& last_joints) const = 0;

    /**
     * @brief 逆向运动学写入定长缓冲（失败不分配）
     * @param pose 目标位姿
     * @param last_joints 上一周期关节；可为 nullptr（按零位择近）
     * @param nlast `last_joints` 长度；与 `numAxes()` 不符则忽略 seed
     * @param out 输出缓冲
     * @param n 缓冲长度，须等于 `numAxes()`
     * @return true 成功写入 `out`；不可达 / 长度不符为 false
     * @note 默认实现仍走 `inverse`（可能分配）。1.x 仅 `DeltaKinematics` 覆盖为栈缓冲。
     */
    virtual bool inverseTo(const CartesianPose& pose,
                           const double* last_joints,
                           size_t nlast,
                           double* out,
                           size_t n) const;

    /**
     * @brief 正向运动学：从定长关节缓冲写位姿
     * @return true 成功；长度不符或计算失败为 false
     * @note 默认实现走 `forward`（可能分配）。
     */
    virtual bool forwardTo(const double* joints, size_t n, CartesianPose& out) const;

    /**
     * @brief 返回全部可达逆解
     * @param pose 目标位姿
     * @return 解列表；默认实现只调用 `inverse` 得到至多一个解
     * @note 有多解的构型应重写；调用方可再用 seed 择近。
     */
    virtual std::vector<std::vector<double>> inverseAll(const CartesianPose& pose) const {
        auto one = inverse(pose, std::vector<double>(numAxes(), 0.0));
        if (one.empty()) return {};
        return {std::move(one)};
    }

    /**
     * @brief 构型名称（调试 / 机型 JSON）
     * @return 如 "delta"、"scara"
     */
    virtual std::string typeName() const = 0;
};
