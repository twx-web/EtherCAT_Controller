#pragma once

/**
 * @file motion_state.h
 * @brief 机器人运动状态机枚举（供 RobotController）
 *
 * 查询为只读，RT 可拷贝枚举值；改状态由总控在周期内完成。
 */
enum class MotionState {
    IDLE,       ///< 空闲
    MOVING,     ///< 执行一次性轨迹
    JOGGING,    ///< 点动模式
    HOMING,     ///< 回零中
    STOPPING,   ///< 停止中
    ERROR       ///< 错误状态
};
