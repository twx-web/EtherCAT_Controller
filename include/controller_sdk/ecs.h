/**
 * @file ecs.h
 * @brief 整机 C ABI：打开机型 JSON、启停内部 RT 线程、使能/运动、读 POD 快照
 *
 * 给 Python / ctypes / 其它语言用。RT 循环不出本 ABI：没有 `on_cycle`、
 * 没有 `set_joint_rt`。命令入队，队列满返回 `ECS_ERR_BUSY`，调用方不等周期。
 *
 * 笛卡尔规划在调用线程（NRT）完成；周期内只消费队列并 `RobotController::update`。
 *
 * **同进程注意：** Python 等 GC 语言崩溃或长时间停顿可能拖死同进程 RT。
 * 硬隔离请另开 `controller_runtime` 进程。C++ 应用继续用现有头文件，不必 `#include "ecs.h"`。
 *
 * 当前后端 **仅 Mock**（无需 IgH），不能控真机 EtherCAT。`machine_json==NULL` 时用内置 3 轴 Delta Mock。
 */

#ifndef CONTROLLER_SDK_ECS_H
#define CONTROLLER_SDK_ECS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ECS_ABI_VERSION 1
#define ECS_MAX_AXES 16

typedef uint64_t ecs_handle_t;
typedef int32_t ecs_status_t;

enum {
    ECS_OK = 0,
    ECS_ERR_INVALID = -1,      /**< 参数非法 / 句柄无效 */
    ECS_ERR_BUSY = -2,         /**< 命令队列满，未入队 */
    ECS_ERR_FAULT = -3,        /**< 轴组或总控故障 */
    ECS_ERR_IO = -4,           /**< 机型文件读失败 */
    ECS_ERR_STATE = -5,        /**< 状态不允许（未 start / 未使能等） */
    ECS_ERR_UNSUPPORTED = -6   /**< 本构建不支持该操作 */
};

/**
 * bit0 enabled  全部轴已使能
 * bit1 fault    存在故障或总控 ERROR
 * bit2 comm_ok  Mock 恒为 1
 * bit3 rt_running 内部实时线程在跑
 */
typedef struct ecs_snapshot {
    int32_t abi_version;
    uint32_t axis_count;
    uint32_t flags;
    uint32_t working_counter;
    uint64_t rt_overrun;
    double joints_rad[ECS_MAX_AXES];
    double target_rad[ECS_MAX_AXES];
    double cart_xyz[3];
    uint16_t status_word[ECS_MAX_AXES];
    int32_t motion_state; /**< MotionState 底层值：0 IDLE … 5 ERROR */
} ecs_snapshot_t;

/**
 * @brief 打开控制器（不启动 RT）
 * @param machine_json 机型路径；NULL 或空串 = 内置 3 轴 Delta Mock
 * @param out 成功时写入句柄
 */
ecs_status_t ecs_open(const char* machine_json, ecs_handle_t* out);

/** @brief 停止 RT（若在跑）并释放句柄 */
ecs_status_t ecs_close(ecs_handle_t h);

/** @brief 启动内部实时线程 */
ecs_status_t ecs_start(ecs_handle_t h);

/** @brief 停止实时线程 */
ecs_status_t ecs_stop(ecs_handle_t h);

/** @brief 入队：全部轴使能。队列满 → BUSY */
ecs_status_t ecs_enable(ecs_handle_t h);

/** @brief 入队：全部轴禁能 */
ecs_status_t ecs_disable(ecs_handle_t h);

/** @brief 入队：急停（Quick Stop，总控 ERROR） */
ecs_status_t ecs_estop(ecs_handle_t h);

/** @brief 入队：清故障 */
ecs_status_t ecs_clear_fault(ecs_handle_t h);

/**
 * @brief 入队：单轴绝对位置 [rad]（RT 消费，CSP 直写）
 * @note 有笛卡尔总控且正在插补时会被下一拍轨迹覆盖
 */
ecs_status_t ecs_move_abs(ecs_handle_t h, int32_t axis, double rad);

/**
 * @brief NRT：笛卡尔直线 S 曲线。规划在调用线程，不进 RT
 * @param xyzabc x,y,z [mm]，a,b,c [deg]；NULL 非法
 */
ecs_status_t ecs_move_cart(ecs_handle_t h, const double xyzabc[6],
                           double vmax, double amax);

/** @brief 拷贝最新 POD 快照（seqlock；调用方分配 `out`） */
ecs_status_t ecs_read_snapshot(ecs_handle_t h, ecs_snapshot_t* out);

/** @brief 错误码短字符串（静态存储） */
const char* ecs_strerror(ecs_status_t st);

/**
 * @brief 最近一次 NRT 错误说明
 * @return 写入的字节数（不含 NUL）；`buf==NULL` 时返回需要的长度
 */
int32_t ecs_last_error(ecs_handle_t h, char* buf, uint32_t len);

#ifdef __cplusplus
}
#endif

#endif /* CONTROLLER_SDK_ECS_H */
