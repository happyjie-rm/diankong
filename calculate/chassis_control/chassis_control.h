/**
 * @file chassis_control.h
 * @brief 固定四轮 M3508 底盘控制 Facade。
 *
 * 对任务层屏蔽 DJ 电机总线细节，仅暴露：
 * - 初始化（绑定 CAN2 + 注册 4 路 M3508）
 * - 周期步进（输入 → 逆解 → 可选速度闭环 → 安全输出）
 * - 强制停机与状态查询
 *
 * 硬件约定：CAN2，命令 ID 0x200，电机设备 ID 1~4（左前/右前/左后/右后）。
 */

#ifndef CHASSIS_CONTROL_H
#define CHASSIS_CONTROL_H

#include "bsp_can.h"
#include "chassis_kinematics.h"
#include "control_common.h"

#include <stdbool.h>
#include <stdint.h>

/**
 * 实车驱动开关。
 * 0（默认）：仅解算与状态机，不向电机下发非零电流，便于上电联调。
 * 1：开启速度闭环与真实驱动。
 */
#ifndef CHASSIS_ACTUATION_ENABLED
#define CHASSIS_ACTUATION_ENABLED (0)
#endif

// 定义底盘的比赛标志位或者是调试标志位 0为调试状态 1为比赛状态
#define CHASSIS_GAME_FLAG (0)

/** 任一电机反馈超时阈值（ms）；超时则整盘进入故障零输出。 */
#define CHASSIS_FEEDBACK_TIMEOUT_MS (20U)
/** 安全零帧心跳间隔（ms）；防止 C620 因长时间无帧而异常。 */
#define CHASSIS_ZERO_HEARTBEAT_MS (20U)
/** 摇杆归一化指令到目标 RPM 的默认缩放（|cmd|≈1 → 约 3000 rpm）。 */
#define CHASSIS_COMMAND_SCALE (3000.0f)
/**
 * 左拨杆 DOWN 档使用的归一化 yaw 常量（|1.0| 经 CHASSIS_COMMAND_SCALE
 * 约对应满量程旋转分量；真车闭环前需标定）。
 */
#define CHASSIS_SWITCH_DOWN_YAW (1.0f)

/**
 * @brief 底盘控制通用输入（由任务层填充，不直接读遥控器）。
 *
 * 速度轴约定与任务层一致：forward 前后、lateral 左右、yaw 旋转。
 * enable 由任务层按左拨杆 sw_l 映射：仅 UP 失能，MID/DOWN 使能；
 * source_online 表示遥控/指令源在线。
 */
typedef struct {
  float forward;      /**< 前后速度指令（归一化，正方向与任务层一致） */
  float lateral;      /**< 左右速度指令（归一化） */
  float yaw;          /**< 旋转指令（归一化；MID 档常为 0，DOWN 档可用常量） */
  bool enable;        /**< 运行使能（任务层：sw_l 为 MID/DOWN 时 true） */
  bool source_online; /**< 指令源在线；false 时强制安全零输出 */
} chassis_control_input_t;

/**
 * @brief 底盘控制状态快照（get_status 返回值拷贝）。
 */
typedef struct {
  control_state_e state;                       /**< 当前控制状态机状态 */
  bool initialized;                            /**< 是否已成功完成 init */
  bool source_online;                          /**< 最近一周期指令源是否在线 */
  bool wheels_online[CHASSIS_WHEEL_COUNT];     /**< 各轮反馈是否在超时窗口内 */
  bool zero_retry_pending;                     /**< 零帧发送失败待重试 */
  float target_rpm[CHASSIS_WHEEL_COUNT];       /**< 逆解后的四轮目标转速 */
  int16_t feedback_rpm[CHASSIS_WHEEL_COUNT];   /**< 各轮反馈转速（读失败为 0） */
  int16_t command[CHASSIS_WHEEL_COUNT];        /**< 实际下发/计划下发的电流命令 */
  err_t last_error;                            /**< 最近一次错误或原因码 */
  uint32_t last_step_tick;                     /**< 最近一次 step 的 tick(ms) */
  uint32_t last_zero_tick;                     /**< 最近一次成功零帧的 tick(ms) */
} chassis_control_status_t;


/**
 * @brief 在指定 CAN2 对象上注册四台 M3508（设备 ID 1-4 / 0x200）。
 *
 * Args:
 *   can: 已初始化但尚未启动的 CAN2 BSP 对象。
 *
 * Returns:
 *   初始化成功返回 OK，否则返回参数、状态或 DJ 电机错误码。
 *
 * Notes:
 *   仅允许调用一次；成功后状态为 SAFE_DISABLED，需由 step 推送零帧。
 */
err_t chassis_control_init(STM32CAN_t *can);

/**
 * @brief 执行一次四轮反馈检查、速度闭环和安全输出。
 *
 * Args:
 *   input: 本周期通用底盘输入。
 *   now_tick: 当前毫秒 tick。
 *
 * Returns:
 *   本周期成功返回 OK；非法输入、反馈超时或发送失败返回对应错误码。
 *
 * Notes:
 *   CHASSIS_ACTUATION_ENABLED=0 时只更新目标与状态，并维持零帧心跳。
 */
err_t chassis_control_step(const chassis_control_input_t *input,
                           uint32_t now_tick);

/**
 * @brief 立即复位四路 PID 并尝试发送 0x200 全零帧。
 *
 * Returns:
 *   零帧发送结果；未初始化返回 STATE_ERR。
 */
err_t chassis_control_force_stop(void);

/**
 * @brief 读取当前底盘控制状态快照。
 *
 * Args:
 *   status: 状态输出缓冲。
 *
 * Returns:
 *   成功返回 OK，空指针返回 PTR_NULL。
 */
err_t chassis_control_get_status(chassis_control_status_t *status);

#endif /* CHASSIS_CONTROL_H */
