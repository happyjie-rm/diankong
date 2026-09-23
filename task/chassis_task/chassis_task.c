/**
 * @file chassis_task.c
 * @brief 底盘控制任务实现
 */
#include "chassis_task.h"

#include "FreeRTOS.h"
#include "chassis_control.h"
#include "comp_utils.h"
#include "task.h"

/* 底盘任务状态（用于调试和错误监控） */
volatile err_t chassis_status = PENDING;

/**
 * @brief 底盘控制任务入口函数
 * @param argument FreeRTOS 任务参数（未使用）
 *
 * @details 任务执行流程：
 *   1. 使用 main.c 已完成注册并启动的 CAN2 设备
 *   2. 初始化底盘电机速度环 PID 参数
 *   3. 周期性调用底盘控制函数（2ms 周期）
 */
void chassis_task(void* argument) {
  RM_UNUSED(argument);

  /* CAN2 及底盘电机已由 main.c 完成注册和启动。 */
  chassis_status = OK;

  /* 初始化底盘电机速度环 PID */
  chassis_speed_pid_init();

  /* 主控制循环 */
  while (1) {
    /* 执行底盘模式控制（根据遥控器状态） */
    Chassis_Mode();

    /* 2ms 控制周期 */
    vTaskDelay(pdMS_TO_TICKS(2U));
  }
}
