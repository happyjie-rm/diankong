#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "comp_cmd.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 舵机命令消息，经 FreeRTOS 队列从 vofa_task 发往 lx824_task。
 */
typedef struct {
  uint8_t id;                //!< 舵机编号
  char op;                   //!< 'w' 写 / 'r' 读
  char cmd[16];              //!< 指令名，如 "id"、"angle"
  float value;               //!< 写操作目标值 / 读操作占位
} servo_cmd_msg_t;

/**
 * @brief FreeRTOS 中 LX824 舵机任务入口。
 */
void lx824_task(void *argument);

/**
 * @brief 向 lx824_task 发送舵机命令（vofa_task 调用）。
 *
 * 将命令参数拷贝到队列消息中，经 FreeRTOS 队列发送给 lx824_task，
 * lx824_task 在自己的任务上下文中执行 LX824 协议收发。
 *
 * @return err_t OK 表示入队成功
 */
err_t LX824Task_SendCommand(uint8_t id, char op, const char *cmd, float value);

#ifdef __cplusplus
}
#endif
