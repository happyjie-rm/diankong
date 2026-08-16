#include "lx8244_task.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "FreeRTOS.h"
#include "comp_utils.h"
#include "lx824.h"
#include "task.h"
#include "usart.h"
#include "vofa_task.h"

#include "cmsis_os.h"

#define LX824_TASK_POLL_INTERVAL_MS (100u)  //! 舵机 ID 轮询周期
#define SERVO_CMD_QUEUE_LEN (4u)            //! 舵机命令队列深度

//! 全局 LX824 对象指针，供其他业务模块复用舵机总线接口。
LX824_t *lx824 = NULL;

//! 舵机命令队列：vofa_task 发命令进来，lx824_task 在自己的上下文处理。
static osMessageQueueId_t servo_cmd_queue = NULL;

// ==================== 舵机命令注册表 ====================

// 前向声明（表在实现在前，先声明让编译器知道签名）
static err_t servo_write_id(uint8_t servo_id, float value);
static err_t servo_read_id(uint8_t servo_id, float *out);
static err_t servo_write_angle(uint8_t servo_id, float value);
static err_t servo_read_angle(uint8_t servo_id, float *out);
static err_t servo_write_offset(uint8_t servo_id, float value);
static err_t servo_read_offset(uint8_t servo_id, float *out);
static err_t servo_write_torque(uint8_t servo_id, float value);
static err_t servo_read_torque(uint8_t servo_id, float *out);

//! 舵机命令表：CMD 名 → {写函数, 读函数}
static const device_cmd_entry_t servo_cmds[] = {
    {"id",    servo_write_id,    servo_read_id},
    {"angle",  servo_write_angle,  servo_read_angle},
    {"offset", servo_write_offset, servo_read_offset},
    {"torque", servo_write_torque, servo_read_torque},
};


// ==================== 写/读函数实现 ====================

//! 写舵机 ID（先读确认舵机在线，再写）
static err_t servo_write_id(uint8_t servo_id, float value)
{
  if ((value < 0.0f) || (value > (float)LX824_ID_MAX)) {
    return OUT_OF_RANGE;
  }

  // 先读 ID，确认舵机在线且有应答
  uint8_t current_id = 0U;
  err_t err = LX824_IdRead(lx824, servo_id, &current_id);
  if (err != OK) {
    return err;  // 舵机无应答 → TIMEOUT / NO_RESPONSE
  }

  uint8_t new_id = (uint8_t)(value + 0.5f);
  return LX824_IdWrite(lx824, servo_id, new_id);
}

//! 读舵机 ID（Cmd 14，阻塞式请求-应答）
static err_t servo_read_id(uint8_t servo_id, float *out)
{
  if (out == NULL) {
    return PTR_NULL;
  }
  uint8_t id_val = 0U;
  err_t err = LX824_IdRead(lx824, servo_id, &id_val);
  if (err == OK) {
    *out = (float)id_val;
  }
  return err;
}

//! 写舵机角度（先判范围再转型）
static err_t servo_write_angle(uint8_t servo_id, float value)
{
  if ((value < 0.0f) || (value > (float)LX824_ANGLE_MAX)) {
    return OUT_OF_RANGE;
  }
  uint16_t angle = (uint16_t)(value + 0.5f);
  return LX824_MoveTimeWrite(lx824, servo_id, angle, 1000U);
}

//! 读舵机实时位置（Cmd 28，阻塞式请求-应答）
static err_t servo_read_angle(uint8_t servo_id, float *out)
{
  if (out == NULL) {
    return PTR_NULL;
  }
  int16_t pos = 0;
  err_t err = LX824_PosRead(lx824, servo_id, &pos);
  if (err == OK) {
    *out = (float)pos;
  }
  return err;
}

//! 写偏差并掉电保存（Cmd 17 调整 + Cmd 18 保存）
static err_t servo_write_offset(uint8_t servo_id, float value)
{
  if ((value < (float)LX824_OFFSET_MIN) || (value > (float)LX824_OFFSET_MAX)) {
    return OUT_OF_RANGE;
  }
  int8_t off = (int8_t)(value + 0.5f);
  err_t err = LX824_AngleOffsetAdjust(lx824, servo_id, off);
  if (err != OK) {
    return err;
  }
  return LX824_AngleOffsetWrite(lx824, servo_id);
}

//! 读偏差（Cmd 19，阻塞式请求-应答）
static err_t servo_read_offset(uint8_t servo_id, float *out)
{
  if (out == NULL) {
    return PTR_NULL;
  }
  int8_t off = 0;
  err_t err = LX824_AngleOffsetRead(lx824, servo_id, &off);
  if (err == OK) {
    *out = (float)off;
  }
  return err;
}

//! 写扭矩加载/释放（Cmd 31，0=释放，1=加载）
static err_t servo_write_torque(uint8_t servo_id, float value)
{
  if ((value < 0.0f) || (value > 1.0f)) {
    return OUT_OF_RANGE;
  }
  uint8_t load = (value > 0.5f) ? 1U : 0U;
  return LX824_LoadOrUnloadWrite(lx824, servo_id, load);
}

//! 读扭矩加载/释放状态（Cmd 32，阻塞式请求-应答）
static err_t servo_read_torque(uint8_t servo_id, float *out)
{
  if (out == NULL) {
    return PTR_NULL;
  }
  uint8_t load = 0U;
  err_t err = LX824_LoadOrUnloadRead(lx824, servo_id, &load);
  if (err == OK) {
    *out = (float)load;
  }
  return err;
}

// ==================== 舵机 FireWater 回传辅助 ====================

static void servo_reply(uint8_t id, const char *cmd, const char *msg)
{
  Vofa_t *vofa = VofaTask_GetInstance();
  if (vofa == NULL) return;
  char reply[64];
  int n = snprintf(reply, sizeof(reply), "servo_%u_%s: %s", id, cmd, msg);
  if ((n > 0) && ((size_t)n < sizeof(reply))) {
    Vofa_SendFireWater(vofa, reply);
  }
}

static void servo_reply_err(uint8_t id, const char *cmd, err_t err)
{
  servo_reply(id, cmd, "failed");
  (void)err;  // TODO: err 转字符串
}

static void servo_reply_value(uint8_t id, const char *cmd, float val)
{
  Vofa_t *vofa = VofaTask_GetInstance();
  if (vofa == NULL) return;
  char reply[64];
  int n = snprintf(reply, sizeof(reply), "servo_%u_%s=%.3f", id, cmd, (double)val);
  if ((n > 0) && ((size_t)n < sizeof(reply))) {
    Vofa_SendFireWater(vofa, reply);
  }
}

// ==================== 舵机命令处理（在 lx824_task 上下文中执行）====================

static void servo_process_cmd(const servo_cmd_msg_t *msg)
{
  if (msg == NULL) return;

  const device_cmd_entry_t *entry = NULL;
  for (size_t i = 0U; i < ARRAY_LEN(servo_cmds); ++i) {
    if (strcmp(servo_cmds[i].cmd, msg->cmd) == 0) {
      entry = &servo_cmds[i];
      break;
    }
  }
  if (entry == NULL) {
    servo_reply(msg->id, msg->cmd, "cmd not found");
    return;
  }

  err_t err = FAILED;

  if (msg->op == 'w') {
    if (entry->write == NULL) {
      servo_reply(msg->id, msg->cmd, "write not support");
      return;
    }
    err = entry->write(msg->id, msg->value);

  } else if (msg->op == 'r') {
    if (entry->read == NULL) {
      servo_reply(msg->id, msg->cmd, "read not support");
      return;
    }
    float out = 0.0f;
    err = entry->read(msg->id, &out);
    if (err == OK) {
      servo_reply_value(msg->id, msg->cmd, out);
    } else {
      servo_reply_err(msg->id, msg->cmd, err);
    }
  } else {
    servo_reply(msg->id, msg->cmd, "unknown op (use w/r)");
    return;
  }

  if (err == OK) {
    servo_reply(msg->id, msg->cmd, "OK");
  } else if (msg->op == 'w') {
    servo_reply_err(msg->id, msg->cmd, err);
  }
}

// ==================== 队列发送接口（vofa_task 调）====================

err_t LX824Task_SendCommand(uint8_t id, char op, const char *cmd, float value)
{
  if (cmd == NULL) {
    return ARG_ERR;
  }

  servo_cmd_msg_t msg;
  msg.id = id;
  msg.op = op;
  msg.value = value;
  (void)strncpy(msg.cmd, cmd, sizeof(msg.cmd) - 1U);
  msg.cmd[sizeof(msg.cmd) - 1U] = '\0';

  if (servo_cmd_queue == NULL) {
    return NO_BUFF;
  }

  osStatus_t ret = osMessageQueuePut(servo_cmd_queue, &msg, 0, 0);
  return (ret == osOK) ? OK : FULL;
}

// ==================== FreeRTOS 任务 ====================

//! LX824 FreeRTOS 任务：初始化 UART1 舵机总线，周期性读取舵机 ID + 处理命令队列。
void lx824_task(void *argument)
{
  RM_UNUSED(argument);

  // 创建舵机命令队列（必须在收消息前创建）
  servo_cmd_queue = osMessageQueueNew(SERVO_CMD_QUEUE_LEN, sizeof(servo_cmd_msg_t), NULL);
  ASSERT(servo_cmd_queue != NULL);

  static LX824_t lx824_instance;
  err_t status = LX824_Init(&lx824_instance, &huart1);
  lx824 = &lx824_instance;

  lx824->thread_alert = xTaskGetCurrentTaskHandle();

  if (status == OK) {
    status = LX824_Start(lx824);
  }
  ASSERT(status == OK);
  if (status != OK) {
    vTaskDelete(NULL);
    return;
  }

  for (;;) {
    // 处理命令队列（不阻塞，有则处理，无则跳过）
    servo_cmd_msg_t cmd_msg;
    while (osMessageQueueGet(servo_cmd_queue, &cmd_msg, NULL, 0) == osOK) {
      servo_process_cmd(&cmd_msg);
    }

    vTaskDelay(pdMS_TO_TICKS(LX824_TASK_POLL_INTERVAL_MS));
  }
}
