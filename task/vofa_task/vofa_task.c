#include "vofa_task.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>

#include "FreeRTOS.h"
#include "comp_cmd.h"
#include "comp_utils.h"
#include "lx8244_task.h"
#include "task.h"
#include "usart.h"
#include "vofa.h"

#define VOFA_TASK_UPDATE_TIMEOUT_MS (10u)  //! VOFA 任务收发周期

//! VOFA 全局实例指针，供 VofaTask_GetInstance 返回。
static Vofa_t *g_vofa = NULL;

// ==================== 命令解析与分发 ====================

//! 快捷回传错误/消息到 VOFA+ 打印视图。
static void Vofa_Reply(const char *msg)
{
  Vofa_t *vofa = VofaTask_GetInstance();
  if (vofa != NULL) {
    Vofa_SendFireWater(vofa, msg);
  }
}

//! 强定义覆盖弱函数：按 _ 拆分 name，路由到对应设备 handler。
void VofaTask_OnCommand(const char *name, float value)
{
  // ---- 第 1 步：复制 name 到局部 buffer，防止 DMA 后续覆盖 ----
  char buf[64];
  size_t len = strlen(name);
  if (len >= sizeof(buf)) {
    return;
  }
  memcpy(buf, name, len);
  buf[len] = '\0';

  // ---- 第 2 步：手动按 _ 拆分为最多 4 段 ----
  char *seg[4] = {buf, NULL, NULL, NULL};
  size_t seg_idx = 0;
  for (char *p = buf; *p != '\0'; ++p) {
    if (*p == '_') {
      *p = '\0';
      ++seg_idx;
      if (seg_idx >= 4) {   // 超过 4 段 → 格式错误
        Vofa_Reply("format err: too many segments");
        return;
      }
      seg[seg_idx] = p + 1;
    }
  }
  if (seg_idx != 3) {       // 必须正好 4 段（3 个 _）
    Vofa_Reply("format err: need 4 segments (device_id_op_cmd=value!)");
    return;
  }

  const char *device = seg[0];
  const char *id_str = seg[1];
  const char *op     = seg[2];
  const char *cmd    = seg[3];

  // ---- 第 3 步：解析 id 为整数 ----
  char *end = NULL;
  long id_val = strtol(id_str, &end, 10);
  if ((end == id_str) || (*end != '\0') || (id_val < 0) || (id_val > 255)) {
    Vofa_Reply("format err: invalid id");
    return;
  }
  const uint8_t id = (uint8_t)id_val;

  // ---- 第 4 步：按 device 匹配路由，经队列发给对应设备任务 ----
  if (strcmp(device, "servo") == 0) {
    (void)LX824Task_SendCommand(id, op[0], cmd, value);
    return;
  }

  // 未匹配的设备 → 回传错误
  Vofa_Reply("unknown device");
}

// ==================== 实例访问 ====================

Vofa_t *VofaTask_GetInstance(void)
{
  return g_vofa;
}

// ==================== 模块内部桥接 ====================

static void OnVofaCommand(const char *name, float value)
{
  VofaTask_OnCommand(name, value);
}

// ==================== FreeRTOS 任务 ====================

void vofa_task(void *argument)
{
  RM_UNUSED(argument);

  static Vofa_t vofa;  // 静态存储，保证任务整个生命周期内有效。
  err_t status = Vofa_Init(&vofa, &huart6);
  vofa.thread_alert = xTaskGetCurrentTaskHandle();
  Vofa_SetCommandCallback(&vofa, OnVofaCommand);
  g_vofa = &vofa;

  if (status == OK) {
    status = Vofa_Start(&vofa);
  }
  ASSERT(status == OK);
  if (status != OK) {
    g_vofa = NULL;
    vTaskDelete(NULL);
    return;
  }

  for (;;) {
    Vofa_Update(&vofa, VOFA_TASK_UPDATE_TIMEOUT_MS);      // ① 处理命令
    // (void)Vofa_Send(&vofa, data, ARRAY_LEN(data));     // ② 波形发送（已关闭）
  }
}
