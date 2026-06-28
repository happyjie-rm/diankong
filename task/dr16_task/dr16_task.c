#include "FreeRTOS.h"
#include <stddef.h>

#include "comp_utils.h"
#include "dev_dr16.h"
#include "task.h"
#include "usart.h"

//! 全局 DR16 对象指针，供其他业务模块读取遥控器状态。
DR16_t *dr16 = NULL;

//! DR16 FreeRTOS 任务：初始化 USART2 接收，并周期性刷新在线状态和解析结果。
void dr16_task(void *argument) {
  RM_UNUSED(argument);

  static DR16_t dr16_instance;  // 静态存储，保证任务整个生命周期内有效。
  err_t status = DR16_Init(&dr16_instance, &huart2);
  dr16 = &dr16_instance;

  dr16->thread_alert = xTaskGetCurrentTaskHandle();

  if (status == OK) {
    status = DR16_Start(dr16);
  }
  ASSERT(status == OK);
  if (status != OK) {
    vTaskDelete(NULL);
    return;
  }

  for (;;) {
    DR16_Update(dr16, DR16_OFFLINE_TIMEOUT_MS);  // 超时未收到合法帧则自动置离线。
  }
}
