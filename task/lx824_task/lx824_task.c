#include "lx8244_task.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "comp_utils.h"
#include "lx824.h"
#include "task.h"
#include "usart.h"

#define LX824_TASK_POLL_INTERVAL_MS (100u)  //! 舵机 ID 轮询周期

//! 全局 LX824 对象指针，供其他业务模块复用舵机总线接口。
LX824_t *lx824 = NULL;

//! 调试观测变量：最近一次读到的舵机 ID 和错误码。
volatile uint8_t debug_lx824_servo_id = 0U;
volatile uint8_t debug_lx824_servo_id_valid = 0U;
volatile int8_t debug_lx824_id_read_error = (int8_t)PENDING;

//! 任务内部缓存：最近一次成功读取到的舵机 ID。
static uint8_t g_servo_id = 0U;
static volatile bool g_servo_id_valid = false;

//! LX824 FreeRTOS 任务：初始化 UART1 舵机总线，并周期性读取舵机 ID。
void lx824_task(void *argument)
{
  RM_UNUSED(argument);

  // 舵机串行总线当前保持使用 UART1（huart2=DR16）
  static LX824_t lx824_instance;  // 静态存储，保证任务整个生命周期内有效。
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
    uint8_t servo_id = 0U;
    // 广播读 ID 只适用于总线上仅挂一个舵机的调试场景。
    const err_t err = LX824_IdRead(lx824, LX824_BROADCAST_ID, &servo_id);
    debug_lx824_id_read_error = (int8_t)err;
    if (err == OK) {
      g_servo_id = servo_id;
      g_servo_id_valid = true;
      debug_lx824_servo_id = servo_id;
      debug_lx824_servo_id_valid = 1U;
    }

    vTaskDelay(pdMS_TO_TICKS(LX824_TASK_POLL_INTERVAL_MS));
  }
}
