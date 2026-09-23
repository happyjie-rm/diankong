#include "imu_can_task.h"

#include "FreeRTOS.h"
#include "comp_utils.h"
#include "task.h"

IMUCAN_t* imu_can_device = NULL;

void imu_can_task(void* argument) {
  RM_UNUSED(argument);
  if ((imu_can_device == NULL) || (imu_can_device->init_error_ != OK)) {
    vTaskDelete(NULL);
    return;
  }

  /* 设备已由 main.c 注册；任务只绑定自己的通知句柄并消费数据。 */
  imu_can_device->thread_alert = xTaskGetCurrentTaskHandle();
  for (;;) imu_can_update(imu_can_device, 2u);
}
