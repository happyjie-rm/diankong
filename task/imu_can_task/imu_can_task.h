#ifndef IMU_CAN_TASK_H
#define IMU_CAN_TASK_H
#include "comp_cmd.h"
#include "imu_can.h"
#ifdef __cplusplus
extern "C" {
#endif

/* IMU 对象由 main.c 注册，任务只负责绑定通知句柄和消费数据。 */
extern IMUCAN_t* imu_can_device;
void imu_can_task(void* argument);
#ifdef __cplusplus
}
#endif
#endif
