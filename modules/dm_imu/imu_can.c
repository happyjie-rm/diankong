#include "imu_can.h"
#include "convert.h"
#include <string.h>


imu_t imu;
static volatile bool frame_ready_;
static uint8_t frame_buffer_[8];

static void IMUCAN_RxCallback(STM32CAN_t *can, const BSP_CAN_Frame_t *frame,
                              void *context) {
  IMUCAN_t *self = (IMUCAN_t *)context;
  (void)can;
  if ((self == NULL) || (frame == NULL) || (frame->ide_ != CAN_ID_STD) ||
      (frame->rtr_ != CAN_RTR_DATA) || (frame->size_ != 8u) ||
      (frame->id_ != self->mst_id))
    return;
  memcpy(frame_buffer_, frame->data_, sizeof(frame_buffer_));
  frame_ready_ = true;
  if (self->thread_alert != NULL) {
    BaseType_t woken = pdFALSE;
    xTaskNotifyFromISR(self->thread_alert, SIGNAL_IMUCAN_RAW_READY, eSetBits,
                       &woken);
    portYIELD_FROM_ISR(woken);
  }
}

static err_t IMU_SendCommand(uint8_t reg_id, uint8_t access, uint32_t data) {
  uint8_t payload[8] = {0xCCu, reg_id, access, 0xDDu, 0u, 0u, 0u, 0u};
  memcpy(&payload[4], &data, sizeof(data));
  if (imu.can_handle == NULL)
    return PTR_NULL;
  return STM32CAN_SendByHandle(imu.can_handle, imu.can_id, payload,
                               sizeof(payload));
}

err_t imu_can_init(IMUCAN_t *self, STM32CAN_t *can, uint8_t can_id,
                   uint8_t mst_id) {
  if ((self == NULL) || (can == NULL))
    return PTR_NULL;
  memset(self, 0, sizeof(*self));
  self->can_ = can;
  self->can_id = can_id;
  self->mst_id = mst_id;
  self->data.can_id = can_id;
  self->data.mst_id = mst_id;
  self->data.can_handle = can->can_handle_;
  imu = self->data;
  self->init_error_ = STM32CAN_SubscribeRx(can, IMUCAN_RxCallback, self);
  return self->init_error_;
}

void imu_can_update(IMUCAN_t *self, uint32_t timeout_ms) {
  if (self == NULL)
    return;
  uint32_t notify_value = 0u;
  (void)xTaskNotifyWait(SIGNAL_IMUCAN_RAW_READY, UINT32_MAX, &notify_value,
                        pdMS_TO_TICKS(timeout_ms));
  /* CAN2 在调度器启动前已由 main.c 开启；此时可能没有任务通知，
   * 但 ISR 已经缓存了帧，因此只要有完整帧就应当消费。 */
  if (frame_ready_) {
    frame_ready_ = false;
    memcpy(self->raw_data, frame_buffer_, sizeof(self->raw_data));
    IMU_UpdateData(self->raw_data);
    self->data = imu;
    self->online_ = true;
  } else
    self->online_ = false;
}

void imu_init(uint8_t can_id, uint8_t mst_id, CAN_HandleTypeDef *can_handle) {
  imu.can_id = can_id;
  imu.mst_id = mst_id;
  imu.can_handle = can_handle;
}
void imu_write_reg(uint8_t reg_id, uint32_t data) {
  (void)IMU_SendCommand(reg_id, CMD_WRITE, data);
}
void imu_read_reg(uint8_t reg_id) {
  (void)IMU_SendCommand(reg_id, CMD_READ, 0u);
}
void imu_reboot(void) { imu_write_reg(REBOOT_IMU, 0u); }
void imu_accel_calibration(void) { imu_write_reg(ACCEL_CALI, 0u); }
void imu_gyro_calibration(void) { imu_write_reg(GYRO_CALI, 0u); }
void imu_change_com_port(imu_com_port_e p) {
  imu_write_reg(CHANGE_COM, (uint32_t)p);
}
void imu_set_active_mode_delay(uint32_t d) { imu_write_reg(SET_DELAY, d); }
void imu_change_to_active(void) { imu_write_reg(CHANGE_ACTIVE, 1u); }
void imu_change_to_request(void) { imu_write_reg(CHANGE_ACTIVE, 0u); }
void imu_set_baud(imu_baudrate_e b) { imu_write_reg(SET_BAUD, (uint32_t)b); }
void imu_set_can_id(uint8_t id) { imu_write_reg(SET_CAN_ID, id); }
void imu_set_mst_id(uint8_t id) { imu_write_reg(SET_MST_ID, id); }
void imu_save_parameters(void) { imu_write_reg(SAVE_PARAM, 0u); }
void imu_restore_settings(void) { imu_write_reg(RESTORE_SETTING, 0u); }
void imu_request_accel(void) { imu_read_reg(ACCEL_DATA); }
void imu_request_gyro(void) { imu_read_reg(GYRO_DATA); }
void imu_request_euler(void) { imu_read_reg(EULER_DATA); }
void imu_request_quat(void) { imu_read_reg(QUAT_DATA); }

static uint16_t U16LE(const uint8_t *p) {
  return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}
static void IMU_UpdateAccel(const uint8_t *p) {
  for (uint8_t i = 0; i < 3u; ++i)
    imu.accel[i] = uint_to_float(U16LE(&p[2u + i * 2u]), ACCEL_CAN_MIN,
                                 ACCEL_CAN_MAX, 16u);
}
static void IMU_UpdateGyro(const uint8_t *p) {
  for (uint8_t i = 0; i < 3u; ++i)
    imu.gyro[i] =
        uint_to_float(U16LE(&p[2u + i * 2u]), GYRO_CAN_MIN, GYRO_CAN_MAX, 16u);
}
static void IMU_UpdateEuler(const uint8_t *p) {
  imu.pitch = uint_to_float(U16LE(&p[2]), PITCH_CAN_MIN, PITCH_CAN_MAX, 16u);
  imu.yaw = uint_to_float(U16LE(&p[4]), YAW_CAN_MIN, YAW_CAN_MAX, 16u);
  imu.roll = uint_to_float(U16LE(&p[6]), ROLL_CAN_MIN, ROLL_CAN_MAX, 16u);
}
static void IMU_UpdateQuaternion(const uint8_t *p) {
  uint16_t v[4];
  v[0] = (uint16_t)((p[1] << 6) | ((p[2] & 0xF8u) >> 2));
  v[1] = (uint16_t)(((p[2] & 3u) << 12) | (p[3] << 4) | ((p[4] & 0xF0u) >> 4));
  v[2] =
      (uint16_t)(((p[4] & 0x0Fu) << 10) | (p[5] << 2) | ((p[6] & 0xC0u) >> 6));
  v[3] = (uint16_t)(((p[6] & 0x3Fu) << 8) | p[7]);
  for (uint8_t i = 0; i < 4u; ++i)
    imu.q[i] = uint_to_float(v[i], QUATERNION_MIN, QUATERNION_MAX, 14u);
}
void IMU_UpdateData(uint8_t *p) {
  if (p == NULL)
    return;
  switch (p[0]) {
  case ACCEL_DATA:
    IMU_UpdateAccel(p);
    break;
  case GYRO_DATA:
    IMU_UpdateGyro(p);
    break;
  case EULER_DATA:
    IMU_UpdateEuler(p);
    break;
  case QUAT_DATA:
    IMU_UpdateQuaternion(p);
    break;
  default:
    break;
  }
}
