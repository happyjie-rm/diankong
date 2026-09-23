#ifndef DM_IMU_CAN_H
#define DM_IMU_CAN_H

#include <stdbool.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "bsp_can.h"
#include "can.h"
#include "comp_def.h"
#include "task.h"

#define ACCEL_CAN_MAX (235.2f)
#define ACCEL_CAN_MIN (-235.2f)
#define GYRO_CAN_MAX (34.88f)
#define GYRO_CAN_MIN (-34.88f)
#define PITCH_CAN_MAX (90.0f)
#define PITCH_CAN_MIN (-90.0f)
#define ROLL_CAN_MAX (180.0f)
#define ROLL_CAN_MIN (-180.0f)
#define YAW_CAN_MAX (180.0f)
#define YAW_CAN_MIN (-180.0f)
#define QUATERNION_MIN (-1.0f)
#define QUATERNION_MAX (1.0f)
#define CMD_READ (0u)
#define CMD_WRITE (1u)

typedef enum { COM_USB = 0, COM_RS485, COM_CAN, COM_VOFA } imu_com_port_e;
typedef enum {
  CAN_BAUD_1M = 0,
  CAN_BAUD_500K,
  CAN_BAUD_400K,
  CAN_BAUD_250K,
  CAN_BAUD_200K,
  CAN_BAUD_100K,
  CAN_BAUD_50K,
  CAN_BAUD_25K
} imu_baudrate_e;
typedef enum {
  REBOOT_IMU = 0,
  ACCEL_DATA,
  GYRO_DATA,
  EULER_DATA,
  QUAT_DATA,
  SET_ZERO,
  ACCEL_CALI,
  GYRO_CALI,
  MAG_CALI,
  CHANGE_COM,
  SET_DELAY,
  CHANGE_ACTIVE,
  SET_BAUD,
  SET_CAN_ID,
  SET_MST_ID,
  DATA_OUTPUT_SELECTION,
  SAVE_PARAM = 254,
  RESTORE_SETTING = 255
} reg_id_e;

typedef struct {
  uint8_t can_id;
  uint8_t mst_id;
  CAN_HandleTypeDef* can_handle;
  float pitch, roll, yaw;
  float gyro[3], accel[3], q[4];
  float cur_temp;
} imu_t;

typedef struct {
  TaskHandle_t thread_alert;
  STM32CAN_t* can_;
  uint8_t can_id, mst_id;
  imu_t data;
  bool online_;
  err_t init_error_;
  uint8_t raw_data[8];
} IMUCAN_t;

extern imu_t imu;
err_t imu_can_init(IMUCAN_t* self, STM32CAN_t* can, uint8_t can_id,
                   uint8_t mst_id);
void imu_can_update(IMUCAN_t* self, uint32_t timeout_ms);
void imu_init(uint8_t can_id, uint8_t mst_id, CAN_HandleTypeDef* can_handle);
void imu_write_reg(uint8_t reg_id, uint32_t data);
void imu_read_reg(uint8_t reg_id);
void imu_reboot(void);
void imu_accel_calibration(void);
void imu_gyro_calibration(void);
void imu_change_com_port(imu_com_port_e port);
void imu_set_active_mode_delay(uint32_t delay);
void imu_change_to_active(void);
void imu_change_to_request(void);
void imu_set_baud(imu_baudrate_e baud);
void imu_set_can_id(uint8_t can_id);
void imu_set_mst_id(uint8_t mst_id);
void imu_save_parameters(void);
void imu_restore_settings(void);
void imu_request_accel(void);
void imu_request_gyro(void);
void imu_request_euler(void);
void imu_request_quat(void);
void IMU_UpdateData(uint8_t* data);

#endif
