/**
 * @file gimbal_control.c
 * @brief 固定 CAN1 / 0x1FF 双轴 GM6020 云台控制实现。
 */
#include "gimbal_control.h"

#include "dj_motor_ctrl.h"
#include "pid_location.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

enum {
  GIMBAL_YAW_INDEX = 0,
  GIMBAL_PITCH_INDEX,
  GIMBAL_AXIS_COUNT
};

typedef struct {
  dj_motor_bus_t bus;
  dj_motor_t motors[GIMBAL_AXIS_COUNT];
  PIDInstance speed_pid[GIMBAL_AXIS_COUNT];
  gimbal_control_status_t status;
  bool init_attempted;
  bool zero_sent;
} gimbal_control_context_t;

static gimbal_control_context_t gimbal_context = {
    .status = {
        .state = CONTROL_STATE_UNINITIALIZED,
        .last_error = PENDING,
    },
};

static const PID_Init_Config_s gimbal_speed_pid_configs[GIMBAL_AXIS_COUNT] = {
    {
        /* 轮 0：yaw */
        .Kp = 0.0f,
        .Ki = 0.0f,
        .Kd = 0.0f,
        .MaxOut = 12000.0f,
        .DeadBand = 0.0f,
        .Improve = PID_Integral_Limit | PID_Derivative_On_Measurement |
                   PID_OutputFilter | PID_DerivativeFilter,
        .IntegralLimit = 3000.0f,
        .CoefA = 0.0f,
        .CoefB = 0.0f,
        .Output_LPF_RC = 0.0002f,
        .Derivative_LPF_RC = 0.0002f,
    },
    {
        /* 轮 1：pitch */
        .Kp = 8.0f,
        .Ki = 0.0f,
        .Kd = 0.0f,
        .MaxOut = 12000.0f,
        .DeadBand = 0.0f,
        .Improve = PID_Integral_Limit | PID_Derivative_On_Measurement |
                   PID_OutputFilter | PID_DerivativeFilter,
        .IntegralLimit = 3000.0f,
        .CoefA = 0.0f,
        .CoefB = 0.0f,
        .Output_LPF_RC = 0.0002f,
        .Derivative_LPF_RC = 0.0002f,
    },
};

/**
 * @brief 重新初始化两路静态速度 PID，清除全部运行状态。
 */
static void gimbal_reset_pids(void) {
  for (uint8_t index = 0U; index < GIMBAL_AXIS_COUNT; ++index) {
    PID_Init_Config_s config = gimbal_speed_pid_config;
    PIDInit(&gimbal_context.speed_pid[index], &config);
  }
}

/**
 * @brief 清零状态中的目标与实际命令。
 */
static void gimbal_clear_targets_and_commands(void) {
  gimbal_context.status.yaw_target_rpm = 0.0f;
  gimbal_context.status.pitch_target_rpm = 0.0f;
  gimbal_context.status.yaw_command = 0;
  gimbal_context.status.pitch_command = 0;
}

/**
 * @brief 按需发送安全零帧，并在发送失败时保留重试标记。
 *
 * Args:
 *   now_tick: 当前毫秒 tick。
 *   force: true 时忽略心跳间隔立即发送。
 *
 * Returns:
 *   未到发送时机或发送成功返回 OK，否则返回底层发送错误。
 */
static err_t gimbal_flush_zero_if_due(uint32_t now_tick, bool force) {
  const bool heartbeat_due =
      !gimbal_context.zero_sent ||
      ((uint32_t)(now_tick - gimbal_context.status.last_zero_tick) >=
       GIMBAL_ZERO_HEARTBEAT_MS);

  gimbal_context.status.yaw_command = 0;
  gimbal_context.status.pitch_command = 0;
  if (!force && !gimbal_context.status.zero_retry_pending && !heartbeat_due) {
    return OK;
  }

  const err_t result =
      dj_motor_zero_and_flush(&gimbal_context.bus, DJ_MOTOR_GROUP_1FF);
  if (result == OK) {
    gimbal_context.zero_sent = true;
    gimbal_context.status.zero_retry_pending = false;
    gimbal_context.status.last_zero_tick = now_tick;
  } else {
    gimbal_context.status.zero_retry_pending = true;
  }
  return result;
}

/**
 * @brief 进入只允许零输出的状态并维持可重试零帧心跳。
 *
 * Args:
 *   state: SAFE_DISABLED、READY 或 FAULT。
 *   cause: 触发状态转换的结果码。
 *   now_tick: 当前毫秒 tick。
 *   force_zero: true 时立即冲刷零帧。
 *   clear_targets: true 时同时清除状态中的速度目标。
 *
 * Returns:
 *   零帧发送失败时返回发送错误，否则返回 cause。
 */
static err_t gimbal_enter_zero_output(control_state_e state, err_t cause,
                                      uint32_t now_tick, bool force_zero,
                                      bool clear_targets) {
  const bool state_changed = gimbal_context.status.state != state;
  gimbal_reset_pids();
  if (clear_targets) {
    gimbal_clear_targets_and_commands();
  } else {
    gimbal_context.status.yaw_command = 0;
    gimbal_context.status.pitch_command = 0;
  }
  gimbal_context.status.state = state;
  gimbal_context.status.last_error = cause;

  const err_t zero_result =
      gimbal_flush_zero_if_due(now_tick, force_zero || state_changed);
  if (zero_result != OK) {
    gimbal_context.status.state = CONTROL_STATE_FAULT;
    gimbal_context.status.last_error = zero_result;
    return zero_result;
  }
  return cause;
}

/**
 * @brief 刷新两台电机反馈与在线标记。
 *
 * Args:
 *   now_tick: 当前毫秒 tick。
 *
 * Returns:
 *   两台电机均能读取反馈且未超过 20ms 时返回 true。
 */
static bool gimbal_refresh_feedback(uint32_t now_tick) {
  dj_motor_feedback_t yaw_feedback = {0};
  dj_motor_feedback_t pitch_feedback = {0};
  const err_t yaw_result = dj_motor_get_feedback(
      &gimbal_context.motors[GIMBAL_YAW_INDEX], &yaw_feedback);
  const err_t pitch_result = dj_motor_get_feedback(
      &gimbal_context.motors[GIMBAL_PITCH_INDEX], &pitch_feedback);

  gimbal_context.status.yaw_online =
      (yaw_result == OK) &&
      dj_motor_is_online(&gimbal_context.motors[GIMBAL_YAW_INDEX], now_tick,
                         GIMBAL_FEEDBACK_TIMEOUT_MS);
  gimbal_context.status.pitch_online =
      (pitch_result == OK) &&
      dj_motor_is_online(&gimbal_context.motors[GIMBAL_PITCH_INDEX], now_tick,
                         GIMBAL_FEEDBACK_TIMEOUT_MS);
  gimbal_context.status.yaw_feedback_rpm =
      (yaw_result == OK) ? yaw_feedback.speed_rpm : 0;
  gimbal_context.status.pitch_feedback_rpm =
      (pitch_result == OK) ? pitch_feedback.speed_rpm : 0;
  return gimbal_context.status.yaw_online &&
         gimbal_context.status.pitch_online;
}

/**
 * @brief 将有限的 PID 浮点输出转换为 GM6020 协议命令。
 *
 * Args:
 *   output: PID 输出。
 *
 * Returns:
 *   限幅到 GM6020 协议范围的整数命令；非有限值返回 0。
 */
static int16_t gimbal_output_to_command(float output) {
  if (!isfinite(output)) {
    return 0;
  }
  if (output > (float)DJ_MOTOR_GM6020_LIMIT) {
    output = (float)DJ_MOTOR_GM6020_LIMIT;
  } else if (output < -(float)DJ_MOTOR_GM6020_LIMIT) {
    output = -(float)DJ_MOTOR_GM6020_LIMIT;
  }
  return (int16_t)output;
}

/**
 * @brief 在指定 CAN1 对象上注册 yaw/pitch 两台 GM6020。
 */
err_t gimbal_control_init(STM32CAN_t *can) {
  if (gimbal_context.init_attempted) {
    return STATE_ERR;
  }

  memset(&gimbal_context, 0, sizeof(gimbal_context));
  gimbal_context.status.state = CONTROL_STATE_UNINITIALIZED;
  gimbal_context.status.last_error = PENDING;

  if (can == NULL) {
    gimbal_context.status.state = CONTROL_STATE_FAULT;
    gimbal_context.status.last_error = PTR_NULL;
    return PTR_NULL;
  }
  if (can->id_ != BSP_CAN1) {
    gimbal_context.status.state = CONTROL_STATE_FAULT;
    gimbal_context.status.last_error = ARG_ERR;
    return ARG_ERR;
  }

  gimbal_context.init_attempted = true;
  err_t result = dj_motor_bus_init(&gimbal_context.bus, can);
  if (result != OK) {
    gimbal_context.status.state = CONTROL_STATE_FAULT;
    gimbal_context.status.last_error = result;
    return result;
  }

  result = dj_motor_init(&gimbal_context.motors[GIMBAL_YAW_INDEX],
                         &gimbal_context.bus, DJ_MOTOR_GM6020, 1U, false);
  if (result == OK) {
    result = dj_motor_init(&gimbal_context.motors[GIMBAL_PITCH_INDEX],
                           &gimbal_context.bus, DJ_MOTOR_GM6020, 2U, false);
  }
  if (result != OK) {
    gimbal_context.status.state = CONTROL_STATE_FAULT;
    gimbal_context.status.last_error = result;
    return result;
  }

  gimbal_reset_pids();
  gimbal_context.status.initialized = true;
  gimbal_context.status.zero_retry_pending = true;
  gimbal_context.status.state = CONTROL_STATE_SAFE_DISABLED;
  gimbal_context.status.last_error = OK;
  return OK;
}

/**
 * @brief 执行一次双轴反馈检查、速度闭环和安全输出。
 */
err_t gimbal_control_step(const gimbal_control_input_t *input,
                          uint32_t now_tick) {
  if (!gimbal_context.status.initialized) {
    return STATE_ERR;
  }

  gimbal_context.status.last_step_tick = now_tick;
  if (input == NULL) {
    gimbal_context.status.source_online = false;
    return gimbal_enter_zero_output(CONTROL_STATE_FAULT, PTR_NULL, now_tick,
                                    true, true);
  }
  gimbal_context.status.source_online = input->source_online;

  if (!isfinite(input->yaw_speed_rpm) ||
      !isfinite(input->pitch_speed_rpm)) {
    return gimbal_enter_zero_output(CONTROL_STATE_FAULT, ARG_ERR, now_tick,
                                    true, true);
  }
  if (!input->source_online) {
    return gimbal_enter_zero_output(CONTROL_STATE_FAULT, NO_RESPONSE,
                                    now_tick, false, true);
  }
  if (!input->enable) {
    return gimbal_enter_zero_output(CONTROL_STATE_SAFE_DISABLED, OK,
                                    now_tick, false, true);
  }
  if (!gimbal_refresh_feedback(now_tick)) {
    return gimbal_enter_zero_output(CONTROL_STATE_FAULT, TIMEOUT, now_tick,
                                    false, true);
  }

  gimbal_context.status.yaw_target_rpm = input->yaw_speed_rpm;
  gimbal_context.status.pitch_target_rpm = input->pitch_speed_rpm;
#if GIMBAL_ACTUATION_ENABLED
  if (gimbal_context.status.zero_retry_pending) {
    return gimbal_enter_zero_output(CONTROL_STATE_SAFE_DISABLED, OK,
                                    now_tick, true, true);
  }

  const float yaw_output = PIDCalculate(
      &gimbal_context.speed_pid[GIMBAL_YAW_INDEX],
      (float)gimbal_context.status.yaw_feedback_rpm,
      input->yaw_speed_rpm);
  const float pitch_output = PIDCalculate(
      &gimbal_context.speed_pid[GIMBAL_PITCH_INDEX],
      (float)gimbal_context.status.pitch_feedback_rpm,
      input->pitch_speed_rpm);
  if ((gimbal_context.speed_pid[GIMBAL_YAW_INDEX]
           .ERRORHandler.ERRORType != PID_ERROR_NONE) ||
      (gimbal_context.speed_pid[GIMBAL_PITCH_INDEX]
           .ERRORHandler.ERRORType != PID_ERROR_NONE)) {
    return gimbal_enter_zero_output(CONTROL_STATE_FAULT, FAILED, now_tick,
                                    true, true);
  }

  gimbal_context.status.yaw_command = gimbal_output_to_command(yaw_output);
  gimbal_context.status.pitch_command =
      gimbal_output_to_command(pitch_output);
  gimbal_context.status.state = CONTROL_STATE_ACTIVE;

  err_t result = dj_motor_set_command(
      &gimbal_context.motors[GIMBAL_YAW_INDEX],
      gimbal_context.status.yaw_command);
  if (result == OK) {
    result = dj_motor_set_command(
        &gimbal_context.motors[GIMBAL_PITCH_INDEX],
        gimbal_context.status.pitch_command);
  }
  if (result != OK) {
    return gimbal_enter_zero_output(CONTROL_STATE_FAULT, result, now_tick,
                                    true, true);
  }

  gimbal_context.status.last_error = OK;
  gimbal_context.status.zero_retry_pending = false;
  return OK;
#else
  (void)gimbal_output_to_command;
  return gimbal_enter_zero_output(CONTROL_STATE_READY, OK, now_tick, false,
                                  false);
#endif
}

/**
 * @brief 立即复位双轴 PID 并尝试发送 0x1FF 全零帧。
 */
err_t gimbal_control_force_stop(void) {
  if (!gimbal_context.status.initialized) {
    return STATE_ERR;
  }
  gimbal_context.status.zero_retry_pending = true;
  return gimbal_enter_zero_output(CONTROL_STATE_SAFE_DISABLED, OK,
                                  gimbal_context.status.last_step_tick, true,
                                  true);
}

/**
 * @brief 读取当前云台控制状态快照。
 */
err_t gimbal_control_get_status(gimbal_control_status_t *status) {
  if (status == NULL) {
    return PTR_NULL;
  }
  *status = gimbal_context.status;
  return OK;
}
