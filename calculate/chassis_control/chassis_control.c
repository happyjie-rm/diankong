/**
 * @file chassis_control.c
 * @brief 固定 CAN2 / 0x200 四轮 M3508 底盘控制实现。
 *
 * 模块职责概览：
 * - 在 CAN2 上注册 4 台 M3508（设备 ID 1~4，命令组 0x200）。
 * - 将任务层下发的通用速度指令（forward/lateral/yaw）经麦克纳姆逆解
 *   得到四轮目标 RPM，再经速度环 PID 输出电流指令。
 * - 统一管理安全态：未初始化、安全失能、故障时强制零输出，并维持
 *   可重试的零帧心跳，避免 C620 因超时失控。
 *
 * 关键编译开关：
 * - CHASSIS_ACTUATION_ENABLED=0（默认）：只做解算与状态机，不向电机发
 *   非零命令，便于上电联调；置 1 后才真正闭环驱动。
 */

#include "chassis_control.h"

#include "dj_motor_ctrl.h"
#include "pid_location.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

/**
 * @brief 底盘控制内部上下文（模块私有，单例）。
 *
 * 任务层只通过公开 API 读写，不直接暴露本结构，避免并发/误写。
 */
typedef struct {
  dj_motor_bus_t bus; /**< DJ 电机总线（绑定 CAN2） */
  dj_motor_t
      motors[CHASSIS_WHEEL_COUNT]; /**< 四轮电机实例，顺序：左前/右前/左后/右后
                                    */
  PIDInstance speed_pid[CHASSIS_WHEEL_COUNT]; /**< 各轮速度环 PID 实例 */
  chassis_control_status_t status;            /**< 对外可查询的状态快照 */
  bool init_attempted; /**< 是否已尝试过初始化（防重复 init） */
  bool zero_sent;      /**< 本会话是否成功发过至少一帧零电流 */
} chassis_control_context_t;

/** 模块级单例上下文；初始为未初始化 + PENDING。 */
static chassis_control_context_t chassis_context = {
    .status =
        {
            .state = CONTROL_STATE_UNINITIALIZED,
            .last_error = PENDING,
        },
};

/**
 * @brief 四轮速度环 PID RAM 调参配置（与参考工程对齐）。
 *
 * 公共约束：
 * - MaxOut=12000：输出限幅对齐 M3508 电流指令量级。
 * - IntegralLimit=3000：积分限幅，抑制积分饱和。
 * - Improve 组合：积分限幅 + 微分基于测量 + 输出/微分低通，减小噪声抖动。
 *
 * 各轮 Kp/Ki 略有差异，用于补偿机械/安装不对称。配置表放在 SRAM 并声明为
 * volatile，便于 Ozone 在运行期间直接修改；主动控制时会同步到 PID 实例，
 * 失能或故障复位时则以本表重新初始化完整 PID 状态。
 */
static volatile PID_Init_Config_s
    chassis_speed_pid_configs[CHASSIS_WHEEL_COUNT] = {
        {
            /* 轮 0：左前 */
            .Kp = 12.0f,
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
            /* 轮 1：右前 */
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
        {
            /* 轮 2：左后 */
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
        {
            /* 轮 3：右后（含小积分项，便于静差收敛） */
            .Kp = 14.0f,
            .Ki = 2.0f,
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
 * @brief 将 RAM 调参配置应用到指定速度 PID。
 *
 * 主动控制路径只更新固定配置字段，保留积分、微分和滤波历史；安全失能或故障
 * 路径通过 PIDInit 完整复位运行状态。先生成局部快照，避免单个字段在一次应用
 * 过程中被重复读取。
 */
static void chassis_apply_pid_config(uint8_t index, bool reset_runtime_state) {
  PID_Init_Config_s config = chassis_speed_pid_configs[index];
  PIDInstance *pid = &chassis_context.speed_pid[index];

  if (reset_runtime_state) {
    PIDInit(pid, config.Kp, config.Ki, config.Kd, config.MaxOut,
            config.IntegralLimit, config.DeadBand, config.Improve, config.CoefA,
            config.CoefB, config.Output_LPF_RC, config.Derivative_LPF_RC);
    return;
  }
}

/**
 * @brief 重新初始化四路静态速度 PID。
 *
 * 进入安全零输出或故障态前调用，清除积分/历史微分，避免再次使能时
 * 旧积分造成电流冲击。
 */
static void chassis_reset_pids(void) {
  for (uint8_t index = 0U; index < CHASSIS_WHEEL_COUNT; ++index) {
    chassis_apply_pid_config(index, true);
  }
}

/**
 * @brief 清零状态中的目标转速与实际电流命令。
 *
 * 用于失能/故障路径，保证对外状态与“无运动意图”一致。
 */
static void chassis_clear_targets_and_commands(void) {
  for (uint8_t index = 0U; index < CHASSIS_WHEEL_COUNT; ++index) {
    chassis_context.status.target_rpm[index] = 0.0f;
    chassis_context.status.command[index] = 0;
  }
}

/**
 * @brief 按需发送 0x200 组安全零帧，并在发送失败时保留重试标记。
 *
 * 发送条件（满足任一即可）：
 * - force=true：调用方强制（状态切换、force_stop 等）；
 * - zero_retry_pending：上一帧零电流发送失败，需重试；
 * - 心跳到期：距上次成功零帧 ≥ CHASSIS_ZERO_HEARTBEAT_MS。
 *
 * 发送失败时置 zero_retry_pending，下周期继续尝试，防止电调超时。
 *
 * @param now_tick 当前毫秒 tick
 * @param force    是否强制发送（忽略心跳间隔）
 * @return 总线发送结果；未到发送时机返回 OK
 */
static err_t chassis_flush_zero_if_due(uint32_t now_tick, bool force) {
  /* 尚未成功发过零帧，或距离上次成功超过心跳间隔 → 需要心跳 */
  const bool heartbeat_due =
      !chassis_context.zero_sent ||
      ((uint32_t)(now_tick - chassis_context.status.last_zero_tick) >=
       CHASSIS_ZERO_HEARTBEAT_MS);

  /* 本地命令快照先清零，便于调试观察 */
  for (uint8_t index = 0U; index < CHASSIS_WHEEL_COUNT; ++index) {
    chassis_context.status.command[index] = 0;
  }

  /* 既不强制、也无重试、心跳未到：本周期不发帧 */
  if (!force && !chassis_context.status.zero_retry_pending && !heartbeat_due) {
    return OK;
  }

  const err_t result =
      dj_motor_zero_and_flush(&chassis_context.bus, DJ_MOTOR_GROUP_200);
  if (result == OK) {
    chassis_context.zero_sent = true;
    chassis_context.status.zero_retry_pending = false;
    chassis_context.status.last_zero_tick = now_tick;
  } else {
    /* 发送失败：保持重试，下周期继续尝试 */
    chassis_context.status.zero_retry_pending = true;
  }
  return result;
}

/**
 * @brief 进入只允许零输出的状态，并维持可重试零帧心跳。
 *
 * 典型入口：输入非法、遥控掉线、反馈超时、PID 异常、force_stop。
 * 流程：复位 PID → 可选清目标 → 更新状态/错误码 → 按需发零帧。
 * 若零帧发送失败，状态升为 FAULT，并把 last_error 改为发送错误。
 *
 * @param state         目标控制状态（SAFE_DISABLED / FAULT / READY 等）
 * @param cause         进入该态的原因错误码（成功路径可为 OK）
 * @param now_tick      当前毫秒 tick
 * @param force_zero    是否强制本周期发送零帧
 * @param clear_targets 是否同时清零 target_rpm（失能通常为 true）
 * @return 零帧失败时返回发送错误；否则返回 cause
 */
static err_t chassis_enter_zero_output(control_state_e state, err_t cause,
                                       uint32_t now_tick, bool force_zero,
                                       bool clear_targets) {
  const bool state_changed = chassis_context.status.state != state;
  chassis_reset_pids();
  if (clear_targets) {
    chassis_clear_targets_and_commands();
  } else {
    /* 保留目标用于调试观察，仅命令清零 */
    for (uint8_t index = 0U; index < CHASSIS_WHEEL_COUNT; ++index) {
      chassis_context.status.command[index] = 0;
    }
  }
  chassis_context.status.state = state;
  chassis_context.status.last_error = cause;

  /* 状态切换时强制发零帧，确保电调立刻收到安全电流 */
  const err_t zero_result =
      chassis_flush_zero_if_due(now_tick, force_zero || state_changed);
  if (zero_result != OK) {
    chassis_context.status.state = CONTROL_STATE_FAULT;
    chassis_context.status.last_error = zero_result;
    return zero_result;
  }
  return cause;
}

/**
 * @brief 刷新四台电机反馈与在线标记。
 *
 * 对每轮：读取 feedback，并用 CHASSIS_FEEDBACK_TIMEOUT_MS 判定在线。
 * 读失败时 feedback_rpm 记 0，该轮 online=false。
 *
 * @param now_tick 当前毫秒 tick
 * @return 四台均在线返回 true，否则 false
 */
static bool chassis_refresh_feedback(uint32_t now_tick) {
  bool all_online = true;

  for (uint8_t index = 0U; index < CHASSIS_WHEEL_COUNT; ++index) {
    dj_motor_feedback_t feedback = {0};
    const err_t result =
        dj_motor_get_feedback(&chassis_context.motors[index], &feedback);
    const bool online =
        (result == OK) &&
        dj_motor_is_online(&chassis_context.motors[index], now_tick,
                           CHASSIS_FEEDBACK_TIMEOUT_MS);
    chassis_context.status.wheels_online[index] = online;
    chassis_context.status.feedback_rpm[index] =
        (result == OK) ? feedback.speed_rpm : 0;
    if (!online) {
      all_online = false;
    }
  }
  return all_online;
}

/**
 * @brief 将有限的 PID 浮点输出转换为 M3508 协议电流命令。
 *
 * - NaN/Inf → 0，避免异常值下发。
 * - 钳位到 ±DJ_MOTOR_M3508_LIMIT 后截断为 int16。
 *
 * @param output PID 浮点输出
 * @return 协议电流命令（有符号 16 位）
 */
static int16_t chassis_output_to_command(float output) {
  if (!isfinite(output)) {
    return 0;
  }
  if (output > (float)DJ_MOTOR_M3508_LIMIT) {
    output = (float)DJ_MOTOR_M3508_LIMIT;
  } else if (output < -(float)DJ_MOTOR_M3508_LIMIT) {
    output = -(float)DJ_MOTOR_M3508_LIMIT;
  }
  return (int16_t)output;
}

/**
 * @brief 在指定 CAN2 对象上注册四台 M3508。
 *
 * 约束：
 * - 全程只允许调用一次（init_attempted 防重入）。
 * - can 必须非空且 id_ 为 BSP_CAN2。
 * - 电机设备 ID 固定为 1~4，对应 0x200 命令组。
 * 成功后进入 SAFE_DISABLED，并标记 zero_retry_pending，由后续 step
 * 尽快推送安全零帧。
 */
err_t chassis_control_init(STM32CAN_t *can) {
  if (chassis_context.init_attempted) {
    return STATE_ERR;
  }

  /* 整上下文清零，避免热重启残留状态 */
  memset(&chassis_context, 0, sizeof(chassis_context));
  chassis_context.status.state = CONTROL_STATE_UNINITIALIZED;
  chassis_context.status.last_error = PENDING;

  if (can == NULL) {
    chassis_context.status.state = CONTROL_STATE_FAULT;
    chassis_context.status.last_error = PTR_NULL;
    return PTR_NULL;
  }
  /* 本模块硬绑定 CAN2，防止误把云台总线传入 */
  if (can->id_ != BSP_CAN2) {
    chassis_context.status.state = CONTROL_STATE_FAULT;
    chassis_context.status.last_error = ARG_ERR;
    return ARG_ERR;
  }

  chassis_context.init_attempted = true;
  err_t result = dj_motor_bus_init(&chassis_context.bus, can);
  if (result != OK) {
    chassis_context.status.state = CONTROL_STATE_FAULT;
    chassis_context.status.last_error = result;
    return result;
  }

  /* 设备 ID = index+1 → 1/2/3/4；最后一个 false 表示非反向安装约定 */
  for (uint8_t index = 0U; index < CHASSIS_WHEEL_COUNT; ++index) {
    result = dj_motor_init(&chassis_context.motors[index], &chassis_context.bus,
                           DJ_MOTOR_M3508, (uint8_t)(index + 1U), false);
    if (result != OK) {
      chassis_context.status.state = CONTROL_STATE_FAULT;
      chassis_context.status.last_error = result;
      return result;
    }
  }

  chassis_reset_pids();
  chassis_context.status.initialized = true;
  /* 上电后尽快发零帧，避免电调处于未定义电流 */
  chassis_context.status.zero_retry_pending = true;
  chassis_context.status.state = CONTROL_STATE_SAFE_DISABLED;
  chassis_context.status.last_error = OK;
  return OK;
}

/**
 * @brief 执行一次四轮反馈检查、速度闭环和安全输出。
 *
 * 推荐由底盘任务周期（如 1ms/2ms）调用。处理顺序：
 * 1) 未初始化 / 空指针 / 非有限指令 → 故障零输出；
 * 2) 指令源掉线或 enable=false → 安全零输出；
 * 3) 任一轮反馈超时 → 故障零输出；
 * 4) 逆运动学解算四轮目标 RPM；
 * 5) CHASSIS_ACTUATION_ENABLED=1 时做速度闭环并下发；
 *    否则仅解算，保持 READY 并按心跳发零帧。
 */
err_t chassis_control_step(const chassis_control_input_t *input,
                           uint32_t now_tick) {
  if (!chassis_context.status.initialized) {
    return STATE_ERR;
  }

  chassis_context.status.last_step_tick = now_tick;
  if (input == NULL) {
    chassis_context.status.source_online = false;
    return chassis_enter_zero_output(CONTROL_STATE_FAULT, PTR_NULL, now_tick,
                                     true, true);
  }
  chassis_context.status.source_online = input->source_online;

  /* 拒绝 NaN/Inf，防止污染 PID 与电机命令 */
  if (!isfinite(input->forward) || !isfinite(input->lateral) ||
      !isfinite(input->yaw)) {
    return chassis_enter_zero_output(CONTROL_STATE_FAULT, ARG_ERR, now_tick,
                                     true, true);
  }
  /* 指令源离线：保持零输出，允许按心跳重试，不强制每周期刷帧 */
  if (!input->source_online) {
    return chassis_enter_zero_output(CONTROL_STATE_FAULT, NO_RESPONSE, now_tick,
                                     false, true);
  }
  /* 使能关闭（任务层：左拨杆 sw_l 为 UP/非法 等）：安全失能 */
  if (!input->enable) {
    return chassis_enter_zero_output(CONTROL_STATE_SAFE_DISABLED, OK, now_tick,
                                     false, true);
  }
  /* 任一电机反馈超时则整盘停转 */
  if (!chassis_refresh_feedback(now_tick)) {
    return chassis_enter_zero_output(CONTROL_STATE_FAULT, TIMEOUT, now_tick,
                                     false, true);
  }

  /* 麦克纳姆逆解：摇杆归一化 × CHASSIS_COMMAND_SCALE → 目标 RPM */
  chassis_kinematics_mecanum(input->forward, input->lateral, input->yaw,
                             CHASSIS_COMMAND_SCALE,
                             chassis_context.status.target_rpm);

#if CHASSIS_ACTUATION_ENABLED
  /* 零帧仍在重试：禁止切入闭环，先完成安全输出 */
  if (chassis_context.status.zero_retry_pending) {
    return chassis_enter_zero_output(CONTROL_STATE_SAFE_DISABLED, OK, now_tick,
                                     true, true);
  }

  /* 四轮速度环：测量=反馈 RPM，设定=逆解目标 RPM */
  for (uint8_t index = 0U; index < CHASSIS_WHEEL_COUNT; ++index) {
    chassis_apply_pid_config(index, false);
    const float output =
        PIDCalculate(&chassis_context.speed_pid[index],
                     (float)chassis_context.status.feedback_rpm[index],
                     chassis_context.status.target_rpm[index]);
    if (chassis_context.speed_pid[index].ERRORHandler.ERRORType !=
        PID_ERROR_NONE) {
      return chassis_enter_zero_output(CONTROL_STATE_FAULT, FAILED, now_tick,
                                       true, true);
    }
    chassis_context.status.command[index] = chassis_output_to_command(output);
  }

  chassis_context.status.state = CONTROL_STATE_ACTIVE;
  /* 先写各电机命令缓冲，再由 dj 层组 0x200 帧发送 */
  for (uint8_t index = 0U; index < CHASSIS_WHEEL_COUNT; ++index) {
    const err_t result = dj_motor_set_command(
        &chassis_context.motors[index], chassis_context.status.command[index]);
    if (result != OK) {
      return chassis_enter_zero_output(CONTROL_STATE_FAULT, result, now_tick,
                                       true, true);
    }
  }

  chassis_context.status.last_error = OK;
  chassis_context.status.zero_retry_pending = false;
  return OK;
#else
  /* 联调模式：避免未使用静态函数告警，同时保持目标可观测 */
  (void)chassis_output_to_command;
  return chassis_enter_zero_output(CONTROL_STATE_READY, OK, now_tick, false,
                                   false);
#endif
}

/**
 * @brief 立即复位四路 PID 并尝试发送 0x200 全零帧。
 *
 * 紧急停机入口：置 zero_retry_pending，强制进入 SAFE_DISABLED。
 * now_tick 使用上次 step 的 tick，保证心跳时间基准连续。
 */
err_t chassis_control_force_stop(void) {
  if (!chassis_context.status.initialized) {
    return STATE_ERR;
  }
  chassis_context.status.zero_retry_pending = true;
  return chassis_enter_zero_output(CONTROL_STATE_SAFE_DISABLED, OK,
                                   chassis_context.status.last_step_tick, true,
                                   true);
}

/**
 * @brief 读取当前底盘控制状态快照。
 *
 * 返回的是结构体值拷贝，调用方可安全持有而不影响内部上下文。
 */
err_t chassis_control_get_status(chassis_control_status_t *status) {
  if (status == NULL) {
    return PTR_NULL;
  }
  *status = chassis_context.status;
  return OK;
}
