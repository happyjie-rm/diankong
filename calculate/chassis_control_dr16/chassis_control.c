/**
 * @file chassis_control.c
 * @brief 麦克纳姆轮底盘控制实现
 */
#include "chassis_control.h"

#include <math.h>
#include <stdbool.h>

#include "bsp_can.h"
#include "can.h"
#include "chassis_dynamics.h"
#include "dj_motor_ctrl.h"
#include "dr16.h"
#include "pid_location.h"
#include "process.h"

/* DR16 task owns the decoded command and updates it periodically. */
extern DR16_t* dr16;
extern uint8_t joint_enable_single;

/* 底盘电机总线与实例 */
dj_motor_bus_t chassis_bus;
dj_motor_t chassis_motors[CHASSIS_MOTOR_COUNT];

/* 速度环 PID 实例 */
static PIDInstance pid_speed[CHASSIS_MOTOR_COUNT];

/* 底盘状态与目标速度 */
static chassis_control_state_t chassis_control_state_;
static float motor_target_speed[CHASSIS_MOTOR_COUNT];
static float torque_ff_current[CHASSIS_MOTOR_COUNT];
#define CHASSIS_GRAVITY_N 20.0f
#define CHASSIS_TORQUE_TO_CURRENT 1000.0f
#define CHASSIS_WHEEL_RADIUS_M 0.076f
#define CHASSIS_SMALL_GYRO_SPEED 1000.0f

/**
 * @brief 初始化底盘 CAN 总线与四个 M3508 电机
 * @return OK 或错误码
 */
err_t chassis_control_init(void) {
  /* 获取 CAN2 的 BSP 对象（假设 CAN2 用于底盘） */
  BSP_CAN_t can_id = BSP_CAN_get_id(CAN2);
  if (can_id == BSP_CAN_ID_ERROR) {
    return NOT_FOUND;
  }

  STM32CAN_t* can2 = STM32CAN_GetInstance(can_id);
  if (can2 == NULL) {
    return PTR_NULL;
  }

  /* 初始化底盘总线 */
  err_t result = dj_motor_bus_init(&chassis_bus, can2);
  if (result != OK) {
    return result;
  }

  /* 初始化四个 M3508 底盘电机（控制组 0x200）
   * 实车电机编号与轮位的对应关系：
   *   左前轮 FL = 4 号电机
   *   右前轮 FR = 3 号电机
   *   左后轮 RL = 2 号电机
   *   右后轮 RR = 1 号电机
   * reversed 参数根据实际机械安装方向设置 */
  result = dj_motor_init(&chassis_motors[CHASSIS_MOTOR_FL], &chassis_bus,
                         DJ_MOTOR_M3508, 4, false); /* 左前轮 = 4 号 */
  if (result != OK) return result;

  result = dj_motor_init(&chassis_motors[CHASSIS_MOTOR_FR], &chassis_bus,
                         DJ_MOTOR_M3508, 3, false); /* 右前轮 = 3 号 */
  if (result != OK) return result;

  result = dj_motor_init(&chassis_motors[CHASSIS_MOTOR_RL], &chassis_bus,
                         DJ_MOTOR_M3508, 2, false); /* 左后轮 = 2 号 */
  if (result != OK) return result;

  result = dj_motor_init(&chassis_motors[CHASSIS_MOTOR_RR], &chassis_bus,
                         DJ_MOTOR_M3508, 1, false); /* 右后轮 = 1 号 */
  if (result != OK) return result;

  return OK;
}

static void chassis_speed_pid_init_single(PIDInstance* pid, float kp, float ki,
                                          float kd, float max_out) {
  PIDInit(pid, kp, ki, kd, max_out, 3000.0f, 0.0f,
          PID_Integral_Limit | PID_Derivative_On_Measurement |
              PID_OutputFilter | PID_DerivativeFilter,
          0.0f, 0.0f, 0.0002f, 0.0002f);
}

void chassis_speed_pid_init(void) {
  chassis_speed_pid_init_single(&pid_speed[CHASSIS_MOTOR_FL], 12.0f, 0.0f, 0.0f,
                                12000.0f);
  chassis_speed_pid_init_single(&pid_speed[CHASSIS_MOTOR_FR], 8.0f, 0.0f, 0.0f,
                                12000.0f);
  chassis_speed_pid_init_single(&pid_speed[CHASSIS_MOTOR_RL], 8.0f, 0.0f, 0.0f,
                                12000.0f);
  chassis_speed_pid_init_single(&pid_speed[CHASSIS_MOTOR_RR], 14.0f, 2.0f, 0.0f,
                                12000.0f);
}

static void chassis_motor_pid_control_speed(uint8_t motor_index,
                                            float target_speed) {
  if (motor_index >= CHASSIS_MOTOR_COUNT) return;

  /* 获取电机反馈 */
  dj_motor_feedback_t feedback;
  if (dj_motor_get_feedback(&chassis_motors[motor_index], &feedback) != OK) {
    return;
  }

  /* 速度环 PID 控制（反馈为 rad/s，需要转换或统一单位）
   * 这里假设目标速度与反馈速度单位一致（RPM） */
  float current_speed = feedback.speed_rpm;
  float motor_current =
      PIDCalculate(&pid_speed[motor_index], current_speed, target_speed);

  /* 限幅到 M3508 允许范围 */
  motor_current += torque_ff_current[motor_index];
  motor_current = CONSTRAIN(motor_current, -16384.0f, 16384.0f);

  /* 发送命令（写齐后自动发送） */
  dj_motor_set_command(&chassis_motors[motor_index], (int16_t)motor_current);
}

static void chassis_stop(void) {
  chassis_control_state_.command.vx = 0.0f;
  chassis_control_state_.command.vy = 0.0f;
  chassis_control_state_.command.wz = 0.0f;

  /* 使用 zero_and_flush 安全停机（推荐的停机方式） */
  dj_motor_zero_and_flush(&chassis_bus, DJ_MOTOR_GROUP_200);
}

static void chassis_control(void) {
  const cmd_rc_t* command = &dr16->dr16_cmd;

  chassis_dynamics_feedforward(torque_ff_current);
  /*
   * 统一底盘坐标系：
   *   +X：前
   *   +Y：左
   *   +Z：上
  *   +wz：顺时针
   *
   * DR16 左摇杆：l.x / l.y
   *   注意：这里保留当前遥控器通道的实际方向反号，
   *   只把它统一映射到机器人底盘坐标系。
   */
  chassis_control_state_.command.vx = -command->ch.l.y * 3000.0f;  // +X：前
  chassis_control_state_.command.vy = -command->ch.l.x * 3000.0f;  // +Y：左
  chassis_control_state_.command.wz = command->ch.r.x * 3000.0f;  // +wz：顺时针

  /*
   * 麦克纳姆轮逆运动学：
   *   底盘坐标系：
   *       +X：前
   *       +Y：左
   *       +Z：上
  *       +wz：顺时针
   *
   *             前（+X）
   *                ↑
   *        4号 FL     3号 FR
   *        左前       右前
   *
   *        2号 RL     1号 RR
   *        左后       右后
   *
   *   输出数组顺序固定为 [FL, FR, RL, RR] = [4, 3, 2, 1]。
   */
  /* 麦克纳姆轮运动学解算 */
  chassis_dynamics_inverse(
      chassis_control_state_.command.vx, chassis_control_state_.command.vy,
      chassis_control_state_.command.wz, motor_target_speed);

  /* 执行速度环 PID 控制（写齐后自动发送 CAN 帧） */
  for (uint8_t i = 0; i < CHASSIS_MOTOR_COUNT; i++) {
    chassis_motor_pid_control_speed(i, motor_target_speed[i]);
  }
}

/**
 * @brief 小陀螺模式
 *
 * 左拨杆 DOWN 时：
 *   - 左摇杆仍然有效：控制前后/左右平移
 *   - 右摇杆完全不参与底盘控制
 *   - 即使左摇杆回中，底盘仍保持固定 wz 原地旋转
 *
 * 因此：
 *   vx = 左摇杆前后
 *   vy = 左摇杆左右
 *   wz = 固定小陀螺速度
 */
static void chassis_small_gyro_control(void) {
  const cmd_rc_t* command = &dr16->dr16_cmd;

  chassis_dynamics_feedforward(torque_ff_current);

  /*
   * 左摇杆：
   *   l.y -> 前后移动 vx
   *   l.x -> 左右移动 vy
   *
   * 右摇杆：
   *   r.x 不读取，因此不会影响小陀螺旋转速度/方向。
   */
  chassis_control_state_.command.vx = -command->ch.l.y * 5000.0f;
  chassis_control_state_.command.vy = -command->ch.l.x * 5000.0f;

  /* 固定原地旋转速度：不动左摇杆时也会持续自转。 */
  chassis_control_state_.command.wz = CHASSIS_SMALL_GYRO_SPEED;

  chassis_dynamics_inverse(
      chassis_control_state_.command.vx, chassis_control_state_.command.vy,
      chassis_control_state_.command.wz, motor_target_speed);

  for (uint8_t i = 0; i < CHASSIS_MOTOR_COUNT; i++) {
    chassis_motor_pid_control_speed(i, motor_target_speed[i]);
  }
}

void Chassis_Mode(void) {
  /* 接收机掉线：底盘必须停止 */
  if ((dr16 == NULL) || !dr16->online_) {
    chassis_stop();
    return;
  }

  if (dr16->dr16_cmd.sw_l == CMD_SW_MID) {
    /*
     * MID：正常底盘模式
     * 左摇杆 -> 前后/左右
     * 右摇杆 -> 旋转
     */
    if (joint_enable_single == 1) {
      chassis_control();
    } else {
      chassis_stop();
    }
  } else if (dr16->dr16_cmd.sw_l == CMD_SW_DOWN) {
    /*
     * DOWN：小陀螺模式
     *
     * 左摇杆仍然有效：
     *   左摇杆前后 -> 底盘前后移动
     *   左摇杆左右 -> 底盘左右移动
     *
     * 右摇杆不读取。
     *
     * 即使左摇杆完全回中：
     *   vx = 0
     *   vy = 0
     *   wz = 固定 3000
     * 底盘仍然持续原地旋转。
     */
    chassis_stop();
  } else {
    /* UP 或异常状态：安全停止 */
    chassis_stop();
  }
}
