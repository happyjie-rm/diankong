/**
 * @file controller.c
 * @author wanghongxi
 * @author modified by neozng
 * @brief  PID控制器定义
 * @version beta
 * @date 2022-11-01
 *
 * @copyrightCopyright (c) 2022 HNU YueLu EC all rights reserved
 */
#include "pid_location.h"

#include "bsp_dwt.h"

#include <string.h>

//* ================= 位置式 PID 优化环节实现 =================
//? 以下静态函数只修改 PIDInstance 的中间项，最终输出仍由 PIDCalculate() 统一汇总。

static void *zmalloc(size_t size) {
  void *ptr = malloc(size);
  if (ptr == NULL) {
    return NULL;
  }

  memset(ptr, 0, size);
  return ptr;
}

//? 梯形积分：用本轮误差和上轮误差的平均值替代矩形积分，降低低采样频率下的积分误差。
static void f_Trapezoid_Intergral(PIDInstance *pid) {
  // 计算梯形的面积,(上底+下底)*高/2
  pid->ITerm = pid->Ki * ((pid->Err + pid->Last_Err) / 2) * pid->dt;
}

//? 变速积分：误差小时保持积分作用，误差变大时线性削弱积分，降低启动或大阶跃时的超调。
static void f_Changing_Integration_Rate(PIDInstance *pid) {
  if (pid->Err * pid->Iout > 0) {
    // 积分呈累积趋势
    if (abs(pid->Err) <= pid->CoefB)
      return; // Full integral
    if (abs(pid->Err) <= (pid->CoefA + pid->CoefB))
      pid->ITerm *= (pid->CoefA - abs(pid->Err) + pid->CoefB) / pid->CoefA;
    else // 最大阈值,不使用积分
      pid->ITerm = 0;
  }
}

//! 积分限幅同时限制 ITerm 和 Iout，避免输出已经饱和时继续累积同向积分。
static void f_Integral_Limit(PIDInstance *pid) {
  static float temp_Output, temp_Iout;
  temp_Iout = pid->Iout + pid->ITerm;
  temp_Output = pid->Pout + pid->Iout + pid->Dout;
  if (abs(temp_Output) > pid->MaxOut && (pid->Err * pid->Iout > 0)) {
    pid->ITerm = 0; // 当前积分项置零
  }

  if (temp_Iout > pid->IntegralLimit) {
    pid->ITerm = 0;
    pid->Iout = pid->IntegralLimit;
  }
  if (temp_Iout < -pid->IntegralLimit) {
    pid->ITerm = 0;
    pid->Iout = -pid->IntegralLimit;
  }
}

//? 微分先行：只对反馈值求微分，避免 Ref 突变时微分项产生尖峰冲击。
static void f_Derivative_On_Measurement(PIDInstance *pid) {
  pid->Dout = pid->Kd * (pid->Last_Measure - pid->Measure) / pid->dt;
}

//? 微分滤波：一阶低通平滑 Dout，用 RC 参数在响应速度和噪声抑制之间折中。
static void f_Derivative_Filter(PIDInstance *pid) {
  pid->Dout = pid->Dout * pid->dt / (pid->Derivative_LPF_RC + pid->dt) +
              pid->Last_Dout * pid->Derivative_LPF_RC /
                  (pid->Derivative_LPF_RC + pid->dt);
}

//? 输出滤波：对最终 Output 做一阶低通，适合抑制执行器指令抖动。
static void f_Output_Filter(PIDInstance *pid) {
  pid->Output =
      pid->Output * pid->dt / (pid->Output_LPF_RC + pid->dt) +
      pid->Last_Output * pid->Output_LPF_RC / (pid->Output_LPF_RC + pid->dt);
}

//! 输出限幅是最后一道保护，所有优化环节计算完成后都必须经过这里。
static void f_Output_Limit(PIDInstance *pid) {
  if (pid->Output > pid->MaxOut) {
    pid->Output = pid->MaxOut;
  }
  if (pid->Output < -(pid->MaxOut)) {
    pid->Output = -(pid->MaxOut);
  }
}

//? 电机堵转检测：输出接近最大且反馈长期跟不上 Ref 时累计 ERRORCount。
//! 该逻辑会在堵转后反向 Ref，开启 PID_ErrorHandle 前需要确认执行器允许反向解堵。
static void f_PID_ErrorHandle(PIDInstance *pid) {
  /*Motor Blocked Handle*/
  if (fabsf(pid->Output) < pid->MaxOut * 0.001f || fabsf(pid->Ref) < 0.0001f)
    return;

  if ((fabsf(pid->Ref - pid->Last_Measure) / fabsf(pid->Ref)) > 0.95f) {
    // Motor blocked counting
    pid->ERRORHandler.ERRORCount++;
  } else {
    pid->ERRORHandler.ERRORCount = 0;
    pid->ERRORHandler.ERRORType &= ~PID_MOTOR_BLOCKED_ERROR;
  }

  if (pid->ERRORHandler.ERRORCount > 500) {
    // Motor blocked over 1000times
    pid->ERRORHandler.ERRORType = PID_MOTOR_BLOCKED_ERROR;
  }
}

//* ================= 位置式 PID 外部算法接口 =================

/**
 * @brief 初始化PID,设置参数和启用的优化环节,将其他数据置零
 *
 * @param pid    PID实例
 * @param config PID初始化设置
 */
void PIDInit(PIDInstance *pid, PID_Init_Config_s *config) {
  if ((pid == NULL) || (config == NULL)) {
    return;
  }
  //? config 的字段与 PIDInstance 起始配置区连续一致，因此旧实现通过 memcpy 快速复制。
  //! 若调整 PIDInstance 前部字段顺序，必须同步检查这里的 memcpy 范围。
  memset(pid, 0, sizeof(PIDInstance));
  memcpy(pid, config, sizeof(PID_Init_Config_s));
  DWT_GetDeltaT(&pid->DWT_CNT);
}
/**
 * @brief 初始化PID,设置参数和启用的优化环节,将其他数据置零
 * @param config PID初始化设置
 * @return PIDInstance* PID实例指针
 */
PIDInstance *PIDRegister(PID_Init_Config_s *config) {
  PIDInstance *pid = NULL;
  if (config == NULL) {
    return NULL;
  }
  pid = (PIDInstance *)zmalloc(sizeof(PIDInstance));
  if (pid == NULL) {
    return NULL;
  }
  pid->Kp = config->Kp;
  pid->Ki = config->Ki;
  pid->Kd = config->Kd;
  pid->MaxOut = config->MaxOut;
  pid->DeadBand = config->DeadBand;

  pid->Improve = config->Improve;
  pid->IntegralLimit = config->IntegralLimit;
  pid->CoefA = config->CoefA;
  pid->CoefB = config->CoefB;
  pid->Output_LPF_RC = config->Output_LPF_RC;
  pid->Derivative_LPF_RC = config->Derivative_LPF_RC;
  DWT_GetDeltaT(&pid->DWT_CNT);
  return pid;
}
/**
 * @brief          PID计算
 * @param[in]      PID结构体
 * @param[in]      测量值
 * @param[in]      期望值
 * @retval         返回空
 */
float PIDCalculate(PIDInstance *pid, float measure, float ref) {
  //! 调用方必须保证 pid 非空；该接口位于高频控制路径，保持旧版本无空指针分支。
  //? 堵转检测先使用上一轮输出和测量值判断故障状态，再参与本轮 Ref 处理。
  if (pid->Improve & PID_ErrorHandle)
    f_PID_ErrorHandle(pid);

  pid->dt = DWT_GetDeltaT(&pid->DWT_CNT);

  // 保存上次的测量值和误差,计算当前error
  pid->Measure = measure;
  pid->Ref = ref;
  pid->Err = pid->Ref - pid->Measure;

  //! 堵转故障会把 Ref 反向，适用于可反向释放的执行器，不适用于单向机构。
  if (pid->ERRORHandler.ERRORType == PID_MOTOR_BLOCKED_ERROR) // 堵转
    pid->Ref = -ref;

  //? 死区内跳过完整 PID 并清零输出，避免小误差附近反复抖动；死区外才执行完整链路。
  if (abs(pid->Err) > pid->DeadBand) {
    //? 位置式基础项：P 使用当前误差，I 使用误差对时间积分，D 使用误差差分。
    pid->Pout = pid->Kp * pid->Err;
    pid->ITerm = pid->Ki * pid->Err * pid->dt;
    pid->Dout = pid->Kd * (pid->Err - pid->Last_Err) / pid->dt;

    //* 按 Improve bit 顺序叠加可选优化环节。
    if (pid->Improve & PID_Trapezoid_Intergral)
      f_Trapezoid_Intergral(pid);
    if (pid->Improve & PID_ChangingIntegrationRate)
      f_Changing_Integration_Rate(pid);
    if (pid->Improve & PID_Derivative_On_Measurement)
      f_Derivative_On_Measurement(pid);
    if (pid->Improve & PID_DerivativeFilter)
      f_Derivative_Filter(pid);
    if (pid->Improve & PID_Integral_Limit)
      f_Integral_Limit(pid);

    pid->Iout += pid->ITerm;                         // 累加积分
    pid->Output = pid->Pout + pid->Iout + pid->Dout; // 计算输出

    // 输出滤波
    if (pid->Improve & PID_OutputFilter)
      f_Output_Filter(pid);

    // 输出限幅
    f_Output_Limit(pid);
  } else // 进入死区, 则清空积分和输出
  {
    //! 进入死区会清空 ITerm 和 Output，适合定位控制；若需保持力矩，需另行设计保持项。
    pid->Output = 0;
    pid->ITerm = 0;
  }

  //? 保存当前数据，形成下一轮微分、滤波和梯形积分所需的历史状态。
  pid->Last_Measure = pid->Measure;
  pid->Last_Output = pid->Output;
  pid->Last_Dout = pid->Dout;
  pid->Last_Err = pid->Err;
  pid->Last_ITerm = pid->ITerm;

  return pid->Output;
}
