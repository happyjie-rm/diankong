/**
 ******************************************************************************
 * @file	 controller.h
 * @author  Wang Hongxi
 * @version V1.1.3
 * @date    2021/7/3
 * @brief
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
#ifndef __PID_LOCATION_H
#define __PID_LOCATION_H

#include "main.h"
#include "memory.h"
#include "stdint.h"
#include "stdlib.h"
// #include "user_lib.h"
#include <math.h>

#ifndef abs
#define abs(x) ((x > 0) ? x : -x)
#endif

//* ================= 位置式 PID 改进功能开关 =================
//? Improve 使用按位或组合，计算路径通过位与判断需要启用哪些修正环节。
//! 修改枚举值会影响已有 PID_Init_Config_s 配置，新增功能时只能追加新 bit。
typedef enum {
  PID_IMPROVE_NONE = 0b00000000,                // 0000 0000
  PID_Integral_Limit = 0b00000001,              // 0000 0001
  PID_Derivative_On_Measurement = 0b00000010,   // 0000 0010
  PID_Trapezoid_Intergral = 0b00000100,         // 0000 0100
  PID_Proportional_On_Measurement = 0b00001000, // 0000 1000
  PID_OutputFilter = 0b00010000,                // 0001 0000
  PID_ChangingIntegrationRate = 0b00100000,     // 0010 0000
  PID_DerivativeFilter = 0b01000000,            // 0100 0000
  PID_ErrorHandle = 0b10000000,                 // 1000 0000
} PID_Improvement_e;

//* ================= 位置式 PID 错误状态 =================
//? 当前错误处理只实现电机堵转检测，ERRORType 按 bit 保存，便于后续扩展多个故障源。
typedef enum errorType_e {
  PID_ERROR_NONE = 0x00U,
  PID_MOTOR_BLOCKED_ERROR = 0x01U
} ErrorType_e;

typedef struct {
  uint64_t ERRORCount;
  ErrorType_e ERRORType;
} PID_ErrorHandler_t;

//* ================= 位置式 PID 运行实例 =================
//? PIDInstance 同时保存配置参数和运行时状态，PIDCalculate() 每轮都会更新缓存字段。
//! 外部不要在控制循环中直接改写 Last_*、Iout、Dout 等运行时字段，否则会破坏状态连续性。
typedef struct {
  //---------------------------------- init config block
  //? config parameter: 这些字段来自 PID_Init_Config_s，初始化后作为调参入口。
  float Kp;
  float Ki;
  float Kd;
  float MaxOut;
  float DeadBand;

  //? improve parameter: 只有 Improve 中对应 bit 置位时，这些参数才会参与计算。
  PID_Improvement_e Improve;
  float IntegralLimit; // 积分限幅
  float CoefA;         // 变速积分 For Changing Integral
  float CoefB; // 变速积分 ITerm = Err*((A-abs(err)+B)/A)  when B<|err|<A+B
  float Output_LPF_RC;     // 输出滤波器 RC = 1/omegac
  float Derivative_LPF_RC; // 微分滤波器系数

  //-----------------------------------
  //? for calculating: 以下字段由 PIDCalculate() 维护，用于保存当前值和上一次状态。
  float Measure;
  float Last_Measure;
  float Err;
  float Last_Err;
  float Last_ITerm;

  float Pout;
  float Iout;
  float Dout;
  float ITerm;

  float Output;
  float Last_Output;
  float Last_Dout;

  float Ref;

  uint32_t DWT_CNT;
  float dt;

  PID_ErrorHandler_t ERRORHandler;
} PIDInstance;

//* ================= 位置式 PID 初始化配置 =================
//? 配置结构只描述固定参数，不保存运行时误差、输出和滤波历史。
typedef struct // config parameter
{
  //? basic parameter: 控制器基础三参数、输出限幅和死区。
  float Kp;
  float Ki;
  float Kd;
  float MaxOut;   // 输出限幅
  float DeadBand; // 死区

  //? improve parameter: 与 Improve bit 对应的可选优化参数。
  PID_Improvement_e Improve;
  float IntegralLimit; // 积分限幅
  float CoefA;         // AB为变速积分参数,变速积分实际上就引入了积分分离
  float CoefB;         // ITerm = Err*((A-abs(err)+B)/A)  when B<|err|<A+B
  float Output_LPF_RC; // RC = 1/omegac
  float Derivative_LPF_RC;
} PID_Init_Config_s;

/**
 * @brief 初始化PID实例
 * @attention 该函数为旧版本，请尽快使用PIDRegister函数
 * @param pid    PID实例指针
 * @param config PID初始化配置
 */
void PIDInit(PIDInstance *pid, PID_Init_Config_s *config);

PIDInstance *PIDRegister(PID_Init_Config_s *config);
/**
 * @brief 计算PID输出
 *
 * @param pid     PID实例指针
 * @param measure 反馈值
 * @param ref     设定值
 * @return float  PID计算输出
 */
float PIDCalculate(PIDInstance *pid, float measure, float ref);

#endif
