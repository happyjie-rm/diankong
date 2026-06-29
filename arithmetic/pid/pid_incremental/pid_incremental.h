/**
 ******************************************************************************
 * @file    jie.h
 * @brief   增量式PID控制器定义
 ******************************************************************************
 */

#ifndef __PID_INCREMENTAL_H
#define __PID_INCREMENTAL_H

// #include "arm_math.h"
#include "stdint.h"
#include "stdlib.h"
#include <math.h>
#include <string.h>

#ifndef abs
#define abs(x) ((x > 0) ? x : -x)
#endif

//* ================= 增量式 PID 改进功能开关 =================
//? Improve 使用按位或组合，PID_Calculate_jie() 通过位与选择启用的增量修正环节。
//! 该枚举和位置式 PID 同名项较多，跨模块引用时注意使用 PID_Improvement_jie_e 类型。
typedef enum {
  PID_IMPROVE_NONE = 0b00000000,
  /* 0000 0000 */ // 无改进
  PID_Integral_Limit = 0b00000001,
  /* 0000 0001 */ // 积分限幅
  PID_Derivative_On_Measurement = 0b00000010,
  /* 0000 0010 */ // 微分先行
  PID_Trapezoid_Intergral = 0b00000100,
  /* 0000 0100 */ // 梯形积分
  PID_FeedForward = 0b00001000,
  /* 0000 1000 */ // 前馈控制
  PID_OutputFilter = 0b00010000,
  /* 0001 0000 */ // 输出滤波
  PID_ChangingIntegrationRate = 0b00100000,
  /* 0010 0000 */ // 变速积分
  PID_DerivativeFilter = 0b01000000,
  /* 0100 0000 */ // 微分项低通滤波
} PID_Improvement_jie_e;

//* ================= 增量式 PID 运行实例 =================
//? PIDInstance_jie 同时整合配置、误差历史、输出增量和最终输出。
//! 控制循环运行时不要手动改写 Err_Pre、Last_Err、Last_Output 等历史字段。
typedef struct {
  //* ==================== 基础 PID 参数 ====================
  float Kp;       /* 比例系数，控制误差响应速度 */
  float Ki;       /* 积分系数，用于消除稳态误差 */
  float Kd;       /* 微分系数，用于抑制超调和改善动态响应 */
  float MaxOut;   /* PID输出限幅值，超过此值将被限制 */
  float DeadBand; /* 死区范围，误差在此范围内时不进行控制 */
  float dt;       /* 采样周期，单位：秒 */
  uint32_t DWT_CNT;
#ifdef USE_TIME
  //! USE_TIME 在头文件包含前定义时会额外启用 DWT 计数器字段，注意不要重复定义。
  uint32_t DWT_CNT; /* DWT计数器，用于高精度时间测量 */
#endif

  //* ==================== 改进功能使能标志 ====================
  uint32_t Improve; /* 各项改进功能的使能位组合，按位或操作 */

  //* ==================== 积分改进参数 ====================
  float IntegralLimit; /* 积分限幅值，防止积分饱和 */
  float CoefA;         /* 变速积分上限系数，误差较大时减小积分作用 */
  float CoefB;         /* 变速积分下限系数，误差较小时保持积分作用 */

  //* ==================== 滤波器参数 ====================
  float Derivative_LPF_RC; /* 微分项低通滤波器时间常数，用于平滑微分项 */
  float Output_LPF_RC;     /* 输出低通滤波器时间常数，RC=1/ωc，用于平滑输出 */

  //* ==================== 前馈控制参数 ====================
  float Kf;              /* 前馈系数，用于根据参考值变化提前补偿 */
  float FeedForward_Max; /* 前馈输出限幅值 */

  //* ==================== 输入输出缓存 ====================
  float actual;      /* 实际值（反馈值） */
  float Last_actual; /* 上一次实际值 */
  float target;      /* 目标值（设定值） */
  float Last_target; /* 上一次目标值 */

  //* ==================== 误差计算缓存 ====================
  float Err;      /* 当前误差 e(k) = target - actual */
  float Last_Err; /* 上一次误差 e(k-1) */
  float Err_Pre;  /* 上上一次误差 e(k-2) */

  //* ==================== PID 各项输出 ====================
  float Pout; /* 比例项输出/增量 */
  float Iout; /* 积分项累计值 */
  float Dout; /* 微分项输出/增量 */

  //* ==================== 最终输出缓存 ====================
  //? 增量式输出先计算 Output_Inc，再累加到 Last_Output 得到 Output。
  float Output;      /* 位置式PID总输出（绝对值） */
  float Output_Inc;  /* 增量式PID输出增量 */
  float Last_Output; /* 上一次输出值 */
  float Last_Dout;   /* 上一次微分输出（用于不完全微分滤波） */
  float Output_Inc_max;
} PIDInstance_jie;

/**
 * @brief PID快速初始化函数（参数化初始化）
 * @param pid            PID实例指针
 * @param kp             比例系数
 * @param ki             积分系数
 * @param kd             微分系数
 * @param dt             采样周期（秒）
 * @param max_output     输出限幅值
 * @param max_integral   积分限幅值
 * @param deadzone       死区范围
 * @param improve_flags  优化功能使能标志位（按位或组合）
 */
void PID_Init_Params_jie(PIDInstance_jie *pid, float kp, float ki, float kd,
                         float dt, float max_output, float max_integral,
                         float deadzone, PID_Improvement_jie_e improve_flags);

/**
 * @brief 计算PID输出
 * @param pid     PID实例指针
 * @param target 设定值
 * @param actual 反馈值
 * @return float  PID计算输出
 */
float PID_Calculate_jie(PIDInstance_jie *pid, float target, float actual);

/**
 * @brief 获取输出增量
 * @param pid     PID实例指针
 * @return float  PID输出增量
 */
float PID_Get_Increment_jie(PIDInstance_jie *pid);

/**
 * @brief PID复位，清零所有运行时状态
 * @param pid PID实例指针
 */
void PID_Reset_jie(PIDInstance_jie *pid);

#endif /* __JIE_H */
