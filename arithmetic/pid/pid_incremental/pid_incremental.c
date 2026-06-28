/**
 * @file jie.c
 * @brief 增量式PID控制器实现
 *
 * 增量式PID公式：
 * Δu(k) = Kp[err(k) - err(k-1)] + Ki*err(k) + Kd[err(k) - 2*err(k-1) +
 * err(k-2)] u(k) = u(k-1) + Δu(k)
 */

#include "pid_incremental.h"
#include "bsp_dwt.h"
#define USE_TIME
//* ================= 增量式 PID 优化环节实现 =================
//? 以下函数只处理单个优化项，PID_Calculate_jie() 负责组合成最终增量输出。

/**
 * @brief 梯形积分（增量式）
 * 使用梯形公式计算积分项，提高积分精度（在采样频率低时有效）（在系统很稳定，但最后一点点误差消除得非常慢时可以试试)
 * ΔI = Ki * (ek + ek-1) / 2 * dt
 */
static void f_Trapezoid_Intergral_jie(PIDInstance_jie *pid) {
  //? 用相邻两次误差的平均值计算积分增量，比矩形积分更平滑。
#ifdef USE_TIME
  pid->Iout = pid->Ki * ((pid->Err + pid->Last_Err) * 0.5f) * pid->dt;
#else
  pid->Iout = pid->Ki * ((pid->Err + pid->Last_Err) * 0.5f);
#endif
}

/**
 * @brief 变速积分（增量式）
 * 误差小时积分作用强，误差大时减弱积分作用，防止积分饱和
 * 通过对积分增量乘以变系数实现（在误差小时积分作用强，误差大时减弱积分作用）（防止系统一启动就冲过头）
 */
static void f_Changing_Integration_Rate_jie(PIDInstance_jie *pid) {
  //! CoefA 必须大于 0，否则线性削弱区间会出现除零风险。
  float abs_err = fabsf(pid->Err);

  //? 误差小于 CoefB：按当前实现清零 Iout，等价于在极小误差区暂停积分增量。
  if (abs_err <= pid->CoefB) {
    pid->Iout = 0;
    /* Iout保持原值，无需修改 */
  }

  //? 误差在 [CoefB, CoefA+CoefB] 之间：线性递减积分作用。
  if (abs_err <= (pid->CoefA + pid->CoefB)) {
    float factor = (pid->CoefA - abs_err + pid->CoefB) / pid->CoefA;
    pid->Iout *= factor;
  }
  //? 误差大于 CoefA+CoefB：不使用积分，降低大偏差阶段的积分饱和。
  else {
    pid->Iout = 0;
  }
}

/**
 * @brief 积分限幅（增量式）
 * 防止积分项过大导致积分饱和
 */
static void f_Integral_Limit_jie(PIDInstance_jie *pid) {
  //! 积分限幅只限制本轮 Iout 增量，不会回退已经累加到 Output 的历史输出。
  if (pid->Iout > pid->IntegralLimit) {
    pid->Iout = pid->IntegralLimit;
  } else if (pid->Iout < -pid->IntegralLimit) {
    pid->Iout = -pid->IntegralLimit;
  }
}

/**
 * @brief 微分先行（增量式）
 * 仅对测量值进行微分，避免设定值突变带来的冲击
 * ΔD = -Kd * (actual - last_actual) / dt
 */
static void f_Derivative_On_actualment_jie(PIDInstance_jie *pid) {
  //? 微分先行用测量值变化计算 Dout，避免 target 阶跃导致微分冲击。
#ifdef USE_TIME
  pid->Dout = pid->Kd * (pid->Last_actual - pid->actual) / pid->dt;
#else
  pid->Dout = pid->Kd * (pid->Last_actual - pid->actual);
#endif
}

/**
 * @brief 微分滤波（不完全微分）
 * 使用一阶低通滤波器平滑微分项，减少噪声影响
 * Dout(k) = alpha * raw_dout + (1-alpha) * Dout(k-1)
 * alpha = dt / (RC + dt)
 */
static void f_Derivative_Filter_jie(PIDInstance_jie *pid, float raw_dout) {
  //! Derivative_LPF_RC 越大滤波越重，响应也越慢；调参时需结合噪声和相位滞后一起看。
  float rc = pid->Derivative_LPF_RC;
  float factor;

#ifdef USE_TIME
  factor = pid->dt / (rc + pid->dt);
#else
  factor = 1.0f / (rc + 1.0f);
#endif

  pid->Dout = raw_dout * factor + pid->Last_Dout * (1.0f - factor);
  pid->Last_Dout = pid->Dout;
}

/**
 * @brief 输出滤波
 * 使用一阶低通滤波器平滑输出，减少输出抖动
 * Output(k) = alpha * raw_output + (1-alpha) * Output(k-1)
 */
static void f_Output_Filter_jie(PIDInstance_jie *pid, float raw_output) {
  //? 输出滤波作用在累加后的总输出上，适合抑制电机电流或舵机指令抖动。
  float rc = pid->Output_LPF_RC;
  float factor;

#ifdef USE_TIME
  factor = pid->dt / (rc + pid->dt);
#else
  factor = 1.0f / (rc + 1.0f);
#endif

  pid->Output = raw_output * factor + pid->Last_Output * (1.0f - factor);
}

/**
 * @brief 输出限幅
 * 将PID输出限制在[-MaxOut, MaxOut]范围内
 */
static void f_Output_Limit_jie(PIDInstance_jie *pid) {
  //! 输出限幅必须在滤波后执行，保证最终写给执行器的值不会越界。
  if (pid->Output > pid->MaxOut) {
    pid->Output = pid->MaxOut;
  } else if (pid->Output < -pid->MaxOut) {
    pid->Output = -pid->MaxOut;
  }
}

/**
 * @brief 前馈控制（增量式）
 * 基于设定值变化率的前馈，提前响应目标变化
 * ΔFF = Kf * (target - last_target) / dt
 */
static float f_FeedForward_jie(PIDInstance_jie *pid) {
  //? 前馈只跟 target 变化率有关，适合目标快速变化时提前补偿控制输出。
  float delta_target = pid->target - pid->Last_target;
  float delta_ff;

#ifdef USE_TIME
  delta_ff = pid->Kf * delta_target / pid->dt;
#else
  delta_ff = pid->Kf * delta_target;
#endif

  /* 前馈限幅 */
  if (delta_ff > pid->FeedForward_Max) {
    delta_ff = pid->FeedForward_Max;
  } else if (delta_ff < -pid->FeedForward_Max) {
    delta_ff = -pid->FeedForward_Max;
  }

  return delta_ff;
}

//* ================= 增量式 PID 外部算法接口 =================

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
                         float deadzone, PID_Improvement_jie_e improve_flags) {
  //! 该初始化接口只做参数写入和运行时清零，调用方负责保证 dt/max_output 等参数有效。
  if (pid == NULL)
    return;

  /* 清零结构体 */
  memset(pid, 0, sizeof(PIDInstance_jie));

  /* 设置基础PID参数 */
  pid->Kp = kp;
  pid->Ki = ki;
  pid->Kd = kd;
  pid->dt = dt;
  pid->MaxOut = max_output;
  pid->IntegralLimit = max_integral;
  pid->DeadBand = deadzone;

  /* 设置优化功能使能标志 */
  pid->Improve = improve_flags;

  /* 滤波器默认参数 */
  pid->Derivative_LPF_RC = 0.1f; /* 默认微分滤波时间常数 */
  pid->Output_LPF_RC = 0.05f;    /* 默认输出滤波时间常数 */

  /* 变速积分默认参数 */
  pid->CoefA = max_output * 0.5f; /* 变速积分上限系数 */
  pid->CoefB = max_output * 0.1f; /* 变速积分下限系数 */

  /* 前馈控制默认参数 */
  pid->Kf = 0.0f;                           /* 默认前馈系数为0（禁用） */
  pid->FeedForward_Max = max_output * 0.5f; /* 默认前馈限幅为最大输出的50% */

#ifdef USE_TIME
  DWT_GetDeltaT(&pid->DWT_CNT);
#endif
}

/**
 * @brief 增量式PID计算
 * @param pid     PID结构体指针
 * @param actual 测量值
 * @param target     设定值
 * @return float 返回PID总输出（位置式输出）
 */
float PID_Calculate_jie(PIDInstance_jie *pid, float target, float actual) {
  //! 调用方必须保证 pid 非空；本函数保持高频控制路径的旧接口开销。
#ifdef USE_TIME
  /* 获取两次pid计算的时间间隔,用于积分和微分 */
  pid->dt = DWT_GetDeltaT(&pid->DWT_CNT);
#endif

  //? 保存上一次测量值和设定值，供微分先行和前馈计算使用。
  pid->Last_actual = pid->actual;
  pid->Last_target = pid->target;

  /* 更新当前值 */
  pid->actual = actual;
  pid->target = target;

  /* 保存误差历史：Err_Pre <- Last_Err <- Err <- 新计算值 */
  pid->Err_Pre = pid->Last_Err;
  pid->Last_Err = pid->Err;
  pid->Err = pid->target - pid->actual;

  //! 死区内只清空本轮增量和积分项，保留 Output 作为执行器保持输出。
  if (fabsf(pid->Err) <= pid->DeadBand) {
    pid->Output_Inc = 0;
    pid->Iout = 0;
    return pid->Output;
  }

  /* ========== 增量式PID各项计算 ========== */

  //* ---------------- 比例项计算 ----------------
  //? ΔP = Kp * (ek - ek-1)，只响应误差变化量。
  pid->Pout = pid->Kp * (pid->Err - pid->Last_Err);

  //* ---------------- 积分项计算 ----------------
  //? 基础积分：ΔI = Ki * ek * dt。
#ifdef USE_TIME
  pid->Iout = pid->Ki * pid->Err * pid->dt;
#else
  pid->Iout = pid->Ki * pid->Err;
#endif

  /* 梯形积分：使用梯形公式代替矩形公式 */
  if (pid->Improve & PID_Trapezoid_Intergral) {
    f_Trapezoid_Intergral_jie(pid);
  }

  /* 变速积分：根据误差大小调整积分作用 */
  if (pid->Improve & PID_ChangingIntegrationRate) {
    f_Changing_Integration_Rate_jie(pid);
  }

  /* 积分限幅：防止积分饱和 */
  if (pid->Improve & PID_Integral_Limit) {
    f_Integral_Limit_jie(pid);
  }

  //* ---------------- 微分项计算 ----------------
  float raw_dout = 0.0f;

  if (pid->Improve & PID_Derivative_On_Measurement) {
    /* 微分先行：仅对测量值微分，避免设定值突变冲击 */
    f_Derivative_On_actualment_jie(pid);
  } else {
    /* 标准微分：ΔD = Kd * (ek - 2*ek-1 + ek-2) */
#ifdef USE_TIME
    raw_dout =
        pid->Kd * (pid->Err - 2.0f * pid->Last_Err + pid->Err_Pre) / pid->dt;
#else
    raw_dout = pid->Kd * (pid->Err - 2.0f * pid->Last_Err + pid->Err_Pre);
#endif

    /* 微分滤波：不完全微分，使用低通滤波器平滑 */
    if (pid->Improve & PID_DerivativeFilter) {
      f_Derivative_Filter_jie(pid, raw_dout);
    } else {
      pid->Dout = raw_dout;
    }
  }

  //* ---------------- 前馈控制 ----------------
  float feedforward = 0.0f;
  if (pid->Improve & PID_FeedForward) {
    feedforward = f_FeedForward_jie(pid);
  }

  //* ---------------- 输出计算 ----------------
  //? 计算总增量：Δu = ΔP + ΔI + ΔD + ΔFF。
  pid->Output_Inc = pid->Pout + pid->Iout + pid->Dout + feedforward;

  //? 计算总输出：u(k) = u(k-1) + Δu(k)。
  float raw_output = pid->Last_Output + pid->Output_Inc;

  /* 输出滤波：使用低通滤波器平滑输出 */
  if (pid->Improve & PID_OutputFilter) {
    f_Output_Filter_jie(pid, raw_output);
  } else {
    pid->Output = raw_output;
  }

  /* 输出限幅 */
  f_Output_Limit_jie(pid);

  /* 保存当前输出状态 */
  pid->Last_Output = pid->Output;

  return pid->Output;
}

/**
 * @brief 获取输出增量
 * @param pid PID实例指针
 * @return float PID输出增量
 */
float PID_Get_Increment_jie(PIDInstance_jie *pid) {
  //? 该接口适合调试或外层限幅观察，只读取最近一次 PID_Calculate_jie() 的增量。
  if (pid == NULL)
    return 0.0f;
  return pid->Output_Inc;
}

/**
 * @brief PID复位，清零所有运行时状态（保留配置参数）
 * @param pid PID实例指针
 */
void PID_Reset_jie(PIDInstance_jie *pid) {
  //! Reset 只清运行时状态，不恢复 Kp/Ki/Kd、限幅、滤波和 Improve 配置。
  if (pid == NULL)
    return;

  /* 清零运行时状态（保留配置参数） */
  pid->actual = 0.0f;
  pid->Last_actual = 0.0f;
  pid->target = 0.0f;
  pid->Last_target = 0.0f;

  pid->Err = 0.0f;
  pid->Last_Err = 0.0f;
  pid->Err_Pre = 0.0f;

  pid->Pout = 0.0f;
  pid->Iout = 0.0f;
  pid->Dout = 0.0f;

  pid->Output = 0.0f;
  pid->Output_Inc = 0.0f;
  pid->Last_Output = 0.0f;
  pid->Last_Dout = 0.0f;

#ifdef USE_TIME
  DWT_GetDeltaT(&pid->DWT_CNT);
#endif
}
