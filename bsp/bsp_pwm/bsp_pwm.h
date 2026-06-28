#ifndef __BSP_PWM_H__
#define __BSP_PWM_H__

#include "main.h"
#include "comp_cmd.h"
#include "comp_utils.h"

// ==================== PWM 类型定义 ====================

//! PWM BSP 控制块。
//! BSP 只保存 HAL 定时器句柄和通道号，不拥有 HAL 句柄生命周期。
typedef struct
{
  TIM_HandleTypeDef *tim_handle_;  //! CubeMX 生成的 HAL TIM 句柄
  uint32_t channel_;               //! HAL TIM_CHANNEL_x 输出通道
  float duty_cycle_;               //! 最近设置的占空比，范围 [0.0, 1.0]
  bool started_;                   //! PWM 输出是否已成功启动
  err_t last_error_;               //! 最近一次 BSP 操作结果
} STM32PWM_t;

#ifdef __cplusplus
extern "C" {
#endif

// ==================== PWM 公开接口 ====================

//! 初始化 PWM BSP 控制块。
//! 只绑定定时器句柄和 PWM 通道；真正启动 PWM 输出由 STM32PWM_Start() 完成。
err_t STM32PWM_Init(STM32PWM_t *self,
                    TIM_HandleTypeDef *tim_handle,
                    uint32_t channel);

//! 启动 PWM 输出。
err_t STM32PWM_Start(STM32PWM_t *self);

//! 设置 PWM 占空比。
//! duty_cycle 约定范围为 [0.0, 1.0]，0 只清零比较值，不隐式停止 PWM。
err_t STM32PWM_SetDutyCycle(STM32PWM_t *self, float duty_cycle);

//! 停止 PWM 输出。
err_t STM32PWM_Stop(STM32PWM_t *self);

//! 读取最近一次错误码。
err_t STM32PWM_GetLastError(const STM32PWM_t *self);

#ifdef __cplusplus
}
#endif

#endif // __BSP_PWM_H__
