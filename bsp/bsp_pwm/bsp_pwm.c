#include "bsp_pwm.h"

// ==================== PWM 基础控制 ====================

//! 初始化 PWM BSP 控制块。
//! PWM 对象按调用方传入的 TIM_HandleTypeDef* 和 TIM_CHANNEL_x 工作，不硬编码业务通道。
err_t STM32PWM_Init(STM32PWM_t *self,
                    TIM_HandleTypeDef *tim_handle,
                    uint32_t channel)
{
  if (self == NULL)
  {
    return PTR_NULL;
  }

  self->tim_handle_ = tim_handle;
  self->channel_ = channel;
  self->duty_cycle_ = 0.0f;
  self->started_ = false;
  self->last_error_ = PENDING;

  ASSERT(self->tim_handle_ != NULL);
  if (self->tim_handle_ == NULL)
  {
    self->last_error_ = PTR_NULL;
    return self->last_error_;
  }

  self->last_error_ = OK;
  return self->last_error_;
}

//! 启动 PWM 输出。
//! HAL 启动失败时保持 started_ 为 false，并把错误码记录为 FAILED。
err_t STM32PWM_Start(STM32PWM_t *self)
{
  if (self == NULL)
  {
    return PTR_NULL;
  }

  ASSERT(self->tim_handle_ != NULL);
  if (self->tim_handle_ == NULL)
  {
    self->last_error_ = PTR_NULL;
    return self->last_error_;
  }

  const HAL_StatusTypeDef status = HAL_TIM_PWM_Start(self->tim_handle_,
                                                     self->channel_);
  VERIFY(status == HAL_OK);
  if (status != HAL_OK)
  {
    self->started_ = false;
    self->last_error_ = FAILED;
    return self->last_error_;
  }

  self->started_ = true;
  self->last_error_ = OK;
  return self->last_error_;
}

//! 设置 PWM 占空比。
//! 先根据 ARR 计算比较值，再写入对应通道 CCR 寄存器。
err_t STM32PWM_SetDutyCycle(STM32PWM_t *self, float duty_cycle)
{
  if (self == NULL)
  {
    return PTR_NULL;
  }

  ASSERT(self->tim_handle_ != NULL);
  if (self->tim_handle_ == NULL)
  {
    self->last_error_ = PTR_NULL;
    return self->last_error_;
  }

  ASSERT((duty_cycle >= 0.0f) && (duty_cycle <= 1.0f));
  if ((duty_cycle < 0.0f) || (duty_cycle > 1.0f))
  {
    self->last_error_ = OUT_OF_RANGE;
    return self->last_error_;
  }

  const uint32_t period = __HAL_TIM_GET_AUTORELOAD(self->tim_handle_);
  const uint32_t pulse = (uint32_t)(duty_cycle * (float)period);

  __HAL_TIM_SET_COMPARE(self->tim_handle_, self->channel_, pulse);

  self->duty_cycle_ = duty_cycle;
  self->last_error_ = OK;
  return self->last_error_;
}

//! 停止 PWM 输出。
//! HAL 停止失败时不强行改 started_，由调用方根据错误码处理。
err_t STM32PWM_Stop(STM32PWM_t *self)
{
  if (self == NULL)
  {
    return PTR_NULL;
  }

  ASSERT(self->tim_handle_ != NULL);
  if (self->tim_handle_ == NULL)
  {
    self->last_error_ = PTR_NULL;
    return self->last_error_;
  }

  const HAL_StatusTypeDef status = HAL_TIM_PWM_Stop(self->tim_handle_,
                                                    self->channel_);
  VERIFY(status == HAL_OK);
  if (status != HAL_OK)
  {
    self->last_error_ = FAILED;
    return self->last_error_;
  }

  self->started_ = false;
  self->last_error_ = OK;
  return self->last_error_;
}

//! 获取 PWM 对象最近一次错误码。
err_t STM32PWM_GetLastError(const STM32PWM_t *self)
{
  if (self == NULL)
  {
    return PTR_NULL;
  }

  return self->last_error_;
}
