#include "bsp_iic.h"

#include <string.h>

// ==================== IIC 对象表 ====================
//! HAL 回调只提供 I2C_HandleTypeDef，BSP 通过 Instance 反查对象式控制块。

static STM32IIC_t *stm32_iic_map[BSP_IIC_NUMBER] = {0};

//! 将 HAL 外设 Instance 转换为 BSP IIC 逻辑 ID。
BSP_IIC_t BSP_IIC_get_id(I2C_TypeDef *addr)
{
  if (addr == NULL)
  {
    return BSP_IIC_ID_ERROR;
  }

#ifdef I2C1
  if (addr == I2C1)
  {
    return BSP_IIC1;
  }
#endif
#ifdef I2C2
  if (addr == I2C2)
  {
    return BSP_IIC2;
  }
#endif
#ifdef I2C3
  if (addr == I2C3)
  {
    return BSP_IIC3;
  }
#endif

  return BSP_IIC_ID_ERROR;
}

//! 检查 BSP IIC 逻辑 ID 是否可用。
static bool BSP_IIC_is_valid_id(BSP_IIC_t id)
{
  return (id != BSP_IIC_ID_ERROR) && (id < BSP_IIC_NUMBER);
}

//! 检查 BSP IIC 回调类型是否可用。
static bool BSP_IIC_is_valid_callback(BSP_IIC_Callback_t type)
{
  return type < BSP_IIC_CB_NUMBER;
}

//! 根据 HAL I2C 句柄查找已注册的 IIC 控制块。
static STM32IIC_t *BSP_IIC_get_object(I2C_HandleTypeDef *hi2c)
{
  if ((hi2c == NULL) || (hi2c->Instance == NULL))
  {
    return NULL;
  }

  const BSP_IIC_t id = BSP_IIC_get_id(hi2c->Instance);
  if (!BSP_IIC_is_valid_id(id))
  {
    return NULL;
  }

  return stm32_iic_map[id];
}

//! 分发 HAL I2C 事件到用户注册的对象式回调。
static void BSP_IIC_dispatch_callback(I2C_HandleTypeDef *hi2c,
                                      BSP_IIC_Callback_t type)
{
  STM32IIC_t *iic = BSP_IIC_get_object(hi2c);
  if (iic == NULL)
  {
    return;
  }

  if (!BSP_IIC_is_valid_callback(type))
  {
    iic->last_error_ = OUT_OF_RANGE;
    return;
  }

  STM32IIC_Callback_t callback = iic->callbacks_[type];
  if (callback == NULL)
  {
    return;
  }

  iic->last_error_ = (type == BSP_IIC_ERROR_CB) ? FAILED : OK;
  callback(iic);
}

// ==================== 对象式 IIC 接口 ====================

//! 初始化 IIC BSP 控制块。
err_t STM32IIC_Init(STM32IIC_t *self, I2C_HandleTypeDef *iic_handle)
{
  if (self == NULL)
  {
    return PTR_NULL;
  }

  self->id_ = (iic_handle != NULL) ? BSP_IIC_get_id(iic_handle->Instance)
                                   : BSP_IIC_ID_ERROR;
  self->iic_handle_ = iic_handle;
  self->last_error_ = PENDING;
  memset(self->callbacks_, 0, sizeof(self->callbacks_));

  ASSERT(self->iic_handle_ != NULL);
  if (self->iic_handle_ == NULL)
  {
    self->last_error_ = PTR_NULL;
    return self->last_error_;
  }

  ASSERT(BSP_IIC_is_valid_id(self->id_));
  if (!BSP_IIC_is_valid_id(self->id_))
  {
    self->last_error_ = NOT_FOUND;
    return self->last_error_;
  }

  ASSERT((stm32_iic_map[self->id_] == NULL) ||
         (stm32_iic_map[self->id_] == self));
  if ((stm32_iic_map[self->id_] != NULL) &&
      (stm32_iic_map[self->id_] != self))
  {
    self->last_error_ = BUSY;
    return self->last_error_;
  }

  stm32_iic_map[self->id_] = self;
  self->last_error_ = OK;
  return self->last_error_;
}

//! 获取 IIC BSP 控制块绑定的 HAL 句柄。
I2C_HandleTypeDef *STM32IIC_GetHandle(const STM32IIC_t *self)
{
  if (self == NULL)
  {
    return NULL;
  }

  return self->iic_handle_;
}

//! 注册 IIC 指定事件类型的用户回调。
err_t STM32IIC_RegisterCallback(STM32IIC_t *self,
                                BSP_IIC_Callback_t type,
                                STM32IIC_Callback_t callback)
{
  if (self == NULL)
  {
    return PTR_NULL;
  }

  ASSERT(BSP_IIC_is_valid_callback(type));
  if (!BSP_IIC_is_valid_callback(type))
  {
    self->last_error_ = OUT_OF_RANGE;
    return self->last_error_;
  }

  ASSERT(callback != NULL);
  if (callback == NULL)
  {
    self->last_error_ = PTR_NULL;
    return self->last_error_;
  }

  self->callbacks_[type] = callback;
  self->last_error_ = OK;
  return self->last_error_;
}

//! 更新 IIC 指定事件类型的用户回调，可传入 NULL 清除回调。
void STM32IIC_SetCallback(STM32IIC_t *self,
                          BSP_IIC_Callback_t type,
                          STM32IIC_Callback_t callback)
{
  if (self == NULL)
  {
    return;
  }

  if (!BSP_IIC_is_valid_callback(type))
  {
    self->last_error_ = OUT_OF_RANGE;
    return;
  }

  self->callbacks_[type] = callback;
  self->last_error_ = OK;
}

//! 获取 IIC 对象最近一次错误码。
err_t STM32IIC_GetLastError(const STM32IIC_t *self)
{
  if (self == NULL)
  {
    return PTR_NULL;
  }

  return self->last_error_;
}

// ==================== HAL 回调分发 ====================

//! I2C master 发送完成回调。
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  BSP_IIC_dispatch_callback(hi2c, BSP_IIC_MASTER_TX_CPLT_CB);
}

//! I2C master 接收完成回调。
void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  BSP_IIC_dispatch_callback(hi2c, BSP_IIC_MASTER_RX_CPLT_CB);
}

//! I2C slave 发送完成回调。
void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  BSP_IIC_dispatch_callback(hi2c, BSP_IIC_SLAVE_TX_CPLT_CB);
}

//! I2C slave 接收完成回调。
void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  BSP_IIC_dispatch_callback(hi2c, BSP_IIC_SLAVE_RX_CPLT_CB);
}

//! I2C listen 完成回调。
void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c)
{
  BSP_IIC_dispatch_callback(hi2c, BSP_IIC_LISTEN_CPLT_CB);
}

//! I2C memory 发送完成回调。
void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  BSP_IIC_dispatch_callback(hi2c, BSP_IIC_MEM_TX_CPLT_CB);
}

//! I2C memory 接收完成回调。
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  BSP_IIC_dispatch_callback(hi2c, BSP_IIC_MEM_RX_CPLT_CB);
}

//! I2C 错误回调。
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
  BSP_IIC_dispatch_callback(hi2c, BSP_IIC_ERROR_CB);
}

//! I2C abort 完成回调。
void HAL_I2C_AbortCpltCallback(I2C_HandleTypeDef *hi2c)
{
  BSP_IIC_dispatch_callback(hi2c, BSP_IIC_ABORT_CPLT_CB);
}
