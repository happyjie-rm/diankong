#ifndef __BSP_IIC_H__
#define __BSP_IIC_H__

#include "main.h"
#include "stm32f4xx_hal_i2c.h"
#include "comp_cmd.h"
#include "comp_utils.h"

// ==================== IIC 类型与回调定义 ====================

//! IIC BSP 逻辑设备编号。
//! 枚举项按当前芯片支持的 I2C 外设宏展开，不绑定具体 CubeMX 生成的 hi2c 全局句柄。
typedef enum
{
#ifdef I2C1
  BSP_IIC1,
#endif
#ifdef I2C2
  BSP_IIC2,
#endif
#ifdef I2C3
  BSP_IIC3,
#endif

  BSP_IIC_NUMBER,
  BSP_IIC_ID_ERROR
} BSP_IIC_t;

//! IIC 事件回调类型。
//! 类型覆盖 HAL I2C 的主/从收发完成、内存读写完成、监听完成、错误和终止完成事件。
typedef enum
{
  BSP_IIC_MASTER_TX_CPLT_CB,
  BSP_IIC_MASTER_RX_CPLT_CB,
  BSP_IIC_SLAVE_TX_CPLT_CB,
  BSP_IIC_SLAVE_RX_CPLT_CB,
  BSP_IIC_LISTEN_CPLT_CB,
  BSP_IIC_MEM_TX_CPLT_CB,
  BSP_IIC_MEM_RX_CPLT_CB,
  BSP_IIC_ERROR_CB,
  BSP_IIC_ABORT_CPLT_CB,

  BSP_IIC_CB_NUMBER
} BSP_IIC_Callback_t;

typedef struct STM32IIC STM32IIC_t;

//! 用户 IIC 事件回调。
//! 回调由 HAL I2C 事件分发路径触发，self 指向已注册的 IIC BSP 控制块。
typedef void (*STM32IIC_Callback_t)(STM32IIC_t *self);

//! IIC BSP 控制块。
//! BSP 只保存 HAL 句柄和事件回调，不拥有 HAL 句柄生命周期。
struct STM32IIC
{
  BSP_IIC_t id_;                                      //! BSP 逻辑 IIC 编号
  I2C_HandleTypeDef *iic_handle_;                     //! CubeMX 生成的 HAL I2C 句柄
  STM32IIC_Callback_t callbacks_[BSP_IIC_CB_NUMBER];  //! 各 HAL I2C 事件对应的用户回调
  err_t last_error_;                                  //! 最近一次 BSP 操作结果
};

#ifdef __cplusplus
extern "C" {
#endif

// ==================== IIC 公开接口 ====================

//! 将 HAL 外设 Instance 映射到 BSP IIC 逻辑 ID。
BSP_IIC_t BSP_IIC_get_id(I2C_TypeDef *addr);

//! 初始化 IIC BSP 控制块。
err_t STM32IIC_Init(STM32IIC_t *self, I2C_HandleTypeDef *iic_handle);

//! 获取 IIC BSP 控制块绑定的 HAL 句柄。
I2C_HandleTypeDef *STM32IIC_GetHandle(const STM32IIC_t *self);

//! 注册 IIC 指定事件类型的用户回调。
err_t STM32IIC_RegisterCallback(STM32IIC_t *self,
                                BSP_IIC_Callback_t type,
                                STM32IIC_Callback_t callback);

//! 更新 IIC 指定事件类型的用户回调，可传入 NULL 清除回调。
void STM32IIC_SetCallback(STM32IIC_t *self,
                          BSP_IIC_Callback_t type,
                          STM32IIC_Callback_t callback);

//! 读取最近一次错误码。
err_t STM32IIC_GetLastError(const STM32IIC_t *self);

#ifdef __cplusplus
}
#endif

#endif // __BSP_IIC_H__
