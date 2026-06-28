#ifndef __BSP_USB_H__
#define __BSP_USB_H__

#include "main.h"
#include "comp_cmd.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"

// ==================== USB CDC 类型与回调定义 ====================

//! USB BSP 逻辑设备编号。
//! 当前工程只启用 USB_OTG_FS CDC 设备，因此 BSP 只映射 FS 逻辑设备。
typedef enum
{
  BSP_USB_FS,

  BSP_USB_NUMBER,
  BSP_USB_ID_ERROR
} BSP_USB_t;

//! 用户接收回调。
//! data 指向本次 CDC OUT 收到的数据片段，size 为片段长度；回调由 USB CDC 接收路径触发。
typedef void (*STM32USB_RxCallback_t)(uint8_t *data, size_t size);

//! 用户发送完成回调。
//! CDC IN 传输完成后触发，用于通知上层发送窗口已经释放。
typedef void (*STM32USB_TxCompleteCallback_t)(void);

//! USB CDC BSP 控制块。
//! BSP 只保存 USB Device 句柄和回调，不拥有 HAL/USBD 句柄生命周期。
typedef struct
{
  BSP_USB_t id_;                                //! BSP 逻辑 USB 设备编号
  USBD_HandleTypeDef *usb_handle_;              //! CubeMX 生成的 USBD 设备句柄
  STM32USB_RxCallback_t rx_callback_;           //! CDC OUT 数据回调
  STM32USB_TxCompleteCallback_t tx_callback_;   //! CDC IN 完成回调
  err_t last_error_;                            //! 最近一次 BSP 操作结果
} STM32USB_t;

#ifdef __cplusplus
extern "C" {
#endif

// ==================== USB CDC 公开接口 ====================

//! 将 USB Device 句柄映射到 BSP USB 逻辑 ID。
//! CDC 回调里使用 hUsbDeviceFS，BSP 通过该函数反查对象表。
BSP_USB_t BSP_USB_get_id(USBD_HandleTypeDef *usb_handle);

//! 初始化 USB CDC BSP 控制块。
//! 只绑定 USB Device 句柄和用户回调；USB 设备启动仍由 MX_USB_DEVICE_Init() 完成。
err_t STM32USB_Init(STM32USB_t *self,
                    USBD_HandleTypeDef *usb_handle,
                    STM32USB_RxCallback_t rx_callback,
                    STM32USB_TxCompleteCallback_t tx_callback);

//! 通过 USB CDC 发送一段数据。
//! 使用 CubeMX 生成的 UserTxBufferFS 作为单块 TX 缓冲，USB 忙时返回 BUSY。
err_t STM32USB_Write(STM32USB_t *self, const uint8_t *data, size_t size);

//! 更新 USB CDC RX 用户回调。
void STM32USB_SetRxCallback(STM32USB_t *self,
                            STM32USB_RxCallback_t callback);

//! 更新 USB CDC TX 完成回调。
void STM32USB_SetTxCompleteCallback(STM32USB_t *self,
                                    STM32USB_TxCompleteCallback_t callback);

//! 检查 USB 设备是否已经枚举并进入 CONFIGURED 状态。
bool STM32USB_IsConfigured(const STM32USB_t *self);

//! 读取最近一次错误码。
err_t STM32USB_GetLastError(const STM32USB_t *self);

//! 处理 CDC OUT 收到的数据。
//! 由 CDC_Receive_FS() 调用；未注册对象时静默返回，保持 CubeMX 默认接收流程可用。
void STM32USB_HandleRxData(USBD_HandleTypeDef *usb_handle,
                           uint8_t *data,
                           size_t size);

//! 处理 CDC IN 发送完成事件。
//! 由 CDC_TransmitCplt_FS() 调用；未注册对象时静默返回。
void STM32USB_HandleTxComplete(USBD_HandleTypeDef *usb_handle);

#ifdef __cplusplus
}
#endif

#endif // __BSP_USB_H__
