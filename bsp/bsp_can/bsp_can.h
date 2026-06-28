#ifndef __BSP_CAN_H__
#define __BSP_CAN_H__

#include "can.h"
#include "comp_cmd.h"
#include "comp_utils.h"
#include "main.h"

typedef CAN_HandleTypeDef hcan_t;

// ==================== CAN 常量与类型 ====================

#define CAN_DATA_SIZE (8U)

//! CAN 滤波器参数打包宏。
//! Object_Para = FilterBank | FIFO | ID 类型 | 帧类型，便于一个字节描述滤波器配置。
#define CAN_FILTER(x) ((uint8_t)((x) << 3))

//! 接收 FIFO 选择。
#define CAN_FIFO_0 (0U << 2)
#define CAN_FIFO_1 (1U << 2)

//! 标准帧或扩展帧。
#define CAN_STDID (1U << 1)
#define CAN_EXTID (0U << 1)

//! 数据帧或远程帧。
#define CAN_DATA_TYPE (0U << 0)
#define CAN_REMOTE_TYPE (1U << 0)

//! BSP CAN 逻辑设备编号。
//! 枚举项按芯片实际启用的 CAN 宏展开，保持和 CubeMX 外设定义一致。
typedef enum
{
#ifdef CAN1
  BSP_CAN1,
#endif
#ifdef CAN2
  BSP_CAN2,
#endif

  BSP_CAN_NUMBER,
  BSP_CAN_ID_ERROR
} BSP_CAN_t;

//! CAN 接收帧描述。
//! BSP 统一使用该结构向对象式回调传递 HAL 收到的原始帧信息。
typedef struct
{
  uint32_t id_;                 //! 标准帧或扩展帧 ID
  uint32_t ide_;                //! HAL IDE 标志，区分标准帧/扩展帧
  uint32_t rtr_;                //! HAL RTR 标志，区分数据帧/远程帧
  uint32_t fifo_;               //! 收到该帧的 RX FIFO 编号
  uint8_t size_;                //! 实际数据长度，最大 8 字节
  uint8_t data_[CAN_DATA_SIZE]; //! CAN 数据区快照
} BSP_CAN_Frame_t;

typedef struct STM32CAN STM32CAN_t;

//! 用户接收回调。
//! frame 只在回调期间有效；若业务需要异步处理，应自行拷贝数据。
typedef void (*STM32CAN_RxCallback_t)(STM32CAN_t *self,
                                      const BSP_CAN_Frame_t *frame);

//! CAN BSP 控制块。
//! BSP 只保存 HAL 句柄和回调，不拥有 HAL 句柄生命周期。
struct STM32CAN
{
  BSP_CAN_t id_;                       //! BSP 逻辑设备编号
  CAN_HandleTypeDef *can_handle_;      //! CubeMX 生成的 HAL CAN 句柄
  STM32CAN_RxCallback_t rx_callback_;  //! 对象式 RX 分发回调
  err_t last_error_;                   //! 最近一次 BSP 操作结果
};



#ifdef __cplusplus
extern "C" {
#endif

// ==================== CAN 公开接口 ====================

//! 将 HAL 外设 Instance 映射到 BSP CAN 逻辑 ID。
BSP_CAN_t BSP_CAN_get_id(CAN_TypeDef *addr);

//! 初始化 CAN BSP 控制块。
err_t STM32CAN_Init(STM32CAN_t *self,
                    CAN_HandleTypeDef *can_handle,
                    STM32CAN_RxCallback_t callback);

//! 启动 CAN 并开启 FIFO0/FIFO1 消息挂起中断。
err_t STM32CAN_Start(STM32CAN_t *self);

//! 配置 HAL CAN 滤波器。
err_t STM32CAN_ConfigFilter(STM32CAN_t *self,
                            const CAN_FilterTypeDef *filter);

//! 发送一帧 CAN 标准数据帧。
err_t STM32CAN_Send(STM32CAN_t *self,
                    uint32_t std_id,
                    const uint8_t *data,
                    size_t size);

//! 发送 DJI 电机四路电流控制帧，高字节在前。
err_t STM32CAN_SendDjiCurrent(STM32CAN_t *self,
                              uint32_t ctrl_id,
                              const int16_t current[4]);

//! 更新对象式 CAN 接收回调。
void STM32CAN_SetRxCallback(STM32CAN_t *self,
                            STM32CAN_RxCallback_t callback);

//! 读取最近一次错误码。
err_t STM32CAN_GetLastError(const STM32CAN_t *self);

//! 兼容旧接口：启动 CAN 并开启 FIFO0/FIFO1 消息挂起中断。
void CAN_Init(CAN_HandleTypeDef *hcan);

//! 兼容旧接口：配置 16 位掩码模式滤波器，一次配置两个 16 位规则。
void CAN_Filter_Mask_Config_16bit(CAN_HandleTypeDef *hcan, uint8_t Object_Para,
                                  uint16_t ID1, uint16_t Mask1,
                                  uint16_t ID2, uint16_t Mask2);

//! 兼容旧接口：配置 32 位掩码模式滤波器。
void CAN_Filter_Mask_Config_32bit(CAN_HandleTypeDef *hcan, uint8_t Object_Para,
                                  uint32_t ID, uint32_t Mask_ID);

//! 兼容旧接口：发送 DJI 电机四路电流控制帧。
uint8_t dj_CAN_Send_Data(CAN_HandleTypeDef *hcan, uint16_t ID, int16_t cm1_iq,
                         int16_t cm2_iq, int16_t cm3_iq, int16_t cm4_iq,
                         uint16_t Length);

//! 兼容旧接口：发送一帧 8 字节全零标准帧。
void CAN_Send_Data_X8(CAN_HandleTypeDef *hcan, uint16_t ID);

//! 兼容旧接口：从指定 FIFO 读取一帧 CAN 数据，并返回 DLC 长度。
uint8_t canx_receive(hcan_t *hcan, uint16_t *recid, uint8_t *buf,
                     uint32_t fifo);

//! 供用户重写的 CAN1 达妙电机接收回调。
void dm_can1_rx_callback(hcan_t *hcan, uint16_t *rec_id, uint8_t *rx_data);
//! 供用户重写的 CAN2 达妙电机接收回调。
void dm_can2_rx_callback(hcan_t *hcan, uint16_t *rec_id, uint8_t *rx_data);
//! 供用户重写的 CAN1 DJI 电机接收回调。
void dj_motor_can1_rx_callback(hcan_t *hcan, uint16_t *rec_id, uint8_t *rx_data);
//! 供用户重写的 CAN2 DJI 电机接收回调。
void dj_motor_can2_rx_callback(hcan_t *hcan, uint16_t *rec_id, uint8_t *rx_data);

//! 兼容旧接口：通用 CAN 标准帧发送接口，最多发送 8 字节数据。
uint8_t dm_can_send_data(hcan_t *hcan, uint16_t id, uint8_t *data,
                         uint32_t len);

#ifdef __cplusplus
}
#endif

#endif // __BSP_CAN_H__
