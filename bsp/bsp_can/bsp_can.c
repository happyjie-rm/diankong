#include "bsp_can.h"

#include <string.h>

// ==================== CAN 对象表 ====================
//! HAL 回调只提供 CAN_HandleTypeDef，BSP 通过 Instance 反查对象式控制块。

static STM32CAN_t *stm32_can_map[BSP_CAN_NUMBER] = {0};

//! CAN2 历史弱回调触发次数，保留给旧业务调试观察。
uint32_t can2_rx_callback_cnt = 0;

//! 将 HAL 外设 Instance 转换为 BSP CAN 逻辑 ID。
BSP_CAN_t BSP_CAN_get_id(CAN_TypeDef *addr)
{
  if (addr == NULL)
  {
    return BSP_CAN_ID_ERROR;
  }

#ifdef CAN1
  if (addr == CAN1)
  {
    return BSP_CAN1;
  }
#endif
#ifdef CAN2
  if (addr == CAN2)
  {
    return BSP_CAN2;
  }
#endif

  return BSP_CAN_ID_ERROR;
}

//! 检查 BSP CAN 逻辑 ID 是否可用。
static bool BSP_CAN_is_valid_id(BSP_CAN_t id)
{
  return (id != BSP_CAN_ID_ERROR) && (id < BSP_CAN_NUMBER);
}

//! 根据 HAL 句柄查找已注册的 CAN 控制块。
static STM32CAN_t *BSP_CAN_get_object(CAN_HandleTypeDef *hcan)
{
  if ((hcan == NULL) || (hcan->Instance == NULL))
  {
    return NULL;
  }

  const BSP_CAN_t id = BSP_CAN_get_id(hcan->Instance);
  if (!BSP_CAN_is_valid_id(id))
  {
    return NULL;
  }

  return stm32_can_map[id];
}

//! 同步更新已注册对象的错误码。
static void BSP_CAN_set_handle_error(CAN_HandleTypeDef *hcan, err_t error)
{
  STM32CAN_t *can = BSP_CAN_get_object(hcan);
  if (can != NULL)
  {
    can->last_error_ = error;
  }
}

//! 启动 HAL CAN 并开启 RX FIFO0/FIFO1 消息挂起中断。
static err_t BSP_CAN_start_handle(CAN_HandleTypeDef *hcan)
{
  ASSERT(hcan != NULL);
  if (hcan == NULL)
  {
    return PTR_NULL;
  }

  HAL_StatusTypeDef status = HAL_OK;
  if (hcan->State != HAL_CAN_STATE_LISTENING)
  {
    status = HAL_CAN_Start(hcan);
    VERIFY(status == HAL_OK);
    if (status != HAL_OK)
    {
      return INIT_ERR;
    }
  }

  status = HAL_CAN_ActivateNotification(
      hcan,
      CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO1_MSG_PENDING);
  VERIFY(status == HAL_OK);
  if (status != HAL_OK)
  {
    return INIT_ERR;
  }

  return OK;
}

//! 尝试最多 3 次寻找空邮箱并发送 CAN 帧，避免邮箱瞬时忙导致直接丢包。
static err_t BSP_CAN_send_with_retry(CAN_HandleTypeDef *hcan,
                                     CAN_TxHeaderTypeDef *tx_header,
                                     const uint8_t *data)
{
  ASSERT(hcan != NULL);
  if (hcan == NULL)
  {
    return PTR_NULL;
  }

  ASSERT(tx_header != NULL);
  if (tx_header == NULL)
  {
    return PTR_NULL;
  }

  ASSERT(data != NULL);
  if (data == NULL)
  {
    return PTR_NULL;
  }

  uint32_t used_mailbox = 0U;
  bool mailbox_seen = false;

  for (uint8_t attempt = 0U; attempt < 3U; ++attempt)
  {
    if (HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0U)
    {
      continue;
    }

    mailbox_seen = true;
    if (HAL_CAN_AddTxMessage(hcan, tx_header, (uint8_t *)data,
                             &used_mailbox) == HAL_OK)
    {
      return OK;
    }
  }

  return mailbox_seen ? FAILED : BUSY;
}

//! 使用 HAL 标准帧发送一段数据。
static err_t BSP_CAN_send_std_data(CAN_HandleTypeDef *hcan,
                                   uint32_t std_id,
                                   const uint8_t *data,
                                   size_t size)
{
  ASSERT(hcan != NULL);
  if (hcan == NULL)
  {
    return PTR_NULL;
  }

  ASSERT(data != NULL);
  if (data == NULL)
  {
    return PTR_NULL;
  }

  ASSERT(size > 0U);
  if (size == 0U)
  {
    return SIZE_ERR;
  }

  ASSERT(size <= CAN_DATA_SIZE);
  if (size > CAN_DATA_SIZE)
  {
    return OUT_OF_RANGE;
  }

  ASSERT(std_id <= 0x7FFU);
  if (std_id > 0x7FFU)
  {
    return OUT_OF_RANGE;
  }

  CAN_TxHeaderTypeDef tx_header = {0};
  tx_header.StdId = std_id;
  tx_header.ExtId = 0U;
  tx_header.IDE = CAN_ID_STD;
  tx_header.RTR = CAN_RTR_DATA;
  tx_header.DLC = (uint32_t)size;
  tx_header.TransmitGlobalTime = DISABLE;

  return BSP_CAN_send_with_retry(hcan, &tx_header, data);
}

//! 将 DJI 四路 int16 电流打包为 8 字节 CAN 数据，高字节在前。
static void BSP_CAN_pack_dji_current(uint8_t data[CAN_DATA_SIZE],
                                     const int16_t current[4])
{
  for (uint8_t i = 0U; i < 4U; ++i)
  {
    data[2U * i] = (uint8_t)(current[i] >> 8);
    data[(2U * i) + 1U] = (uint8_t)current[i];
  }
}

//! 将 Object_Para 编码转换为 HAL 滤波器公共字段。
static void BSP_CAN_fill_filter_common(CAN_FilterTypeDef *filter,
                                       uint8_t object_para,
                                       uint32_t scale)
{
  filter->FilterBank = (uint32_t)(object_para >> 3);
  filter->FilterFIFOAssignment = (uint32_t)((object_para >> 2) & 0x01U);
  filter->FilterActivation = ENABLE;
  filter->FilterMode = CAN_FILTERMODE_IDMASK;
  filter->FilterScale = scale;
  filter->SlaveStartFilterBank = 14U;
}

//! 兼容旧接口的 16 位滤波器配置生成。
static void BSP_CAN_make_filter_16bit(CAN_FilterTypeDef *filter,
                                      uint8_t object_para,
                                      uint16_t id1,
                                      uint16_t mask1,
                                      uint16_t id2,
                                      uint16_t mask2)
{
  memset(filter, 0, sizeof(*filter));

  filter->FilterIdHigh = id1;
  filter->FilterMaskIdHigh = mask1;
  filter->FilterIdLow = id2;
  filter->FilterMaskIdLow = mask2;

  BSP_CAN_fill_filter_common(filter, object_para, CAN_FILTERSCALE_16BIT);
}

//! 兼容旧接口的 32 位滤波器配置生成。
static void BSP_CAN_make_filter_32bit(CAN_FilterTypeDef *filter,
                                      uint8_t object_para,
                                      uint32_t id,
                                      uint32_t mask_id)
{
  memset(filter, 0, sizeof(*filter));

  const bool is_standard = (object_para & CAN_STDID) != 0U;
  const bool is_remote = (object_para & CAN_REMOTE_TYPE) != 0U;
  const uint32_t rtr_bit = is_remote ? CAN_RTR_REMOTE : CAN_RTR_DATA;

  if (is_standard)
  {
    const uint32_t filter_id = (id << 5) | CAN_ID_STD | rtr_bit;
    const uint32_t filter_mask = (mask_id << 5) | CAN_ID_EXT | CAN_RTR_REMOTE;

    filter->FilterIdHigh = (uint16_t)(filter_id >> 16);
    filter->FilterIdLow = (uint16_t)filter_id;
    filter->FilterMaskIdHigh = (uint16_t)(filter_mask >> 16);
    filter->FilterMaskIdLow = (uint16_t)filter_mask;
  }
  else
  {
    const uint32_t filter_id = (id << 3) | CAN_ID_EXT | rtr_bit;
    const uint32_t filter_mask = (mask_id << 3) | CAN_ID_EXT | CAN_RTR_REMOTE;

    filter->FilterIdHigh = (uint16_t)(filter_id >> 16);
    filter->FilterIdLow = (uint16_t)filter_id;
    filter->FilterMaskIdHigh = (uint16_t)(filter_mask >> 16);
    filter->FilterMaskIdLow = (uint16_t)filter_mask;
  }

  BSP_CAN_fill_filter_common(filter, object_para, CAN_FILTERSCALE_32BIT);
}

// ==================== 对象式 CAN 接口 ====================

//! 初始化 CAN BSP 控制块。
err_t STM32CAN_Init(STM32CAN_t *self,
                    CAN_HandleTypeDef *can_handle,
                    STM32CAN_RxCallback_t callback)
{
  if (self == NULL)
  {
    return PTR_NULL;
  }

  self->id_ = (can_handle != NULL) ? BSP_CAN_get_id(can_handle->Instance)
                                   : BSP_CAN_ID_ERROR;
  self->can_handle_ = can_handle;
  self->rx_callback_ = callback;
  self->last_error_ = PENDING;

  ASSERT(self->can_handle_ != NULL);
  if (self->can_handle_ == NULL)
  {
    self->last_error_ = PTR_NULL;
    return self->last_error_;
  }

  ASSERT(BSP_CAN_is_valid_id(self->id_));
  if (!BSP_CAN_is_valid_id(self->id_))
  {
    self->last_error_ = NOT_FOUND;
    return self->last_error_;
  }

  ASSERT((stm32_can_map[self->id_] == NULL) ||
         (stm32_can_map[self->id_] == self));
  if ((stm32_can_map[self->id_] != NULL) && (stm32_can_map[self->id_] != self))
  {
    self->last_error_ = BUSY;
    return self->last_error_;
  }

  stm32_can_map[self->id_] = self;
  self->last_error_ = OK;
  return self->last_error_;
}

//! 启动 CAN 并开启 RX FIFO 消息挂起通知。
err_t STM32CAN_Start(STM32CAN_t *self)
{
  if (self == NULL)
  {
    return PTR_NULL;
  }

  ASSERT(self->can_handle_ != NULL);
  if (self->can_handle_ == NULL)
  {
    self->last_error_ = PTR_NULL;
    return self->last_error_;
  }

  self->last_error_ = BSP_CAN_start_handle(self->can_handle_);
  return self->last_error_;
}

//! 配置 HAL CAN 滤波器。
err_t STM32CAN_ConfigFilter(STM32CAN_t *self,
                            const CAN_FilterTypeDef *filter)
{
  if (self == NULL)
  {
    return PTR_NULL;
  }

  ASSERT(self->can_handle_ != NULL);
  if (self->can_handle_ == NULL)
  {
    self->last_error_ = PTR_NULL;
    return self->last_error_;
  }

  ASSERT(filter != NULL);
  if (filter == NULL)
  {
    self->last_error_ = PTR_NULL;
    return self->last_error_;
  }

  const HAL_StatusTypeDef status =
      HAL_CAN_ConfigFilter(self->can_handle_, (CAN_FilterTypeDef *)filter);
  VERIFY(status == HAL_OK);
  self->last_error_ = (status == HAL_OK) ? OK : FAILED;
  return self->last_error_;
}

//! 发送一帧 CAN 标准数据帧。
err_t STM32CAN_Send(STM32CAN_t *self,
                    uint32_t std_id,
                    const uint8_t *data,
                    size_t size)
{
  if (self == NULL)
  {
    return PTR_NULL;
  }

  self->last_error_ =
      BSP_CAN_send_std_data(self->can_handle_, std_id, data, size);
  return self->last_error_;
}

//! 发送 DJI 电机四路电流控制帧，高字节在前。
err_t STM32CAN_SendDjiCurrent(STM32CAN_t *self,
                              uint32_t ctrl_id,
                              const int16_t current[4])
{
  if (self == NULL)
  {
    return PTR_NULL;
  }

  ASSERT(current != NULL);
  if (current == NULL)
  {
    self->last_error_ = PTR_NULL;
    return self->last_error_;
  }

  ASSERT(ctrl_id <= 0x7FFU);
  if (ctrl_id > 0x7FFU)
  {
    self->last_error_ = OUT_OF_RANGE;
    return self->last_error_;
  }

  uint8_t data[CAN_DATA_SIZE] = {0};
  BSP_CAN_pack_dji_current(data, current);

  self->last_error_ =
      BSP_CAN_send_std_data(self->can_handle_, ctrl_id, data, CAN_DATA_SIZE);
  return self->last_error_;
}

//! 更新对象式 CAN 接收回调。
void STM32CAN_SetRxCallback(STM32CAN_t *self,
                            STM32CAN_RxCallback_t callback)
{
  if (self == NULL)
  {
    return;
  }

  self->rx_callback_ = callback;
}

//! 读取最近一次错误码。
err_t STM32CAN_GetLastError(const STM32CAN_t *self)
{
  if (self == NULL)
  {
    return PTR_NULL;
  }

  return self->last_error_;
}

// ==================== 兼容旧接口 ====================

//! 启动 CAN 并开启 FIFO0/FIFO1 消息挂起中断。
void CAN_Init(CAN_HandleTypeDef *hcan)
{
  const err_t error = BSP_CAN_start_handle(hcan);
  BSP_CAN_set_handle_error(hcan, error);
}

//! 配置 16 位掩码模式滤波器，一次配置两个 16 位规则。
void CAN_Filter_Mask_Config_16bit(CAN_HandleTypeDef *hcan, uint8_t Object_Para,
                                  uint16_t ID1, uint16_t Mask1,
                                  uint16_t ID2, uint16_t Mask2)
{
  ASSERT(hcan != NULL);
  if (hcan == NULL)
  {
    return;
  }

  CAN_FilterTypeDef filter;
  BSP_CAN_make_filter_16bit(&filter, Object_Para, ID1, Mask1, ID2, Mask2);

  const HAL_StatusTypeDef status = HAL_CAN_ConfigFilter(hcan, &filter);
  VERIFY(status == HAL_OK);
  BSP_CAN_set_handle_error(hcan, (status == HAL_OK) ? OK : FAILED);
}

//! 配置 32 位掩码模式滤波器。
void CAN_Filter_Mask_Config_32bit(CAN_HandleTypeDef *hcan, uint8_t Object_Para,
                                  uint32_t ID, uint32_t Mask_ID)
{
  ASSERT(hcan != NULL);
  if (hcan == NULL)
  {
    return;
  }

  CAN_FilterTypeDef filter;
  BSP_CAN_make_filter_32bit(&filter, Object_Para, ID, Mask_ID);

  const HAL_StatusTypeDef status = HAL_CAN_ConfigFilter(hcan, &filter);
  VERIFY(status == HAL_OK);
  BSP_CAN_set_handle_error(hcan, (status == HAL_OK) ? OK : FAILED);
}

//! 发送 DJI 电机四路电流控制帧。
uint8_t dj_CAN_Send_Data(CAN_HandleTypeDef *hcan, uint16_t ID, int16_t cm1_iq,
                         int16_t cm2_iq, int16_t cm3_iq, int16_t cm4_iq,
                         uint16_t Length)
{
  const int16_t current[4] = {cm1_iq, cm2_iq, cm3_iq, cm4_iq};
  uint8_t data[CAN_DATA_SIZE] = {0};
  BSP_CAN_pack_dji_current(data, current);

  const err_t error = BSP_CAN_send_std_data(hcan, ID, data, Length);
  BSP_CAN_set_handle_error(hcan, error);
  return (error == OK) ? 1U : 0U;
}

//! 兼容历史声明：发送一帧 8 字节全零标准帧。
void CAN_Send_Data_X8(CAN_HandleTypeDef *hcan, uint16_t ID)
{
  const uint8_t data[CAN_DATA_SIZE] = {0};
  const err_t error = BSP_CAN_send_std_data(hcan, ID, data, CAN_DATA_SIZE);
  BSP_CAN_set_handle_error(hcan, error);
}

//! 从指定 FIFO 读取一帧 CAN 数据，并返回 DLC 长度。
uint8_t canx_receive(hcan_t *hcan, uint16_t *rec_id, uint8_t *buf,
                     uint32_t fifo)
{
  ASSERT(hcan != NULL);
  ASSERT(rec_id != NULL);
  ASSERT(buf != NULL);
  if ((hcan == NULL) || (rec_id == NULL) || (buf == NULL))
  {
    return 0U;
  }

  CAN_RxHeaderTypeDef rx_header = {0};
  if (HAL_CAN_GetRxMessage(hcan, fifo, &rx_header, buf) != HAL_OK)
  {
    BSP_CAN_set_handle_error(hcan, FAILED);
    return 0U;
  }

  *rec_id = (uint16_t)((rx_header.IDE == CAN_ID_STD) ? rx_header.StdId
                                                     : rx_header.ExtId);
  BSP_CAN_set_handle_error(hcan, OK);
  return (uint8_t)rx_header.DLC;
}

//! 供用户重写的 CAN1 达妙电机接收回调。
__weak void dm_can1_rx_callback(hcan_t *hcan, uint16_t *rec_id,
                                uint8_t *rx_data)
{
  RM_UNUSED(hcan);
  RM_UNUSED(rec_id);
  RM_UNUSED(rx_data);
}

//! 供用户重写的 CAN2 达妙电机接收回调。
__weak void dm_can2_rx_callback(hcan_t *hcan, uint16_t *rec_id,
                                uint8_t *rx_data)
{
  RM_UNUSED(hcan);
  RM_UNUSED(rec_id);
  RM_UNUSED(rx_data);
}

//! 供用户重写的 CAN1 DJI 电机接收回调。
__weak void dj_motor_can1_rx_callback(hcan_t *hcan, uint16_t *rec_id,
                                      uint8_t *rx_data)
{
  RM_UNUSED(hcan);
  RM_UNUSED(rec_id);
  RM_UNUSED(rx_data);
}

//! 供用户重写的 CAN2 DJI 电机接收回调。
__weak void dj_motor_can2_rx_callback(hcan_t *hcan, uint16_t *rec_id,
                                      uint8_t *rx_data)
{
  RM_UNUSED(hcan);
  RM_UNUSED(rec_id);
  RM_UNUSED(rx_data);
}

//! 通用 CAN 标准帧发送接口，最多发送 8 字节数据。
uint8_t dm_can_send_data(CAN_HandleTypeDef *hcan, uint16_t id, uint8_t *data,
                         uint32_t len)
{
  const err_t error = BSP_CAN_send_std_data(hcan, id, data, (size_t)len);
  BSP_CAN_set_handle_error(hcan, error);
  return (error == OK) ? 1U : 0U;
}

// ==================== HAL 回调分发 ====================

//! 把 HAL RX 头转换为 BSP 统一帧结构。
static void BSP_CAN_fill_frame(BSP_CAN_Frame_t *frame,
                               const CAN_RxHeaderTypeDef *rx_header,
                               const uint8_t *rx_data,
                               uint32_t fifo)
{
  frame->id_ = (rx_header->IDE == CAN_ID_STD) ? rx_header->StdId
                                              : rx_header->ExtId;
  frame->ide_ = rx_header->IDE;
  frame->rtr_ = rx_header->RTR;
  frame->fifo_ = fifo;
  frame->size_ = (rx_header->DLC <= CAN_DATA_SIZE)
                     ? (uint8_t)rx_header->DLC
                     : (uint8_t)CAN_DATA_SIZE;
  memcpy(frame->data_, rx_data, CAN_DATA_SIZE);
}

//! 未注册对象式回调时，回落到历史弱回调分发。
static void BSP_CAN_dispatch_legacy(CAN_HandleTypeDef *hcan,
                                    BSP_CAN_Frame_t *frame)
{
  uint16_t rec_id = (uint16_t)frame->id_;

  if (hcan == &hcan1)
  {
    dm_can1_rx_callback(hcan, &rec_id, frame->data_);
    dj_motor_can1_rx_callback(hcan, &rec_id, frame->data_);
  }
  else if (hcan == &hcan2)
  {
    ++can2_rx_callback_cnt;
    dm_can2_rx_callback(hcan, &rec_id, frame->data_);
    dj_motor_can2_rx_callback(hcan, &rec_id, frame->data_);
  }
}

//! CAN RX 中断统一分发入口。
static void BSP_CAN_RX_ISR_Handler(CAN_HandleTypeDef *hcan, uint32_t fifo)
{
  ASSERT(hcan != NULL);
  if (hcan == NULL)
  {
    return;
  }

  CAN_RxHeaderTypeDef rx_header = {0};
  uint8_t rx_data[CAN_DATA_SIZE] = {0};
  if (HAL_CAN_GetRxMessage(hcan, fifo, &rx_header, rx_data) != HAL_OK)
  {
    BSP_CAN_set_handle_error(hcan, FAILED);
    return;
  }

  BSP_CAN_Frame_t frame = {0};
  BSP_CAN_fill_frame(&frame, &rx_header, rx_data, fifo);

  STM32CAN_t *can = BSP_CAN_get_object(hcan);
  if ((can != NULL) && (can->rx_callback_ != NULL))
  {
    can->last_error_ = OK;
    can->rx_callback_(can, &frame);
    return;
  }

  BSP_CAN_set_handle_error(hcan, OK);
  BSP_CAN_dispatch_legacy(hcan, &frame);
}

//! FIFO0 消息挂起回调。
void HAL_CAN_RxFifo0MsgPendingCallback(hcan_t *hcan)
{
  BSP_CAN_RX_ISR_Handler(hcan, CAN_RX_FIFO0);
}

//! FIFO1 消息挂起回调。
void HAL_CAN_RxFifo1MsgPendingCallback(hcan_t *hcan)
{
  BSP_CAN_RX_ISR_Handler(hcan, CAN_RX_FIFO1);
}
