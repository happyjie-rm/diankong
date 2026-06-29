#include "bsp_uart.h"

#include <string.h>

// ==================== DMA 对象表 ====================
//! RX = 硬件循环 DMA + 软件读指针；TX = 普通 DMA + 两块软件缓冲。
//! HAL 回调只负责切分数据和推进发送状态，协议解析和复杂业务应放在任务上下文。

//! 单缓冲 RX 控制块映射表。
//! 第一维按 BSP_UART_t 逻辑设备索引，HAL RX 事件通过外设 Instance 反查到对应对象。
static STM32UART_t *stm32_uart_map[BSP_UART_NUMBER] = {0};

//! 双缓冲 TX 控制块映射表。
//! HAL TX 完成回调通过该表找到正在发送的 TX 对象，并触发续发。
static STM32UARTDoubleBufTx_t *stm32_uart_double_buf_tx_map[BSP_UART_NUMBER] = {0};

//! 将 HAL 外设 Instance 转换为 BSP UART 逻辑 ID。
//! 该函数集中维护 UART 外设和 BSP 枚举的映射关系，新增串口时优先在这里补映射。
BSP_UART_t BSP_UART_get_id(USART_TypeDef *addr)
{
  if (addr == NULL)
  {
    return BSP_UART_ID_ERROR;
  }

#ifdef USART1
  if (addr == USART1)
  {
    return BSP_USART1;
  }
#endif
#ifdef USART2
  if (addr == USART2)
  {
    return BSP_USART2;
  }
#endif
#ifdef USART3
  if (addr == USART3)
  {
    return BSP_USART3;
  }
#endif
#ifdef USART6
  if (addr == USART6)
  {
    return BSP_USART6;
  }
#endif
#ifdef UART4
  if (addr == UART4)
  {
    return BSP_UART4;
  }
#endif
#ifdef UART5
  if (addr == UART5)
  {
    return BSP_UART5;
  }
#endif

  return BSP_UART_ID_ERROR;
}

//! 检查 BSP UART 逻辑 ID 是否可用。
//! BSP_UART_ID_ERROR 和越界值都视为非法，避免访问对象表越界。
static bool BSP_UART_is_valid_id(BSP_UART_t id)
{
  return (id != BSP_UART_ID_ERROR) && (id < BSP_UART_NUMBER);
}

// ==================== 单缓冲 DMA RX ====================

//! 初始化单缓冲 DMA RX 控制块。
//! 该函数只做参数绑定和对象注册；DMA 接收由 STM32UART_SetRxDMA() 显式启动。
err_t STM32UART_Init(STM32UART_t *self,
                     UART_HandleTypeDef *uart_handle,
                     BSP_UART_RawData_t dma_buff_rx,
                     STM32UART_RxCallback_t callback)
{
  if (self == NULL)
  {
    return PTR_NULL;
  }

  self->id_ = (uart_handle != NULL) ? BSP_UART_get_id(uart_handle->Instance) : BSP_UART_ID_ERROR;
  self->last_rx_pos_ = 0U;
  self->dma_buff_rx_ = dma_buff_rx;
  self->uart_handle_ = uart_handle;
  self->rx_callback_ = callback;
  self->last_error_ = PENDING;

  ASSERT(self->uart_handle_ != NULL);
  if (self->uart_handle_ == NULL)
  {
    self->last_error_ = PTR_NULL;
    return self->last_error_;
  }

  ASSERT(BSP_UART_is_valid_id(self->id_));
  if (!BSP_UART_is_valid_id(self->id_))
  {
    self->last_error_ = NOT_FOUND;
    return self->last_error_;
  }

  ASSERT(self->dma_buff_rx_.addr_ != NULL);
  if (self->dma_buff_rx_.addr_ == NULL)
  {
    self->last_error_ = PTR_NULL;
    return self->last_error_;
  }

  ASSERT(self->dma_buff_rx_.size_ > 0U);
  if (self->dma_buff_rx_.size_ == 0U)
  {
    self->last_error_ = SIZE_ERR;
    return self->last_error_;
  }

  ASSERT((stm32_uart_map[self->id_] == NULL) || (stm32_uart_map[self->id_] == self));
  if ((stm32_uart_map[self->id_] != NULL) && (stm32_uart_map[self->id_] != self))
  {
    self->last_error_ = BUSY;
    return self->last_error_;
  }

  stm32_uart_map[self->id_] = self;
  self->last_error_ = OK;
  return self->last_error_;
}

//! 启动单缓冲 DMA RX。
//! RX DMA 使用 DMA_CIRCULAR，HAL_UARTEx_ReceiveToIdle_DMA() 用 IDLE 事件驱动数据切分。
//! 调用前必须保证 dma_buff_rx_ 指向的缓冲区在整个接收期间有效。
err_t STM32UART_SetRxDMA(STM32UART_t *self)
{
  if (self == NULL)
  {
    return PTR_NULL;
  }

  ASSERT(self->uart_handle_ != NULL);
  if (self->uart_handle_ == NULL)
  {
    self->last_error_ = PTR_NULL;
    return self->last_error_;
  }

  if ((self->uart_handle_->Init.Mode & UART_MODE_RX) != UART_MODE_RX)
  {
    self->last_error_ = NOT_SUPPORT;
    return self->last_error_;
  }

  ASSERT(self->uart_handle_->hdmarx != NULL);
  if (self->uart_handle_->hdmarx == NULL)
  {
    self->last_error_ = PTR_NULL;
    return self->last_error_;
  }

  ASSERT(self->dma_buff_rx_.addr_ != NULL);
  if (self->dma_buff_rx_.addr_ == NULL)
  {
    self->last_error_ = PTR_NULL;
    return self->last_error_;
  }

  ASSERT(self->dma_buff_rx_.size_ > 0U);
  if (self->dma_buff_rx_.size_ == 0U)
  {
    self->last_error_ = SIZE_ERR;
    return self->last_error_;
  }

  self->uart_handle_->hdmarx->Init.Mode = DMA_CIRCULAR;
  const HAL_StatusTypeDef dma_status = HAL_DMA_Init(self->uart_handle_->hdmarx);
  VERIFY(dma_status == HAL_OK);
  if (dma_status != HAL_OK)
  {
    self->last_error_ = INIT_ERR;
    return self->last_error_;
  }

  const HAL_StatusTypeDef rx_status = HAL_UARTEx_ReceiveToIdle_DMA(
      self->uart_handle_,
      (uint8_t *)self->dma_buff_rx_.addr_,
      (uint16_t)self->dma_buff_rx_.size_);
  VERIFY(rx_status == HAL_OK);
  if (rx_status != HAL_OK)
  {
    self->last_error_ = INIT_ERR;
    return self->last_error_;
  }

  self->last_error_ = OK;
  return self->last_error_;
}

//! 更新单缓冲 RX 用户回调。
//! 不触碰 DMA 状态，只替换后续 RX 数据片段的处理入口。
void STM32UART_SetRxCallback(STM32UART_t *self, STM32UART_RxCallback_t callback)
{
  if (self == NULL)
  {
    return;
  }

  self->rx_callback_ = callback;
}

//! 获取单缓冲 RX 最近一次错误码。
err_t STM32UART_GetLastError(const STM32UART_t *self)
{
  if (self == NULL)
  {
    return PTR_NULL;
  }

  return self->last_error_;
}

//! 将一段 RX 数据交给用户回调。
//! ISR 路径已经完成环形缓冲切片；这里仅做防御检查并调用 rx_callback_。
//! 用户回调可能运行在 HAL 回调上下文中，不应执行长时间阻塞操作。
void STM32UART_HandleRxData(STM32UART_t *self, uint8_t *data, size_t size)
{
  if (self == NULL)
  {
    return;
  }

  ASSERT(self->rx_callback_ != NULL);
  if (self->rx_callback_ == NULL)
  {
    self->last_error_ = PTR_NULL;
    return;
  }

  ASSERT(data != NULL);
  if (data == NULL)
  {
    self->last_error_ = PTR_NULL;
    return;
  }

  ASSERT(size > 0U);
  if (size == 0U)
  {
    self->last_error_ = SIZE_ERR;
    return;
  }

  self->last_error_ = OK;
  self->rx_callback_(data, size);
}

//! 单缓冲 RX 事件处理入口。
//! 根据 DMA 剩余计数计算当前写入位置，并把 last_rx_pos_ 到 curr_pos 的增量切片回调。
//! 环形缓冲回绕时会拆成尾部和头部两段分别回调。
static void STM32_UART_RX_ISR_Handler(UART_HandleTypeDef *uart_handle)
{
  ASSERT(uart_handle != NULL);
  if (uart_handle == NULL)
  {
    return;
  }

  const BSP_UART_t id = BSP_UART_get_id(uart_handle->Instance);
  ASSERT(BSP_UART_is_valid_id(id));
  if (!BSP_UART_is_valid_id(id))
  {
    return;
  }

  STM32UART_t *uart = stm32_uart_map[id];
  ASSERT(uart != NULL);
  if (uart == NULL)
  {
    return;
  }

  ASSERT(uart_handle->hdmarx != NULL);
  if (uart_handle->hdmarx == NULL)
  {
    uart->last_error_ = PTR_NULL;
    return;
  }

  uint8_t *rx_buf = (uint8_t *)uart->dma_buff_rx_.addr_;
  const size_t dma_size = uart->dma_buff_rx_.size_;
  ASSERT(rx_buf != NULL);
  if (rx_buf == NULL)
  {
    uart->last_error_ = PTR_NULL;
    return;
  }

  ASSERT(dma_size > 0U);
  if (dma_size == 0U)
  {
    uart->last_error_ = SIZE_ERR;
    return;
  }

  const size_t dma_remaining = (size_t)__HAL_DMA_GET_COUNTER(uart_handle->hdmarx);
  ASSERT(dma_remaining <= dma_size);
  if (dma_remaining > dma_size)
  {
    uart->last_error_ = OUT_OF_RANGE;
    return;
  }

  const size_t curr_pos = dma_size - dma_remaining;
  const size_t last_pos = uart->last_rx_pos_;
  ASSERT(last_pos <= dma_size);
  if (last_pos > dma_size)
  {
    uart->last_error_ = OUT_OF_RANGE;
    uart->last_rx_pos_ = curr_pos;
    return;
  }

  if (curr_pos != last_pos)
  {
    if (curr_pos > last_pos)
    {
      const size_t data_size = curr_pos - last_pos;
      STM32UART_HandleRxData(uart, rx_buf + last_pos, data_size);
    }
    else
    {
      const size_t first_part_size = dma_size - last_pos;
      STM32UART_HandleRxData(uart, rx_buf + last_pos, first_part_size);

      if (curr_pos > 0U)
      {
        const size_t second_part_size = curr_pos;
        STM32UART_HandleRxData(uart, rx_buf, second_part_size);
      }
    }

    uart->last_rx_pos_ = curr_pos;
  }
}

// ==================== 双缓冲 DMA TX ====================

//! 初始化双缓冲 DMA TX 控制块。
//! 两块软件缓冲轮流作为 DMA 源，允许 DMA 忙时提前填充下一块 pending 数据。
err_t STM32UARTDoubleBufTx_Init(STM32UARTDoubleBufTx_t *self,
                                UART_HandleTypeDef *uart_handle,
                                BSP_UART_RawData_t dma_buff_0,
                                BSP_UART_RawData_t dma_buff_1,
                                STM32UART_TxCompleteCallback_t callback)
{
  if (self == NULL)
  {
    return PTR_NULL;
  }

  self->id_ = (uart_handle != NULL) ? BSP_UART_get_id(uart_handle->Instance) : BSP_UART_ID_ERROR;
  self->last_tx_pos_ = 0U;
  self->dma_buff_0_ = dma_buff_0;
  self->dma_buff_1_ = dma_buff_1;
  self->uart_handle_ = uart_handle;
  self->tx_callback_ = callback;
  self->last_error_ = PENDING;
  self->active_buf_ = 0U;
  self->pending_size_ = 0U;
  self->tx_busy_ = false;

  ASSERT(self->uart_handle_ != NULL);
  if (self->uart_handle_ == NULL)
  {
    self->last_error_ = PTR_NULL;
    return self->last_error_;
  }

  ASSERT(BSP_UART_is_valid_id(self->id_));
  if (!BSP_UART_is_valid_id(self->id_))
  {
    self->last_error_ = NOT_FOUND;
    return self->last_error_;
  }

  ASSERT(self->dma_buff_0_.addr_ != NULL);
  if (self->dma_buff_0_.addr_ == NULL)
  {
    self->last_error_ = PTR_NULL;
    return self->last_error_;
  }

  ASSERT(self->dma_buff_1_.addr_ != NULL);
  if (self->dma_buff_1_.addr_ == NULL)
  {
    self->last_error_ = PTR_NULL;
    return self->last_error_;
  }

  ASSERT((self->dma_buff_0_.size_ > 0U) &&
         (self->dma_buff_0_.size_ == self->dma_buff_1_.size_));
  if ((self->dma_buff_0_.size_ == 0U) ||
      (self->dma_buff_0_.size_ != self->dma_buff_1_.size_))
  {
    self->last_error_ = SIZE_ERR;
    return self->last_error_;
  }

  ASSERT((stm32_uart_double_buf_tx_map[self->id_] == NULL) ||
         (stm32_uart_double_buf_tx_map[self->id_] == self));
  if ((stm32_uart_double_buf_tx_map[self->id_] != NULL) &&
      (stm32_uart_double_buf_tx_map[self->id_] != self))
  {
    self->last_error_ = BUSY;
    return self->last_error_;
  }

  stm32_uart_double_buf_tx_map[self->id_] = self;
  self->last_error_ = OK;
  return self->last_error_;
}

//! 配置双缓冲 TX 的 DMA 通道。
//! TX DMA 使用 DMA_NORMAL，每次 Flush 只发送当前 active buffer 的 pending 数据。
err_t STM32UARTDoubleBufTx_SetTxDMA(STM32UARTDoubleBufTx_t *self)
{
  if (self == NULL)
  {
    return PTR_NULL;
  }

  ASSERT(self->uart_handle_ != NULL);
  if (self->uart_handle_ == NULL)
  {
    self->last_error_ = PTR_NULL;
    return self->last_error_;
  }

  if ((self->uart_handle_->Init.Mode & UART_MODE_TX) != UART_MODE_TX)
  {
    self->last_error_ = NOT_SUPPORT;
    return self->last_error_;
  }

  ASSERT(self->uart_handle_->hdmatx != NULL);
  if (self->uart_handle_->hdmatx == NULL)
  {
    self->last_error_ = PTR_NULL;
    return self->last_error_;
  }

  self->uart_handle_->hdmatx->Init.Mode = DMA_NORMAL;
  const HAL_StatusTypeDef dma_status = HAL_DMA_Init(self->uart_handle_->hdmatx);
  VERIFY(dma_status == HAL_OK);
  if (dma_status != HAL_OK)
  {
    self->last_error_ = INIT_ERR;
    return self->last_error_;
  }

  self->active_buf_ = 0U;
  self->pending_size_ = 0U;
  self->tx_busy_ = false;

  self->last_error_ = OK;
  return self->last_error_;
}

//! 写入一帧 TX 数据。
//! DMA 空闲时写入 active buffer 并立即启动发送；DMA 忙时写入另一块缓冲等待完成回调续发。
//! 本实现只保留一块 pending 缓冲，连续写入会覆盖尚未发送的 pending 数据，调用方需控制节奏。
err_t STM32UARTDoubleBufTx_Write(STM32UARTDoubleBufTx_t *self,
                                 const uint8_t *data,
                                 size_t size)
{
  if (self == NULL)
  {
    return PTR_NULL;
  }

  ASSERT(data != NULL);
  if (data == NULL)
  {
    self->last_error_ = PTR_NULL;
    return self->last_error_;
  }

  ASSERT(size > 0U);
  if (size == 0U)
  {
    self->last_error_ = SIZE_ERR;
    return self->last_error_;
  }

  ASSERT(size <= self->dma_buff_0_.size_);
  if (size > self->dma_buff_0_.size_)
  {
    self->last_error_ = OUT_OF_RANGE;
    return self->last_error_;
  }

  const uint8_t fill_buf = self->tx_busy_ ? (uint8_t)(1U - self->active_buf_) : self->active_buf_;
  void *buf_addr = (fill_buf == 0U) ? self->dma_buff_0_.addr_ : self->dma_buff_1_.addr_;

  memcpy(buf_addr, data, size);
  self->pending_size_ = size;

  if (!self->tx_busy_)
  {
    return STM32UARTDoubleBufTx_Flush(self);
  }

  self->last_error_ = OK;
  return self->last_error_;
}

//! 提交 pending 数据到 HAL DMA 发送。
//! Flush 成功后 pending_size_ 清零，发送完成后由 HAL_UART_TxCpltCallback() 切换 active buffer。
err_t STM32UARTDoubleBufTx_Flush(STM32UARTDoubleBufTx_t *self)
{
  if (self == NULL)
  {
    return PTR_NULL;
  }

  if (self->pending_size_ == 0U)
  {
    self->last_error_ = EMPTY;
    return self->last_error_;
  }

  if (self->tx_busy_)
  {
    self->last_error_ = BUSY;
    return self->last_error_;
  }

  void *buf_addr = (self->active_buf_ == 0U) ? self->dma_buff_0_.addr_ : self->dma_buff_1_.addr_;

  self->tx_busy_ = true;
  const HAL_StatusTypeDef tx_status = HAL_UART_Transmit_DMA(
      self->uart_handle_,
      (uint8_t *)buf_addr,
      (uint16_t)self->pending_size_);
  VERIFY(tx_status == HAL_OK);
  if (tx_status != HAL_OK)
  {
    self->tx_busy_ = false;
    self->last_error_ = FAILED;
    return self->last_error_;
  }

  self->pending_size_ = 0U;
  self->last_error_ = OK;
  return self->last_error_;
}

//! 更新双缓冲 TX 完成回调。
//! 不影响正在进行的 DMA 发送，只改变后续完成事件通知对象。
void STM32UARTDoubleBufTx_SetTxCompleteCallback(
    STM32UARTDoubleBufTx_t *self,
    STM32UART_TxCompleteCallback_t callback)
{
  if (self == NULL)
  {
    return;
  }

  self->tx_callback_ = callback;
}

//! 获取双缓冲 TX 最近一次错误码。
err_t STM32UARTDoubleBufTx_GetLastError(const STM32UARTDoubleBufTx_t *self)
{
  if (self == NULL)
  {
    return PTR_NULL;
  }

  return self->last_error_;
}

//! 处理 TX DMA 完成事件。
//! 释放 busy 状态、切换 active buffer；如果另一块缓冲有 pending 数据则立即续发。
void STM32UARTDoubleBufTx_HandleTxComplete(STM32UARTDoubleBufTx_t *self)
{
  if (self == NULL)
  {
    return;
  }

  self->tx_busy_ = false;
  self->active_buf_ = (uint8_t)(1U - self->active_buf_);

  if (self->pending_size_ > 0U)
  {
    (void)STM32UARTDoubleBufTx_Flush(self);
  }

  if (self->tx_callback_ != NULL)
  {
    self->tx_callback_();
  }
}

// ==================== HAL 回调分发 ====================

//! HAL Receive-To-Idle 事件回调。
//! HAL 只提供 UART 句柄和本轮 size；这里反查 BSP 对象后按 DMA 计数器切片。
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
  RM_UNUSED(size);

  if (huart == NULL)
  {
    return;
  }

  const BSP_UART_t id = BSP_UART_get_id(huart->Instance);
  if (!BSP_UART_is_valid_id(id))
  {
    return;
  }

  if (stm32_uart_map[id] != NULL)
  {
    STM32_UART_RX_ISR_Handler(huart);
  }
}

//! HAL TX DMA 完成回调。
//! 反查双缓冲 TX 控制块并推进发送状态；未注册对象时静默返回。
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart == NULL)
  {
    return;
  }

  const BSP_UART_t id = BSP_UART_get_id(huart->Instance);
  if (!BSP_UART_is_valid_id(id))
  {
    return;
  }

  STM32UARTDoubleBufTx_t *tx = stm32_uart_double_buf_tx_map[id];
  if (tx != NULL)
  {
    STM32UARTDoubleBufTx_HandleTxComplete(tx);
  }
}
