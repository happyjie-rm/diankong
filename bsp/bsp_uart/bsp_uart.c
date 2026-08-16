#include "bsp_uart.h"

#include <string.h>

// ==================== DMA 对象表 ====================
//! RX = 硬件循环 DMA + 软件读指针；TX = 普通 DMA + 两块软件缓冲。
//! HAL 回调只负责切分数据和推进发送状态，协议解析和复杂业务应放在任务上下文。

//! 单缓冲 RX 控制块映射表。
//! 第一维按 BSP_UART_t 逻辑设备索引，HAL RX 事件通过外设 Instance 反查到对应对象。
static STM32UART_t *stm32_uart_map[BSP_UART_NUMBER] = {0};

//! 双缓冲 TX 控制块映射表（软件双缓冲）。
//! HAL TX 完成回调通过该表找到正在发送的 TX 对象，并触发续发。
static STM32UARTDoubleBufTx_t *stm32_uart_double_buf_tx_map[BSP_UART_NUMBER] = {0};

//! 硬件双缓冲 TX 控制块映射表（DBM 模式）。
//! HAL TC/HT 中断回调通过该表找到对应的硬件双缓冲对象。
static STM32UARTDoubleBufHwTx_t *stm32_uart_double_buf_hw_tx_map[BSP_UART_NUMBER] = {0};

//! 硬件双缓冲 RX 控制块映射表（DBM 模式）。
//! HAL TC/HT 中断回调通过该表找到对应的硬件双缓冲 RX 对象。
static STM32UARTDoubleBufHwRx_t *stm32_uart_double_buf_hw_rx_map[BSP_UART_NUMBER] = {0};

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

// ==================== 硬件双缓冲 DMA TX（DBM 模式） ====================

//! 初始化硬件双缓冲 DMA TX 控制块。
//! 只绑定句柄、两块 DMA 缓冲和回调；不触碰 DMA 寄存器。
err_t STM32UARTDoubleBufHwTx_Init(STM32UARTDoubleBufHwTx_t *self,
                                  UART_HandleTypeDef *uart_handle,
                                  BSP_UART_RawData_t dma_buff_0,
                                  BSP_UART_RawData_t dma_buff_1,
                                  STM32UART_TxCompleteCallback_t tc_callback,
                                  STM32UART_TxCompleteCallback_t ht_callback)
{
  if (self == NULL)
  {
    return PTR_NULL;
  }

  self->id_ = (uart_handle != NULL) ? BSP_UART_get_id(uart_handle->Instance) : BSP_UART_ID_ERROR;
  self->dma_buff_0_ = dma_buff_0;
  self->dma_buff_1_ = dma_buff_1;
  self->uart_handle_ = uart_handle;
  self->tc_callback_ = tc_callback;
  self->ht_callback_ = ht_callback;
  self->last_error_ = PENDING;
  self->write_buf_ = 0U;
  self->buf_size_ = 0U;

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

  self->buf_size_ = self->dma_buff_0_.size_;

  ASSERT((stm32_uart_double_buf_hw_tx_map[self->id_] == NULL) ||
         (stm32_uart_double_buf_hw_tx_map[self->id_] == self));
  if ((stm32_uart_double_buf_hw_tx_map[self->id_] != NULL) &&
      (stm32_uart_double_buf_hw_tx_map[self->id_] != self))
  {
    self->last_error_ = BUSY;
    return self->last_error_;
  }

  stm32_uart_double_buf_hw_tx_map[self->id_] = self;
  self->last_error_ = OK;
  return self->last_error_;
}

//! 配置 TX DMA 为硬件双缓冲模式（DBM）。
//!
//! 操作步骤：
//!   1. 将 DMA 通道设为 Circular 模式（DBM 需要 Circular）
//!   2. 设置 DMA_SxCR 的 DBM 位
//!   3. 分别设置 M0AR 和 M1AR 为两块缓冲地址
//!   4. 使能 TC 和 HT 中断
//!   5. 设置 NDTR 为单块缓冲大小
//!
//! 注意：HAL 库的 HAL_DMA_Init() 不处理 DBM 相关寄存器，
//!       因此需要直接操作寄存器完成 DBM 配置。
err_t STM32UARTDoubleBufHwTx_SetTxDBM(STM32UARTDoubleBufHwTx_t *self)
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

  ASSERT(self->buf_size_ > 0U);
  if (self->buf_size_ == 0U)
  {
    self->last_error_ = SIZE_ERR;
    return self->last_error_;
  }

  /* ---- 获取 DMA 流句柄 ---- */
  DMA_Stream_TypeDef *dma_stream = (DMA_Stream_TypeDef *)self->uart_handle_->hdmatx->Instance;
  ASSERT(dma_stream != NULL);
  if (dma_stream == NULL)
  {
    self->last_error_ = PTR_NULL;
    return self->last_error_;
  }

  /* ---- 步骤 1：先让 HAL 做基础初始化（外设/存储器地址、数据宽度等） ---- */
  /* 将 DMA 模式设为 CIRCULAR（DBM 需要 Circular） */
  self->uart_handle_->hdmatx->Init.Mode = DMA_CIRCULAR;
  const HAL_StatusTypeDef dma_status = HAL_DMA_Init(self->uart_handle_->hdmatx);
  VERIFY(dma_status == HAL_OK);
  if (dma_status != HAL_OK)
  {
    self->last_error_ = INIT_ERR;
    return self->last_error_;
  }

  /* ---- 步骤 2：手动配置 DBM 相关寄存器 ---- */
  /* 关闭 DMA 流（配置 DBM 前必须禁用） */
  CLEAR_BIT(dma_stream->CR, DMA_SxCR_EN);

  /* 设置 M0AR = 缓冲 0 地址 */
  dma_stream->M0AR = (uint32_t)self->dma_buff_0_.addr_;
  /* 设置 M1AR = 缓冲 1 地址 */
  dma_stream->M1AR = (uint32_t)self->dma_buff_1_.addr_;

  /* 设置 NDTR = 单块缓冲大小（F4 HAL 无 NbrOfData 字段，直接操作寄存器） */
  __HAL_DMA_SET_COUNTER(self->uart_handle_->hdmatx, (uint16_t)self->buf_size_);

  /* 使能 DBM 位 */
  SET_BIT(dma_stream->CR, DMA_SxCR_DBM);

  /* 使能 TC 和 HT 中断 */
  SET_BIT(dma_stream->CR, DMA_SxCR_TCIE | DMA_SxCR_HTIE);

  /* 重新使能 DMA 流 */
  SET_BIT(dma_stream->CR, DMA_SxCR_EN);

  self->write_buf_ = 0U;
  self->last_error_ = OK;
  return self->last_error_;
}

//! 写入一帧数据到当前空闲缓冲。
//!
//! 工作流程：
//!   - 首次调用时，写入缓冲 0 并调用 HAL_UART_Transmit_DMA() 启动发送。
//!   - DMA 启动后，硬件 DBM 自动在 M0AR/M1AR 之间切换。
//!   - 后续调用写入 write_buf_ 指向的空闲缓冲，DMA 硬件会自动读取。
//!
//! 限制：单次写入长度不能超过 buf_size_。
err_t STM32UARTDoubleBufHwTx_Write(STM32UARTDoubleBufHwTx_t *self,
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

  ASSERT(size <= self->buf_size_);
  if (size > self->buf_size_)
  {
    self->last_error_ = OUT_OF_RANGE;
    return self->last_error_;
  }

  ASSERT(self->uart_handle_ != NULL);
  if (self->uart_handle_ == NULL)
  {
    self->last_error_ = PTR_NULL;
    return self->last_error_;
  }

  /* 选择当前可写入的缓冲 */
  void *buf_addr = (self->write_buf_ == 0U) ? self->dma_buff_0_.addr_
                                            : self->dma_buff_1_.addr_;

  memcpy(buf_addr, data, size);

  /* 判断 DMA 是否已启动：检查 DMA 流是否使能 */
  DMA_Stream_TypeDef *dma_stream = (DMA_Stream_TypeDef *)self->uart_handle_->hdmatx->Instance;
  const bool dma_running = (dma_stream != NULL) &&
                           ((dma_stream->CR & DMA_SxCR_EN) != 0U);

  if (!dma_running)
  {
    /* 首次发送：启动 DMA */
    self->write_buf_ = 1U; /* 下次写入切到缓冲 1 */
    const HAL_StatusTypeDef tx_status = HAL_UART_Transmit_DMA(
        self->uart_handle_,
        (uint8_t *)buf_addr,
        (uint16_t)size);
    VERIFY(tx_status == HAL_OK);
    if (tx_status != HAL_OK)
    {
      self->last_error_ = FAILED;
      return self->last_error_;
    }
  }
  else
  {
    /* DMA 已在运行，切换 write_buf_ 让下次写入另一块 */
    self->write_buf_ = (uint8_t)(1U - self->write_buf_);
  }

  self->last_error_ = OK;
  return self->last_error_;
}

//! 获取硬件双缓冲 TX 最近一次错误码。
err_t STM32UARTDoubleBufHwTx_GetLastError(const STM32UARTDoubleBufHwTx_t *self)
{
  if (self == NULL)
  {
    return PTR_NULL;
  }

  return self->last_error_;
}

//! 处理硬件双缓冲 TC 中断（整块发完）。
//! 切换 write_buf_ 并调用 tc_callback_。
void STM32UARTDoubleBufHwTx_HandleTC(STM32UARTDoubleBufHwTx_t *self)
{
  if (self == NULL)
  {
    return;
  }

  /* TC 触发时，DMA 刚发完一块缓冲，write_buf_ 指向刚发完的那块，
   * 将其切换为另一块，让上层继续写入 */
  self->write_buf_ = (uint8_t)(1U - self->write_buf_);

  if (self->tc_callback_ != NULL)
  {
    self->tc_callback_();
  }
}

//! 处理硬件双缓冲 HT 中断（半块发完，即切到另一块）。
//! 切换 write_buf_ 并调用 ht_callback_。
void STM32UARTDoubleBufHwTx_HandleHT(STM32UARTDoubleBufHwTx_t *self)
{
  if (self == NULL)
  {
    return;
  }

  /* HT 触发时，DMA 刚切到另一块缓冲开始发送，
   * write_buf_ 指向刚切过来的这块，将其切换让上层写入刚发完的那块 */
  self->write_buf_ = (uint8_t)(1U - self->write_buf_);

  if (self->ht_callback_ != NULL)
  {
    self->ht_callback_();
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
//! 优先匹配 RX 硬件双缓冲（DBM），其次匹配 TX 硬件双缓冲（DBM），最后匹配软件双缓冲。
//! 未注册对象时静默返回。
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

  /* 优先匹配 RX 硬件双缓冲（DBM）—— RX DMA TC = 缓冲 0 写满 */
  STM32UARTDoubleBufHwRx_t *hw_rx = stm32_uart_double_buf_hw_rx_map[id];
  if (hw_rx != NULL)
  {
    STM32UARTDoubleBufHwRx_HandleTC(hw_rx);
    return;
  }

  /* 其次匹配 TX 硬件双缓冲（DBM） */
  STM32UARTDoubleBufHwTx_t *hw_tx = stm32_uart_double_buf_hw_tx_map[id];
  if (hw_tx != NULL)
  {
    STM32UARTDoubleBufHwTx_HandleTC(hw_tx);
    return;
  }

  /* 回退到软件双缓冲 */
  STM32UARTDoubleBufTx_t *tx = stm32_uart_double_buf_tx_map[id];
  if (tx != NULL)
  {
    STM32UARTDoubleBufTx_HandleTxComplete(tx);
  }
}

//! HAL TX DMA 半传输完成回调（用于硬件双缓冲 HT 中断）。
//! 优先匹配 RX 硬件双缓冲（DBM），其次匹配 TX 硬件双缓冲（DBM）。
//! 仅当使用 DBM 模式时注册该回调。
void HAL_UART_TxHalfCpltCallback(UART_HandleTypeDef *huart)
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

  /* 优先匹配 RX 硬件双缓冲（DBM）—— RX DMA HT = 缓冲 1 写满 */
  STM32UARTDoubleBufHwRx_t *hw_rx = stm32_uart_double_buf_hw_rx_map[id];
  if (hw_rx != NULL)
  {
    STM32UARTDoubleBufHwRx_HandleHT(hw_rx);
    return;
  }

  /* 其次匹配 TX 硬件双缓冲（DBM） */
  STM32UARTDoubleBufHwTx_t *hw_tx = stm32_uart_double_buf_hw_tx_map[id];
  if (hw_tx != NULL)
  {
    STM32UARTDoubleBufHwTx_HandleHT(hw_tx);
  }
}

// ==================== 硬件双缓冲 DMA RX（DBM 模式） ====================

//! 初始化硬件双缓冲 DMA RX 控制块。
//! 只绑定句柄、两块 DMA 缓冲和回调；不触碰 DMA 寄存器。
err_t STM32UARTDoubleBufHwRx_Init(STM32UARTDoubleBufHwRx_t *self,
                                  UART_HandleTypeDef *uart_handle,
                                  BSP_UART_RawData_t dma_buff_0,
                                  BSP_UART_RawData_t dma_buff_1,
                                  STM32UART_RxCallback_t tc_callback,
                                  STM32UART_RxCallback_t ht_callback)
{
  if (self == NULL)
  {
    return PTR_NULL;
  }

  self->id_ = (uart_handle != NULL) ? BSP_UART_get_id(uart_handle->Instance) : BSP_UART_ID_ERROR;
  self->dma_buff_0_ = dma_buff_0;
  self->dma_buff_1_ = dma_buff_1;
  self->uart_handle_ = uart_handle;
  self->tc_callback_ = tc_callback;
  self->ht_callback_ = ht_callback;
  self->last_error_ = PENDING;
  self->ready_buf_ = 0U;
  self->buf_size_ = 0U;

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

  self->buf_size_ = self->dma_buff_0_.size_;

  ASSERT((stm32_uart_double_buf_hw_rx_map[self->id_] == NULL) ||
         (stm32_uart_double_buf_hw_rx_map[self->id_] == self));
  if ((stm32_uart_double_buf_hw_rx_map[self->id_] != NULL) &&
      (stm32_uart_double_buf_hw_rx_map[self->id_] != self))
  {
    self->last_error_ = BUSY;
    return self->last_error_;
  }

  stm32_uart_double_buf_hw_rx_map[self->id_] = self;
  self->last_error_ = OK;
  return self->last_error_;
}

//! 配置 RX DMA 为硬件双缓冲模式（DBM）。
//!
//! 操作步骤：
//!   1. 将 DMA 流设为 Circular 模式（DBM 需要 Circular）
//!   2. 设置 DMA_SxCR 的 DBM 位
//!   3. 分别设置 M0AR 和 M1AR 为两块缓冲地址
//!   4. 使能 TC 和 HT 中断
//!   5. 设置 NDTR 为单块缓冲大小
//!   6. 调用 HAL_UART_Receive_DMA() 启动 UART DMA 接收
//!
//! 注意：HAL 库的 HAL_DMA_Init() 不处理 DBM 相关寄存器，
//!       因此需要直接操作寄存器完成 DBM 配置。
err_t STM32UARTDoubleBufHwRx_SetRxDBM(STM32UARTDoubleBufHwRx_t *self)
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

  ASSERT(self->buf_size_ > 0U);
  if (self->buf_size_ == 0U)
  {
    self->last_error_ = SIZE_ERR;
    return self->last_error_;
  }

  /* ---- 获取 DMA 流句柄 ---- */
  DMA_Stream_TypeDef *dma_stream = (DMA_Stream_TypeDef *)self->uart_handle_->hdmarx->Instance;
  ASSERT(dma_stream != NULL);
  if (dma_stream == NULL)
  {
    self->last_error_ = PTR_NULL;
    return self->last_error_;
  }

  /* ---- 步骤 1：先让 HAL 做基础初始化（外设/存储器地址、数据宽度等） ---- */
  /* 将 DMA 模式设为 CIRCULAR（DBM 需要 Circular） */
  self->uart_handle_->hdmarx->Init.Mode = DMA_CIRCULAR;
  const HAL_StatusTypeDef dma_status = HAL_DMA_Init(self->uart_handle_->hdmarx);
  VERIFY(dma_status == HAL_OK);
  if (dma_status != HAL_OK)
  {
    self->last_error_ = INIT_ERR;
    return self->last_error_;
  }

  /* ---- 步骤 2：手动配置 DBM 相关寄存器 ---- */
  /* 关闭 DMA 流（配置 DBM 前必须禁用） */
  CLEAR_BIT(dma_stream->CR, DMA_SxCR_EN);

  /* 设置 M0AR = 缓冲 0 地址 */
  dma_stream->M0AR = (uint32_t)self->dma_buff_0_.addr_;
  /* 设置 M1AR = 缓冲 1 地址 */
  dma_stream->M1AR = (uint32_t)self->dma_buff_1_.addr_;

  /* 设置 NDTR = 单块缓冲大小 */
  __HAL_DMA_SET_COUNTER(self->uart_handle_->hdmarx, (uint16_t)self->buf_size_);

  /* 使能 DBM 位 */
  SET_BIT(dma_stream->CR, DMA_SxCR_DBM);

  /* 使能 TC 和 HT 中断 */
  SET_BIT(dma_stream->CR, DMA_SxCR_TCIE | DMA_SxCR_HTIE);

  /* 重新使能 DMA 流 */
  SET_BIT(dma_stream->CR, DMA_SxCR_EN);

  /* ---- 步骤 3：启动 UART DMA 接收 ---- */
  /* 使用 HAL_UART_Receive_DMA() 启动，DMA 已在 DBM 模式下运行 */
  const HAL_StatusTypeDef rx_status = HAL_UART_Receive_DMA(
      self->uart_handle_,
      (uint8_t *)self->dma_buff_0_.addr_,
      (uint16_t)self->buf_size_);
  VERIFY(rx_status == HAL_OK);
  if (rx_status != HAL_OK)
  {
    self->last_error_ = INIT_ERR;
    return self->last_error_;
  }

  self->ready_buf_ = 0U;
  self->last_error_ = OK;
  return self->last_error_;
}

//! 获取硬件双缓冲 RX 最近一次错误码。
err_t STM32UARTDoubleBufHwRx_GetLastError(const STM32UARTDoubleBufHwRx_t *self)
{
  if (self == NULL)
  {
    return PTR_NULL;
  }

  return self->last_error_;
}

//! 处理硬件双缓冲 RX TC 中断（缓冲 0 写满）。
//! TC 触发时，DMA 刚写满缓冲 0 并切到缓冲 1，
//! 标记 ready_buf_ = 0 并调用 tc_callback_ 通知上层处理缓冲 0 的数据。
void STM32UARTDoubleBufHwRx_HandleTC(STM32UARTDoubleBufHwRx_t *self)
{
  if (self == NULL)
  {
    return;
  }

  self->ready_buf_ = 0U;

  if (self->tc_callback_ != NULL)
  {
    self->tc_callback_((uint8_t *)self->dma_buff_0_.addr_, self->buf_size_);
  }
}

//! 处理硬件双缓冲 RX HT 中断（缓冲 1 写满）。
//! HT 触发时，DMA 刚写满缓冲 1 并切回缓冲 0，
//! 标记 ready_buf_ = 1 并调用 ht_callback_ 通知上层处理缓冲 1 的数据。
void STM32UARTDoubleBufHwRx_HandleHT(STM32UARTDoubleBufHwRx_t *self)
{
  if (self == NULL)
  {
    return;
  }

  self->ready_buf_ = 1U;

  if (self->ht_callback_ != NULL)
  {
    self->ht_callback_((uint8_t *)self->dma_buff_1_.addr_, self->buf_size_);
  }
}
