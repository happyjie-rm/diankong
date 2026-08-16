# bsp_uart

`bsp_uart` 把 STM32 UART 封装成两条清晰的路径：RX 用 `ReceiveToIdle + 循环 DMA` 切片，TX 用 `双软件缓冲 + 普通 DMA` 续发。这样上层可以把协议解析放到任务里，而不是堆在中断里。

## 这份 BSP 负责什么

- RX：`STM32UART_Init` 绑定句柄和接收缓冲，`STM32UART_SetRxDMA` 启动 DMA，`HAL_UARTEx_RxEventCallback` 自动把新字节切成片段。
- TX：`STM32UARTDoubleBufTx_Init` 绑定两块发送缓冲，`STM32UARTDoubleBufTx_Write` 写入待发送数据，`HAL_UART_TxCpltCallback` 负责推进下一段。
- 跳转入口：下面每个函数名都直接链接到 `.c/.h` 的位置。

## 数据结构

| 名称                             | 作用             | 跳转                              |
| -------------------------------- | ---------------- | --------------------------------- |
| `BSP_UART_t`                     | BSP 逻辑串口编号 | [bsp_uart.h:13](./bsp_uart.h#L13) |
| `STM32UART_RxCallback_t`         | RX 数据片段回调  | [bsp_uart.h:40](./bsp_uart.h#L40) |
| `STM32UART_TxCompleteCallback_t` | TX 完成回调      | [bsp_uart.h:44](./bsp_uart.h#L44) |
| `BSP_UART_RawData_t`             | 原始缓冲描述     | [bsp_uart.h:48](./bsp_uart.h#L48) |
| `STM32UART_t`                    | 环形 RX 控制块   | [bsp_uart.h:56](./bsp_uart.h#L56) |
| `STM32UARTDoubleBufTx_t`         | 双缓冲 TX 控制块 | [bsp_uart.h:69](./bsp_uart.h#L69) |
| `STM32UARTDoubleBufHwTx_t`       | 硬件双缓冲 TX 控制块（DBM） | [bsp_uart.h:82](./bsp_uart.h#L82) |
| `STM32UARTDoubleBufHwRx_t`       | 硬件双缓冲 RX 控制块（DBM） | [bsp_uart.h:97](./bsp_uart.h#L97) |

## 常用用法

### 1. 先把 UART 句柄映射成 BSP 编号

`BSP_UART_get_id()` 会把 `USART1/2/3...` 反查成 BSP 内部编号。新增外设时，先看这个函数的映射表。

### 2. 接收原始串口数据

RX 采用 `环形 DMA + 空闲中断`。`HAL_UARTEx_RxEventCallback()` 不会把整帧协议直接交给你，而是把 DMA 中新增的字节切成一段一段回调给 `STM32UART_RxCallback_t`。

```c
static uint8_t uart1_rx_buf[128];
static STM32UART_t uart1_rx;

static void uart1_rx_cb(uint8_t *data, size_t size)
{
    // 在任务里做协议解析
}

void app_uart1_init(void)
{
    BSP_UART_RawData_t rx = {uart1_rx_buf, sizeof(uart1_rx_buf)};
    STM32UART_Init(&uart1_rx, &huart1, rx, uart1_rx_cb);
    STM32UART_SetRxDMA(&uart1_rx);
}
```

注意点：

- `huart` 必须已经在 CubeMX 里配置成 RX 可用。
- `dma_buff_rx_` 指向的缓冲区必须在整个接收期间有效。
- 回调里最好只做轻量处理，长耗时逻辑放到任务上下文。

### 3. 双缓冲发送

TX 不是硬件双缓冲，而是两块软件缓冲轮流喂给 DMA。`STM32UARTDoubleBufTx_Write()` 会把数据拷贝到当前可用缓冲；DMA 忙时，另一块缓冲会作为待续发数据。

```c
static uint8_t uart1_tx0[128];
static uint8_t uart1_tx1[128];
static STM32UARTDoubleBufTx_t uart1_tx;

static void uart1_tx_done(void)
{
    // 一帧发送完成后的通知
}

void app_uart1_tx_init(void)
{
    BSP_UART_RawData_t tx0 = {uart1_tx0, sizeof(uart1_tx0)};
    BSP_UART_RawData_t tx1 = {uart1_tx1, sizeof(uart1_tx1)};

    STM32UARTDoubleBufTx_Init(&uart1_tx, &huart1, tx0, tx1, uart1_tx_done);
    STM32UARTDoubleBufTx_SetTxDMA(&uart1_tx);
}
```

注意点：

- 两块 TX 缓冲大小必须相同。
- 单次写入长度不能超过单块缓冲容量。
- 连续写太快时，pending 数据会被后一次写入覆盖。

### 4. 硬件双缓冲接收（DBM 模式）

RX DBM 利用 DMA 硬件双缓冲特性，DMA 在两块缓冲之间自动切换接收。TC 中断表示缓冲 0 写满，HT 中断表示缓冲 1 写满。上层在回调中直接处理已满缓冲，无需软件拷贝。

```c
static uint8_t uart1_rx_buf0[128];
static uint8_t uart1_rx_buf1[128];
static STM32UARTDoubleBufHwRx_t uart1_rx_dbm;

static void uart1_rx_buf0_ready(uint8_t *data, size_t size)
{
    // 缓冲 0 已写满，处理数据（size 固定为单块缓冲大小）
}

static void uart1_rx_buf1_ready(uint8_t *data, size_t size)
{
    // 缓冲 1 已写满，处理数据
}

void app_uart1_rx_dbm_init(void)
{
    BSP_UART_RawData_t rx0 = {uart1_rx_buf0, sizeof(uart1_rx_buf0)};
    BSP_UART_RawData_t rx1 = {uart1_rx_buf1, sizeof(uart1_rx_buf1)};

    STM32UARTDoubleBufHwRx_Init(&uart1_rx_dbm, &huart1,
                                rx0, rx1,
                                uart1_rx_buf0_ready,   // tc_callback = 缓冲 0 满
                                uart1_rx_buf1_ready);  // ht_callback = 缓冲 1 满
    STM32UARTDoubleBufHwRx_SetRxDBM(&uart1_rx_dbm);
}
```

注意点：

- 两块 RX 缓冲大小必须相同。
- 回调运行在中断上下文中，不应执行长时间阻塞操作。
- 回调的 size 参数固定等于单块缓冲大小。
- RX DBM 和单缓冲 RX（STM32UART_t）不能同时用于同一个串口。

## 接口速查

### 反查与基础处理

| 函数                     | 作用                            | 跳转                                                  |
| ------------------------ | ------------------------------- | ----------------------------------------------------- |
| `BSP_UART_get_id`        | `USART_TypeDef *` 反查 BSP 编号 | [实现](./bsp_uart.c#L19) / [声明](./bsp_uart.h#L92)   |
| `STM32UART_HandleRxData` | 把一段 RX 数据交给用户回调      | [实现](./bsp_uart.c#L228) / [声明](./bsp_uart.h#L116) |
| `STM32UART_GetLastError` | 读取单缓冲 RX 最近错误码        | [实现](./bsp_uart.c#L215) / [声明](./bsp_uart.h#L112) |

### 单缓冲环形 RX

| 函数                      | 作用                        | 跳转                                                  |
| ------------------------- | --------------------------- | ----------------------------------------------------- |
| `STM32UART_Init`          | 绑定 RX 控制块              | [实现](./bsp_uart.c#L77) / [声明](./bsp_uart.h#L97)   |
| `STM32UART_SetRxDMA`      | 启动 `ReceiveToIdle` RX DMA | [实现](./bsp_uart.c#L137) / [声明](./bsp_uart.h#L104) |
| `STM32UART_SetRxCallback` | 更新 RX 回调                | [实现](./bsp_uart.c#L204) / [声明](./bsp_uart.h#L108) |

### 双缓冲 TX

| 函数                                         | 作用                     | 跳转                                                  |
| -------------------------------------------- | ------------------------ | ----------------------------------------------------- |
| `STM32UARTDoubleBufTx_Init`                  | 绑定双缓冲 TX 控制块     | [实现](./bsp_uart.c#L353) / [声明](./bsp_uart.h#L121) |
| `STM32UARTDoubleBufTx_SetTxDMA`              | 配置 TX DMA              | [实现](./bsp_uart.c#L428) / [声明](./bsp_uart.h#L129) |
| `STM32UARTDoubleBufTx_Write`                 | 写入待发送数据           | [实现](./bsp_uart.c#L475) / [声明](./bsp_uart.h#L133) |
| `STM32UARTDoubleBufTx_Flush`                 | 提交 pending 数据到 DMA  | [实现](./bsp_uart.c#L522) / [声明](./bsp_uart.h#L139) |
| `STM32UARTDoubleBufTx_SetTxCompleteCallback` | 更新 TX 完成回调         | [实现](./bsp_uart.c#L563) / [声明](./bsp_uart.h#L142) |
| `STM32UARTDoubleBufTx_GetLastError`          | 读取双缓冲 TX 最近错误码 | [实现](./bsp_uart.c#L576) / [声明](./bsp_uart.h#L147) |
| `STM32UARTDoubleBufTx_HandleTxComplete`      | 处理 TX DMA 完成事件     | [实现](./bsp_uart.c#L588) / [声明](./bsp_uart.h#L151) |

### 硬件双缓冲 TX（DBM 模式）

| 函数                                      | 作用                     | 跳转                                                  |
| ----------------------------------------- | ------------------------ | ----------------------------------------------------- |
| `STM32UARTDoubleBufHwTx_Init`             | 绑定 DBM TX 控制块       | [实现](./bsp_uart.c#L604) / [声明](./bsp_uart.h#L157) |
| `STM32UARTDoubleBufHwTx_SetTxDBM`         | 配置 TX DMA 为 DBM 模式  | [实现](./bsp_uart.c#L680) / [声明](./bsp_uart.h#L165) |
| `STM32UARTDoubleBufHwTx_Write`            | 写入数据到空闲缓冲       | [实现](./bsp_uart.c#L770) / [声明](./bsp_uart.h#L171) |
| `STM32UARTDoubleBufHwTx_GetLastError`     | 读取最近错误码           | [实现](./bsp_uart.c#L848) / [声明](./bsp_uart.h#L177) |
| `STM32UARTDoubleBufHwTx_HandleTC`         | 处理 TC 中断             | [实现](./bsp_uart.c#L860) / [声明](./bsp_uart.h#L180) |
| `STM32UARTDoubleBufHwTx_HandleHT`         | 处理 HT 中断             | [实现](./bsp_uart.c#L882) / [声明](./bsp_uart.h#L184) |

### 硬件双缓冲 RX（DBM 模式）

| 函数                                      | 作用                     | 跳转                                                  |
| ----------------------------------------- | ------------------------ | ----------------------------------------------------- |
| `STM32UARTDoubleBufHwRx_Init`             | 绑定 DBM RX 控制块       | [实现](./bsp_uart.c#L918) / [声明](./bsp_uart.h#L189) |
| `STM32UARTDoubleBufHwRx_SetRxDBM`         | 配置 RX DMA 为 DBM 模式  | [实现](./bsp_uart.c#L1000) / [声明](./bsp_uart.h#L197) |
| `STM32UARTDoubleBufHwRx_GetLastError`     | 读取最近错误码           | [实现](./bsp_uart.c#L1108) / [声明](./bsp_uart.h#L201) |
| `STM32UARTDoubleBufHwRx_HandleTC`         | 处理 TC 中断（缓冲 0 满）| [实现](./bsp_uart.c#L1120) / [声明](./bsp_uart.h#L204) |
| `STM32UARTDoubleBufHwRx_HandleHT`         | 处理 HT 中断（缓冲 1 满）| [实现](./bsp_uart.c#L1140) / [声明](./bsp_uart.h#L208) |

### HAL 回调入口

| 函数                         | 作用                | 跳转                      |
| ---------------------------- | ------------------- | ------------------------- |
| `HAL_UARTEx_RxEventCallback` | RX 空闲事件分发入口 | [实现](./bsp_uart.c#L613) |
| `HAL_UART_TxCpltCallback`    | TX/RX DMA 完成分发  | [实现](./bsp_uart.c#L636) |
| `HAL_UART_TxHalfCpltCallback`| TX/RX DMA 半完成分发| [实现](./bsp_uart.c#L665) |

## 一句话记忆

- RX 看 `ReceiveToIdle + 循环 DMA` 或 `DBM 硬件双缓冲`。
- TX 看 `双缓冲 + DMA_NORMAL` 或 `DBM 硬件双缓冲`。
- 真正给上层的，是切好的数据片段和发送完成回调。
