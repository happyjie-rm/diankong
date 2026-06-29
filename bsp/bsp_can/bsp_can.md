# bsp_can

`bsp_can` 既提供对象式 `STM32CAN_t` 接口，也保留了一套旧接口兼容层。新代码优先走对象式：`STM32CAN_Init` -> `STM32CAN_Start` -> `STM32CAN_ConfigFilter` -> `STM32CAN_Send / STM32CAN_SendDjiCurrent`。接收时 HAL FIFO0/FIFO1 中断会把帧填成 `BSP_CAN_Frame_t`，先给对象回调，没注册再回退到旧 weak callback。

## 这份 BSP 负责什么

- 统一 CAN 句柄映射：`BSP_CAN_get_id`
- 统一 RX 帧结构：`BSP_CAN_Frame_t`
- 新旧两套接收入口共存
- 兼容旧项目的 `CAN_Init`、`canx_receive`、`dj_CAN_Send_Data` 等接口

## 常量与数据结构

| 名称                              | 作用                        | 跳转                            |
| --------------------------------- | --------------------------- | ------------------------------- |
| `CAN_DATA_SIZE`                   | CAN 数据区长度，固定 8 字节 | [bsp_can.h:13](./bsp_can.h#L13) |
| `CAN_FILTER(x)`                   | 过滤器编号打包宏            | [bsp_can.h:17](./bsp_can.h#L17) |
| `CAN_FIFO_0 / CAN_FIFO_1`         | FIFO 选择                   | [bsp_can.h:20](./bsp_can.h#L20) |
| `CAN_STDID / CAN_EXTID`           | 标准帧 / 扩展帧             | [bsp_can.h:24](./bsp_can.h#L24) |
| `CAN_DATA_TYPE / CAN_REMOTE_TYPE` | 数据帧 / 远程帧             | [bsp_can.h:28](./bsp_can.h#L28) |
| `BSP_CAN_t`                       | BSP 逻辑 CAN 编号           | [bsp_can.h:33](./bsp_can.h#L33) |
| `BSP_CAN_Frame_t`                 | RX 帧统一描述               | [bsp_can.h:48](./bsp_can.h#L48) |
| `STM32CAN_RxCallback_t`           | 对象式 RX 回调              | [bsp_can.h:62](./bsp_can.h#L62) |
| `STM32CAN_t`                      | CAN BSP 控制块              | [bsp_can.h:67](./bsp_can.h#L67) |

 [bsp_can.h:77](./bsp_can.c#133)
## 常用用法

### 1. 先初始化再启动

```c
static STM32CAN_t can1;

static void can1_rx_cb(STM32CAN_t *self, const BSP_CAN_Frame_t *frame)
{
    // frame->id_ / frame->size_ / frame->data_
}

void app_can1_init(void)
{
    STM32CAN_Init(&can1, &hcan1, can1_rx_cb);
    STM32CAN_ConfigFilter(&can1, &filter);
    STM32CAN_Start(&can1);
}
```

注意点：

- `STM32CAN_Init` 只负责绑定句柄和回调，不会自动启动 CAN。
- `STM32CAN_Start` 会开启 FIFO0/FIFO1 消息挂起中断。
- 过滤器仍由 CubeMX/HAL 配置决定，`STM32CAN_ConfigFilter` 只是把 `CAN_FilterTypeDef` 送给 HAL。

### 2. 发送标准帧 / DJI 电流帧

```c
uint8_t data[8] = {0x01, 0x02, 0x03, 0x04};
STM32CAN_Send(&can1, 0x123, data, 4);

int16_t current[4] = {1000, 1000, 1000, 1000};
STM32CAN_SendDjiCurrent(&can1, 0x200, current);
```

注意点：

- 标准帧 ID 不能超过 `0x7FF`。
- 单帧数据长度不能超过 8 字节。
- `STM32CAN_SendDjiCurrent` 按高字节在前打包 4 路电流。

### 3. 接收帧

对象式回调优先级最高。HAL 收到帧后会先填成 `BSP_CAN_Frame_t`，如果对象注册了回调，就走 `STM32CAN_RxCallback_t`；否则回退到 `dm_can1_rx_callback`、`dj_motor_can1_rx_callback` 等旧接口。

```c
static void can1_rx_cb(STM32CAN_t *self, const BSP_CAN_Frame_t *frame)
{
    if (frame->size_ == 8U)
    {
        // 解析标准 8 字节报文
    }
}
```

### 4. 旧接口兼容层

这些接口主要给旧工程和历史代码使用：

- `CAN_Init`
- `CAN_Filter_Mask_Config_16bit`
- `CAN_Filter_Mask_Config_32bit`
- `dj_CAN_Send_Data`
- `CAN_Send_Data_X8`
- `canx_receive`
- `dm_can1_rx_callback`
- `dm_can2_rx_callback`
- `dj_motor_can1_rx_callback`
- `dj_motor_can2_rx_callback`
- `dm_can_send_data`

如果你在新代码里写功能，优先用对象式接口；兼容层更适合保留既有业务，不建议再扩展新逻辑。

## 接口速查

### 反查与对象式接口

| 函数                      | 作用                          | 跳转                                                |
| ------------------------- | ----------------------------- | --------------------------------------------------- |
| `BSP_CAN_get_id`          | `CAN_TypeDef *` 反查 BSP 编号 | [实现](./bsp_can.c#L14) / [声明](./bsp_can.h#L92)   |
| `STM32CAN_Init`           | 绑定 CAN 控制块               | [实现](./bsp_can.c#L274) / [声明](./bsp_can.h#L95)  |
| `STM32CAN_Start`          | 启动 CAN 并开通知             | [实现](./bsp_can.c#L317) / [声明](./bsp_can.h#L100) |
| `STM32CAN_ConfigFilter`   | 配置 HAL 滤波器               | [实现](./bsp_can.c#L336) / [声明](./bsp_can.h#L103) |
| `STM32CAN_Send`           | 发送标准帧                    | [实现](./bsp_can.c#L366) / [声明](./bsp_can.h#L107) |
| `STM32CAN_SendDjiCurrent` | 发送 DJI 电流帧               | [实现](./bsp_can.c#L382) / [声明](./bsp_can.h#L113) |
| `STM32CAN_SetRxCallback`  | 更新对象式 RX 回调            | [实现](./bsp_can.c#L414) / [声明](./bsp_can.h#L118) |
| `STM32CAN_GetLastError`   | 读取最近错误码                | [实现](./bsp_can.c#L426) / [声明](./bsp_can.h#L122) |

### 兼容旧接口

| 函数                           | 作用                    | 跳转                                                |
| ------------------------------ | ----------------------- | --------------------------------------------------- |
| `CAN_Init`                     | 启动 CAN 并开 FIFO 中断 | [实现](./bsp_can.c#L439) / [声明](./bsp_can.h#L125) |
| `CAN_Filter_Mask_Config_16bit` | 16 位掩码滤波器         | [实现](./bsp_can.c#L446) / [声明](./bsp_can.h#L128) |
| `CAN_Filter_Mask_Config_32bit` | 32 位掩码滤波器         | [实现](./bsp_can.c#L465) / [声明](./bsp_can.h#L133) |
| `dj_CAN_Send_Data`             | 发送 DJI 四路电流控制帧 | [实现](./bsp_can.c#L483) / [声明](./bsp_can.h#L137) |
| `CAN_Send_Data_X8`             | 发送 8 字节全零标准帧   | [实现](./bsp_can.c#L497) / [声明](./bsp_can.h#L142) |
| `canx_receive`                 | 从指定 FIFO 读一帧      | [实现](./bsp_can.c#L505) / [声明](./bsp_can.h#L145) |
| `dm_can1_rx_callback`          | CAN1 达妙电机弱回调     | [实现](./bsp_can.c#L530) / [声明](./bsp_can.h#L149) |
| `dm_can2_rx_callback`          | CAN2 达妙电机弱回调     | [实现](./bsp_can.c#L539) / [声明](./bsp_can.h#L151) |
| `dj_motor_can1_rx_callback`    | CAN1 DJI 电机弱回调     | [实现](./bsp_can.c#L548) / [声明](./bsp_can.h#L153) |
| `dj_motor_can2_rx_callback`    | CAN2 DJI 电机弱回调     | [实现](./bsp_can.c#L557) / [声明](./bsp_can.h#L155) |
| `dm_can_send_data`             | 通用 CAN 标准帧发送接口 | [实现](./bsp_can.c#L566) / [声明](./bsp_can.h#L158) |

### HAL 回调入口

| 函数                                | 作用              | 跳转                     |
| ----------------------------------- | ----------------- | ------------------------ |
| `HAL_CAN_RxFifo0MsgPendingCallback` | FIFO0 RX 分发入口 | [实现](./bsp_can.c#L645) |
| `HAL_CAN_RxFifo1MsgPendingCallback` | FIFO1 RX 分发入口 | [实现](./bsp_can.c#L651) |

## 一句话记忆

- 新代码优先走对象式 `STM32CAN_t`。
- 旧代码还能用兼容层，但别再把新逻辑堆进去。
- RX 帧先统一成 `BSP_CAN_Frame_t`，再决定是对象回调还是旧 weak 回调。
