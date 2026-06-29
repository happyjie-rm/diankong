#pragma once

#ifdef __cplusplus
extern "C" {
#endif

//! FreeRTOS 中 I6x 遥控器任务入口。
//! CubeMX 创建 i6x_task 线程后调用该函数，内部初始化 USART1 上的 iBus 接收逻辑。
void I6X_task(void *argument);

#ifdef __cplusplus
}
#endif

