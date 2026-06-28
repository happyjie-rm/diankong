#pragma once

#ifdef __cplusplus
extern "C" {
#endif

//! FreeRTOS 中 DR16 遥控器任务入口。
//! CubeMX 创建同名线程后调用该函数，内部初始化 USART2 上的 DR16 接收逻辑。
void dr16_task(void *argument);

#ifdef __cplusplus
}
#endif
