#pragma once

#ifdef __cplusplus
extern "C" {
#endif

//! FreeRTOS 中裁判系统 game 任务入口。
//! CubeMX 创建线程后调用该函数，内部初始化 UART4 上的裁判/图传接收逻辑。
void game_task(void *argument);

#ifdef __cplusplus
}
#endif
