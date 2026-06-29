#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief FreeRTOS 中 LX824 舵机任务入口。
 *
 * CubeMX 在 Core/Src/freertos.c 中生成了同名弱函数 lx824_task；本函数提供强定义，
 * 由系统创建的 lx824_task 线程调用，周期性读取总线上舵机的 ID。
 */
void lx824_task(void *argument);

/**
 * @brief 读取最近一次成功获取到的舵机 ID。
 *
 * 任务循环通过 LX824 协议的读 ID 指令（Cmd 14）轮询舵机，成功后缓存结果。
 * 该函数供其他任务/业务代码按需读取，是 LX824_IdRead() 的 C 包装接口。
 *
 * @param servo_id 输出参数地址，不能为 NULL。
 * @return true 表示已经至少成功读到过一次舵机 ID；false 表示参数为空或尚未读到。
 */
// bool LX824Task_GetServoId(uint8_t *servo_id);

#ifdef __cplusplus
}
#endif
