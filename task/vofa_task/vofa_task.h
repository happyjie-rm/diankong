#pragma once

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief FreeRTOS 中 VOFA 任务入口。
 *
 * CubeMX 在 Core/Src/freertos.c 中生成了同名弱函数 vofa_task；本函数提供强定义，
 * 由系统创建的 vofa 线程调用，用于启动 USART3 上的 VOFA 收发逻辑。
 */
void vofa_task(void *argument);

/**
 * @brief 读取 VOFA 接收到的 speed 参数。
 *
 * 上位机发送 `speed=100!` 且解析成功后，本函数会返回 true，并把 100.0f 写入
 * speed 指向的位置。该函数是 Vofa_GetSpeed() 的 C 包装接口。
 *
 * @param speed 输出参数地址，不能为 NULL。
 * @return true 表示 speed 已经至少收到过一次；false 表示参数为空或尚未收到。
 */
bool VofaTask_GetSpeed(float *speed);

/**
 * @brief 读取 VOFA 接收到的 angle 参数。
 *
 * 上位机发送 `angle=30!` 且解析成功后，本函数会返回 true，并把 30.0f 写入
 * angle 指向的位置。该函数是 Vofa_GetAngle() 的 C 包装接口。
 *
 * @param angle 输出参数地址，不能为 NULL。
 * @return true 表示 angle 已经至少收到过一次；false 表示参数为空或尚未收到。
 */
bool VofaTask_GetAngle(float *angle);

/**
 * @brief 按名称读取 VOFA 参数表中的参数。
 *
 * 当前模块参数表包含 `speed` 和 `angle`。该函数是 Vofa_GetParameter() 的
 * C 包装接口，适合 C/C++ 任务层代码按字符串读取参数。
 *
 * @param name 参数名，例如 "speed" 或 "angle"。
 * @param value 输出参数地址，不能为 NULL。
 * @return true 表示参数存在且已经收到过；false 表示参数不存在、参数为空或尚未收到。
 */
bool VofaTask_GetParameter(const char *name, float *value);

/**
 * @brief VOFA ASCII 命令接收扩展点。
 *
 * Vofa 解析到合法的 `name=value!` 命令后，会先更新模块内部参数表，再调用此扩展点。
 * 默认实现为空，业务代码可在其他编译单元中提供同名强定义来接收额外参数，不需要
 * 修改 Vofa 驱动或任务循环。
 *
 * @param name 命令名，生命周期仅在回调执行期间有效。
 * @param value 解析后的有限浮点值。
 */
void VofaTask_OnCommand(const char *name, float value);

#ifdef __cplusplus
}
#endif
