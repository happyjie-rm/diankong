#pragma once

#include "comp_cmd.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 完成 CAN 对象绑定、过滤器配置、设备注册与总线启动。
 *
 * 在 RTOS 调度器启动前调用一次即可；内部任一步骤失败即进入 Error_Handler()。
 */
void can_device_Init(void);

/**
 * @brief 注册所有 CAN 设备及其 RX 回调。
 *
 * 须在 STM32CAN_Init() 之后、CAN_StartAll() 之前调用，确保任务之间不会
 * 竞争 STM32CAN_SubscribeRx() 与设备对象初始化。
 * @return OK 成功；否则为失败步骤返回的错误码。
 */
err_t CAN_RegisterAllDevices(void);

/**
 * @brief 启动全部 CAN 总线。
 *
 * 所有设备注册完成后才允许调用；调用后禁止新增 RX 订阅者。
 * @return OK 成功；否则为失败步骤返回的错误码。
 */
err_t CAN_StartAll(void);

#ifdef __cplusplus
}
#endif
