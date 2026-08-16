#include "chassis_task.h"

#include "chassis_control.h"

#include "FreeRTOS.h"
#include "task.h"

#include "dr16.h"
#include "main.h"

extern DR16_t *dr16;

volatile err_t chassis_status = PENDING;

/**
 * @brief 底盘任务
 * @details 在任务循环中完成快照读取、通用输入映射与控制周期调用。
 *
 * 左拨杆 sw_l 模式（仅影响 enable/yaw，平移仍跟左摇杆）：
 * - CMD_SW_UP：安全失能（enable=false）
 * - CMD_SW_MID：正常控制，yaw=0
 * - CMD_SW_DOWN：正常控制，yaw=CHASSIS_SWITCH_DOWN_YAW（归一化）
 * - 其它/非法：失能
 */
void chassis_task(void *argument)
{
  (void)argument;

  for (;;)
  {
    cmd_rc_t command = {0};
    bool remote_online = false;

    // if (dr16 != NULL)
    // {
    //   const err_t snapshot_result =
    //       DR16_GetSnapshot(dr16, &command, &remote_online);
    //   if (snapshot_result != OK)
    //   {
    //       remote_online = false;
    //   }
    // }

    /* 默认安全：仅 MID/DOWN 使能；yaw 仅在对应档位覆盖 */
    bool enable = false;
    float yaw = 0.0f;
    if (remote_online)
    {
      switch (command.sw_l)
      {
        case CMD_SW_UP:
          enable = false;
          yaw = 0.0f;
          break;
        case CMD_SW_MID:
          enable = true;
          yaw = 0.0f;
          break;
        case CMD_SW_DOWN:
          enable = true;
          yaw = CHASSIS_SWITCH_DOWN_YAW;
          break;
        default:
          enable = false;
          yaw = 0.0f;
          break;
      }
    }

    const chassis_control_input_t input = {
        .forward = -command.ch.l.y,
        .lateral = -command.ch.l.x,
        .yaw = yaw,
        .enable = enable,
        .source_online = remote_online,
    };
    chassis_status = chassis_control_step(&input, HAL_GetTick());

    vTaskDelay(pdMS_TO_TICKS(2U));
  }
}
