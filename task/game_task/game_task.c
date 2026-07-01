#include "game_task.h"

#include "FreeRTOS.h"
#include "comp_utils.h"
#include "game.h"
#include "task.h"
#include "usart.h"

#define GAME_TASK_UPDATE_TIMEOUT_MS (10u) //! 裁判系统接收任务周期

//! 全局裁判系统对象指针，供其他业务模块读取裁判/图传状态。
Game_t *game = NULL;

//! 裁判系统 FreeRTOS 任务：初始化 UART3 接收，并周期性解析裁判系统数据帧。
void game_task(void *argument)
{
  RM_UNUSED(argument);

  static Game_t game_instance; // 静态存储，保证任务整个生命周期内有效。
  remote_control_data_init();

  err_t status = Game_Init(&game_instance, &huart3, &custom_robot_data);
  game = &game_instance;

  game->thread_alert = xTaskGetCurrentTaskHandle();

  if (status == OK) {
    status = Game_Start(game);
  }
  ASSERT(status == OK);
  if (status != OK) {
    vTaskDelete(NULL);
    return;
  }

  for (;;) {
    Game_Update(game, GAME_TASK_UPDATE_TIMEOUT_MS); // 超时无新数据时保持最近一次合法解析结果。
  }
}
