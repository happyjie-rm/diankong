/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for dr16 */
osThreadId_t dr16Handle;
const osThreadAttr_t dr16_attributes = {
  .name = "dr16",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for vt13 */
osThreadId_t vt13Handle;
const osThreadAttr_t vt13_attributes = {
  .name = "vt13",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for vofa */
osThreadId_t vofaHandle;
const osThreadAttr_t vofa_attributes = {
  .name = "vofa",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for imu_can */
osThreadId_t imu_canHandle;
const osThreadAttr_t imu_can_attributes = {
  .name = "imu_can",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for lx824 */
osThreadId_t lx824Handle;
const osThreadAttr_t lx824_attributes = {
  .name = "lx824",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for I6X */
osThreadId_t I6XHandle;
const osThreadAttr_t I6X_attributes = {
  .name = "I6X",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for imu_485 */
osThreadId_t imu_485Handle;
const osThreadAttr_t imu_485_attributes = {
  .name = "imu_485",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for gimbal */
osThreadId_t gimbalHandle;
const osThreadAttr_t gimbal_attributes = {
  .name = "gimbal",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for chassis */
osThreadId_t chassisHandle;
const osThreadAttr_t chassis_attributes = {
  .name = "chassis",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for joint */
osThreadId_t jointHandle;
const osThreadAttr_t joint_attributes = {
  .name = "joint",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void dr16_task(void *argument);
void vt13_task(void *argument);
void vofa_task(void *argument);
void imu_can_task(void *argument);
void lx824_task(void *argument);
void I6X_task(void *argument);
void imu_485_task(void *argument);
void gimbal_task(void *argument);
void chassis_task(void *argument);
void joint_task(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void configureTimerForRunTimeStats(void);
unsigned long getRunTimeCounterValue(void);
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName);
void vApplicationMallocFailedHook(void);

/* USER CODE BEGIN 1 */
/* Functions needed when configGENERATE_RUN_TIME_STATS is on */
__weak void configureTimerForRunTimeStats(void)
{

}

__weak unsigned long getRunTimeCounterValue(void)
{
return 0;
}
/* USER CODE END 1 */

/* USER CODE BEGIN 4 */
// 栈溢出钩子函数
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName)
{
  (void)xTask;
  (void)pcTaskName;
  taskDISABLE_INTERRUPTS();
  for (;;) {
  }
}
/* USER CODE END 4 */

/* USER CODE BEGIN 5 */
// 内存分配失败钩子函数
void vApplicationMallocFailedHook(void)
{
  taskDISABLE_INTERRUPTS();
  for (;;)
  {
  }
}
/* USER CODE END 5 */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of dr16 */
  dr16Handle = osThreadNew(dr16_task, NULL, &dr16_attributes);

  /* creation of vt13 */
  vt13Handle = osThreadNew(vt13_task, NULL, &vt13_attributes);

  /* creation of vofa */
  vofaHandle = osThreadNew(vofa_task, NULL, &vofa_attributes);

  /* creation of imu_can */
  imu_canHandle = osThreadNew(imu_can_task, NULL, &imu_can_attributes);

  /* creation of lx824 */
  lx824Handle = osThreadNew(lx824_task, NULL, &lx824_attributes);

  /* creation of I6X */
  I6XHandle = osThreadNew(I6X_task, NULL, &I6X_attributes);

  /* creation of imu_485 */
  imu_485Handle = osThreadNew(imu_485_task, NULL, &imu_485_attributes);

  /* creation of gimbal */
  gimbalHandle = osThreadNew(gimbal_task, NULL, &gimbal_attributes);

  /* creation of chassis */
  chassisHandle = osThreadNew(chassis_task, NULL, &chassis_attributes);

  /* creation of joint */
  jointHandle = osThreadNew(joint_task, NULL, &joint_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_dr16_task */
/**
* @brief Function implementing the dr16 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_dr16_task */
__weak void dr16_task(void *argument)
{
  /* USER CODE BEGIN dr16_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END dr16_task */
}

/* USER CODE BEGIN Header_vt13_task */
/**
* @brief Function implementing the vt13 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_vt13_task */
__weak void vt13_task(void *argument)
{
  /* USER CODE BEGIN vt13_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END vt13_task */
}

/* USER CODE BEGIN Header_vofa_task */
/**
* @brief Function implementing the vofa thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_vofa_task */
__weak void vofa_task(void *argument)
{
  /* USER CODE BEGIN vofa_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END vofa_task */
}

/* USER CODE BEGIN Header_imu_can_task */
/**
* @brief Function implementing the imu_can thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_imu_can_task */
__weak void imu_can_task(void *argument)
{
  /* USER CODE BEGIN imu_can_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END imu_can_task */
}

/* USER CODE BEGIN Header_lx824_task */
/**
* @brief Function implementing the lx824 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_lx824_task */
__weak void lx824_task(void *argument)
{
  /* USER CODE BEGIN lx824_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END lx824_task */
}

/* USER CODE BEGIN Header_I6X_task */
/**
* @brief Function implementing the I6X thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_I6X_task */
__weak void I6X_task(void *argument)
{
  /* USER CODE BEGIN I6X_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END I6X_task */
}

/* USER CODE BEGIN Header_imu_485_task */
/**
* @brief Function implementing the imu_485 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_imu_485_task */
__weak void imu_485_task(void *argument)
{
  /* USER CODE BEGIN imu_485_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END imu_485_task */
}

/* USER CODE BEGIN Header_gimbal_task */
/**
* @brief Function implementing the gimbal thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_gimbal_task */
__weak void gimbal_task(void *argument)
{
  /* USER CODE BEGIN gimbal_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END gimbal_task */
}

/* USER CODE BEGIN Header_chassis_task */
/**
* @brief Function implementing the chassis thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_chassis_task */
__weak void chassis_task(void *argument)
{
  /* USER CODE BEGIN chassis_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END chassis_task */
}

/* USER CODE BEGIN Header_joint_task */
/**
* @brief Function implementing the joint thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_joint_task */
__weak void joint_task(void *argument)
{
  /* USER CODE BEGIN joint_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END joint_task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

