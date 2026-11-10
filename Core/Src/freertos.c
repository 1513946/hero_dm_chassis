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
#include "FreeRTOS.h"
#include "cmsis_os2.h"

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
/* Definitions for task */
osThreadId_t taskHandle;
const osThreadAttr_t task_attributes = {
  .name = "task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for crg_test_task */
osThreadId_t crg_test_taskHandle;
const osThreadAttr_t crg_test_task_attributes = {
  .name = "crg_test_task",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for imu_task */
osThreadId_t imu_taskHandle;
const osThreadAttr_t imu_task_attributes = {
  .name = "imu_task",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for gimbal_task */
osThreadId_t gimbal_taskHandle;
const osThreadAttr_t gimbal_task_attributes = {
  .name = "gimbal_task",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for shoot_task */
osThreadId_t shoot_taskHandle;
const osThreadAttr_t shoot_task_attributes = {
  .name = "shoot_task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void Crg_test_Task(void *argument);
void Imu_Task(void *argument);
void Gimbal_Task(void *argument);
void Shoot_Task(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

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
  /* creation of task */
  taskHandle = osThreadNew(StartDefaultTask, NULL, &task_attributes);

  /* creation of crg_test_task */
  crg_test_taskHandle = osThreadNew(Crg_test_Task, NULL, &crg_test_task_attributes);

  /* creation of imu_task */
  imu_taskHandle = osThreadNew(Imu_Task, NULL, &imu_task_attributes);

  /* creation of gimbal_task */
  gimbal_taskHandle = osThreadNew(Gimbal_Task, NULL, &gimbal_task_attributes);

  /* creation of shoot_task */
  shoot_taskHandle = osThreadNew(Shoot_Task, NULL, &shoot_task_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the task thread.
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

/* USER CODE BEGIN Header_Crg_test_Task */
/**
* @brief Function implementing the crg_test_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Crg_test_Task */
__weak void Crg_test_Task(void *argument)
{
  /* USER CODE BEGIN Crg_test_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Crg_test_Task */
}

/* USER CODE BEGIN Header_Imu_Task */
/**
* @brief Function implementing the imu_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Imu_Task */
__weak void Imu_Task(void *argument)
{
  /* USER CODE BEGIN Imu_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Imu_Task */
}

/* USER CODE BEGIN Header_Gimbal_Task */
/**
* @brief Function implementing the gimbal_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Gimbal_Task */
__weak void Gimbal_Task(void *argument)
{
  /* USER CODE BEGIN Gimbal_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Gimbal_Task */
}

/* USER CODE BEGIN Header_Shoot_Task */
/**
* @brief Function implementing the shoot_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Shoot_Task */
__weak void Shoot_Task(void *argument)
{
  /* USER CODE BEGIN Shoot_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Shoot_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

