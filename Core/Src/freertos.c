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
#include <stdio.h>
#include "usart.h"
#include "fdcan.h"
#include "BMI088driver.h"
#include "BMI088Middleware.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* ---- CAN / M3508 test ---- */
#define M3508_CMD_ID      0x200u   /* C620 control frame, 4 x int16 current */
#define M3508_CMD_MAX     1000     /* ~1.2A; +-16384 corresponds to +-20A   */
#define M3508_RAMP_STEP   1        /* +1 per 1ms -> 1s from 0 to MAX        */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* ---- CAN / M3508 test: watch these in the Keil debugger ---- */
volatile int32_t  m3508_bus      = -1;   /* 0=FDCAN1, 1=FDCAN2, 2=FDCAN3 */
volatile int32_t  m3508_motor    = -1;   /* 0..3 -> ESC id 1..4          */
volatile int16_t  m3508_cmd      = 0;    /* current being sent           */
volatile uint32_t m3508_last_rx_tick = 0;
volatile uint32_t m3508_rx_total[3];     /* frames received per bus      */
volatile int16_t  m3508_ecd[4];
volatile int16_t  m3508_rpm[4];
volatile int16_t  m3508_cur[4];
volatile uint8_t  m3508_temp[4];

/* USER CODE END Variables */
/* Definitions for ui_task */
osThreadId_t ui_taskHandle;
const osThreadAttr_t ui_task_attributes = {
  .name = "ui_task",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for crg_test_task */
osThreadId_t crg_test_taskHandle;
const osThreadAttr_t crg_test_task_attributes = {
  .name = "crg_test_task",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for imu_task */
osThreadId_t imu_taskHandle;
const osThreadAttr_t imu_task_attributes = {
  .name = "imu_task",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityHigh1,
};
/* Definitions for referee_usart_t */
osThreadId_t referee_usart_tHandle;
const osThreadAttr_t referee_usart_t_attributes = {
  .name = "referee_usart_t",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for chassis_task */
osThreadId_t chassis_taskHandle;
const osThreadAttr_t chassis_task_attributes = {
  .name = "chassis_task",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
static void Can_Test_Start(FDCAN_HandleTypeDef *h);
static void M3508_SendCurrent(int32_t bus, int16_t i1, int16_t i2, int16_t i3, int16_t i4);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void Crg_test_Task(void *argument);
void Imu_Task(void *argument);
void Referee_Usart_Task(void *argument);
void Chassis_Task(void *argument);

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
  /* creation of ui_task */
  //ui_taskHandle = osThreadNew(StartDefaultTask, NULL, &ui_task_attributes);

  /* creation of crg_test_task */
  crg_test_taskHandle = osThreadNew(Crg_test_Task, NULL, &crg_test_task_attributes);

  /* creation of imu_task */
  imu_taskHandle = osThreadNew(Imu_Task, NULL, &imu_task_attributes);

  /* creation of referee_usart_t */
  //referee_usart_tHandle = osThreadNew(Referee_Usart_Task, NULL, &referee_usart_t_attributes);

  /* creation of chassis_task */
  //chassis_taskHandle = osThreadNew(Chassis_Task, NULL, &chassis_task_attributes);

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
__weak void StartDefaultTask(void *argument)
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
  /* ---- CAN / M3508 bring-up test --------------------------------------
   * All three FDCAN buses are started with an accept-all filter.
   * The first motor feedback frame (0x201..0x204) decides which bus and
   * which ESC slot is used, so the wiring does not have to be known in
   * advance. Watch these in the Keil debugger:
   *     m3508_bus, m3508_motor, m3508_cmd
   *     m3508_rx_total[3]
   *     m3508_rpm[4], m3508_cur[4], m3508_ecd[4], m3508_temp[4]
   * No UART output is produced here.
   * -------------------------------------------------------------------- */
  Can_Test_Start(&hfdcan1);
  Can_Test_Start(&hfdcan2);
  Can_Test_Start(&hfdcan3);

  /* Infinite loop */
  for(;;)
  {
    /* Nothing is transmitted until a motor answers on some bus. */
    if ((m3508_bus >= 0) && (m3508_motor >= 0))
    {
      if ((HAL_GetTick() - m3508_last_rx_tick) > 100u)
      {
        /* Feedback lost -> drop the current instead of driving blind */
        m3508_cmd = 0;
      }
      else if (m3508_cmd < M3508_CMD_MAX)
      {
        m3508_cmd += M3508_RAMP_STEP;
      }

      int16_t cur[4] = {0, 0, 0, 0};
      cur[(uint32_t)m3508_motor & 0x03u] = m3508_cmd;
      M3508_SendCurrent(m3508_bus, cur[0], cur[1], cur[2], cur[3]);
    }

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

/* USER CODE BEGIN Header_Referee_Usart_Task */
/**
* @brief Function implementing the referee_usart_t thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Referee_Usart_Task */
__weak void Referee_Usart_Task(void *argument)
{
  /* USER CODE BEGIN Referee_Usart_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Referee_Usart_Task */
}

/* USER CODE BEGIN Header_Chassis_Task */
/**
* @brief Function implementing the chassis_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Chassis_Task */
__weak void Chassis_Task(void *argument)
{
  /* USER CODE BEGIN Chassis_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Chassis_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* Map a HAL handle back to 0/1/2 so the received frames can be counted. */
static int32_t Can_BusIndex(FDCAN_HandleTypeDef *h)
{
  if (h->Instance == FDCAN1) return 0;
  if (h->Instance == FDCAN2) return 1;
  if (h->Instance == FDCAN3) return 2;
  return -1;
}

/*
 * Enable one FDCAN bus:
 *   - accept-all standard filter into RX FIFO0 (mask = 0)
 *   - unmatched frames also go to RX FIFO0 instead of being rejected
 *   - start the peripheral and enable the new-message interrupt
 */
static void Can_Test_Start(FDCAN_HandleTypeDef *h)
{
  FDCAN_FilterTypeDef f = {0};

  f.IdType       = FDCAN_STANDARD_ID;
  f.FilterIndex  = 0;                 /* must be < StdFiltersNbr */
  f.FilterType   = FDCAN_FILTER_MASK;
  f.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  f.FilterID1    = 0x000;             /* ID   */
  f.FilterID2    = 0x000;             /* mask: 0 = accept everything */

  (void)HAL_FDCAN_ConfigFilter(h, &f);

  (void)HAL_FDCAN_ConfigGlobalFilter(h,
                                     FDCAN_ACCEPT_IN_RX_FIFO0,
                                     FDCAN_ACCEPT_IN_RX_FIFO0,
                                     FDCAN_FILTER_REMOTE,
                                     FDCAN_FILTER_REMOTE);

  (void)HAL_FDCAN_Start(h);

  (void)HAL_FDCAN_ActivateNotification(h, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
}

/*
 * Send the C620 control frame 0x200 (4 x int16, big endian).
 * bus: 0 = FDCAN1, 1 = FDCAN2, 2 = FDCAN3
 */
static void M3508_SendCurrent(int32_t bus, int16_t i1, int16_t i2, int16_t i3, int16_t i4)
{
  FDCAN_HandleTypeDef  *h;
  FDCAN_TxHeaderTypeDef tx = {0};
  uint8_t d[8];

  if      (bus == 0) h = &hfdcan1;
  else if (bus == 1) h = &hfdcan2;
  else if (bus == 2) h = &hfdcan3;
  else               return;

  tx.Identifier          = M3508_CMD_ID;
  tx.IdType              = FDCAN_STANDARD_ID;
  tx.TxFrameType         = FDCAN_DATA_FRAME;
  tx.DataLength          = FDCAN_DLC_BYTES_8;
  tx.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  tx.BitRateSwitch       = FDCAN_BRS_OFF;      /* classic CAN, 1Mbps */
  tx.FDFormat            = FDCAN_CLASSIC_CAN;
  tx.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
  tx.MessageMarker       = 0;

  d[0] = (uint8_t)((uint16_t)i1 >> 8); d[1] = (uint8_t)i1;
  d[2] = (uint8_t)((uint16_t)i2 >> 8); d[3] = (uint8_t)i2;
  d[4] = (uint8_t)((uint16_t)i3 >> 8); d[5] = (uint8_t)i3;
  d[6] = (uint8_t)((uint16_t)i4 >> 8); d[7] = (uint8_t)i4;

  if (HAL_FDCAN_GetTxFifoFreeLevel(h) > 0u)
  {
    (void)HAL_FDCAN_AddMessageToTxFifoQ(h, &tx, d);
  }
}

/*
 * RX FIFO0 callback for all three FDCAN instances.
 * C620 feedback: 0x201..0x204, 8 bytes:
 *   [0..1] mechanical angle, [2..3] rpm, [4..5] actual current,
 *   [6] temperature
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
  FDCAN_RxHeaderTypeDef rx;
  uint8_t d[8];
  int32_t bus;

  if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0u)
  {
    return;
  }

  while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0u)
  {
    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx, d) != HAL_OK)
    {
      break;
    }

    bus = Can_BusIndex(hfdcan);
    if (bus >= 0)
    {
      m3508_rx_total[bus]++;
    }

    if ((rx.IdType == FDCAN_STANDARD_ID) &&
        (rx.Identifier >= 0x201u) && (rx.Identifier <= 0x204u))
    {
      uint32_t i = rx.Identifier - 0x201u;

      m3508_ecd[i]  = (int16_t)(((uint16_t)d[0] << 8) | d[1]);
      m3508_rpm[i]  = (int16_t)(((uint16_t)d[2] << 8) | d[3]);
      m3508_cur[i]  = (int16_t)(((uint16_t)d[4] << 8) | d[5]);
      m3508_temp[i] = d[6];

      if (bus >= 0)
      {
        if (m3508_bus < 0)
        {
          m3508_bus = bus;              /* remember the bus that answers */
        }
        if (m3508_motor < 0)
        {
          m3508_motor = (int32_t)i;     /* remember the ESC id           */
        }
      }
      m3508_last_rx_tick = HAL_GetTick();
    }
  }
}

/* USER CODE END Application */

