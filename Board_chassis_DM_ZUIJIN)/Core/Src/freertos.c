/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "ws2812.h"
#include "ins.h"
#include "Task_manager.h"
#include "bsp_dwt.h"
#include "Saber_C3.h"
#include "board_comm.h"
#include "vofa.h"
#include "N100.h"
#include "SBUS.h"
//#include "BMI088driver.h"
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
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for ChassisTask */
osThreadId_t ChassisTaskHandle;
const osThreadAttr_t ChassisTask_attributes = {
  .name = "ChassisTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for LegMotorTask */
osThreadId_t LegMotorTaskHandle;
const osThreadAttr_t LegMotorTask_attributes = {
  .name = "LegMotorTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for IMUTask */
osThreadId_t IMUTaskHandle;
const osThreadAttr_t IMUTask_attributes = {
  .name = "IMUTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for BoardcommTask */
osThreadId_t BoardcommTaskHandle;
const osThreadAttr_t BoardcommTask_attributes = {
  .name = "BoardcommTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for VofaTask */
osThreadId_t VofaTaskHandle;
const osThreadAttr_t VofaTask_attributes = {
  .name = "VofaTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for ET16STask */
osThreadId_t ET16STaskHandle;
const osThreadAttr_t ET16STask_attributes = {
  .name = "ET16STask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartChassisTask(void *argument);
void StartLegMotorTask(void *argument);
void StartIMUTask(void *argument);
void StartBoardcommTask(void *argument);
void StartVofaTask(void *argument);
void StartET16STask(void *argument);

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
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of ChassisTask */
  ChassisTaskHandle = osThreadNew(StartChassisTask, NULL, &ChassisTask_attributes);

  /* creation of LegMotorTask */
  LegMotorTaskHandle = osThreadNew(StartLegMotorTask, NULL, &LegMotorTask_attributes);

  /* creation of IMUTask */
  IMUTaskHandle = osThreadNew(StartIMUTask, NULL, &IMUTask_attributes);

  /* creation of BoardcommTask */
  BoardcommTaskHandle = osThreadNew(StartBoardcommTask, NULL, &BoardcommTask_attributes);

  /* creation of VofaTask */
  VofaTaskHandle = osThreadNew(StartVofaTask, NULL, &VofaTask_attributes);

  /* creation of ET16STask */
  ET16STaskHandle = osThreadNew(StartET16STask, NULL, &ET16STask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
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
		WS2812_Ctrl(50, 0, 0);
		osDelay(500);
		WS2812_Ctrl(10, 10, 50);
		osDelay(500);

  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartChassisTask */
/**
* @brief Function implementing the ChassisTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartChassisTask */
void StartChassisTask(void *argument)
{
  /* USER CODE BEGIN StartChassisTask */
  /* Infinite loop */
	static float chassis_start;
  static float chassis_dt;
	ChassisInit();
  for(;;)
  {
		chassis_start = DWT_GetTimeline_ms();
    ChassisCalcTask();
    chassis_dt = DWT_GetTimeline_ms() - chassis_start;
    osDelay(1);
  }
  /* USER CODE END StartChassisTask */
}

/* USER CODE BEGIN Header_StartLegMotorTask */
/**
* @brief Function implementing the LegMotorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLegMotorTask */
void StartLegMotorTask(void *argument)
{
  /* USER CODE BEGIN StartLegMotorTask */
  /* Infinite loop */
	static float leg_start;
  static float leg_dt;
  for(;;)
  {
		leg_start = DWT_GetTimeline_ms();
				DM_MotorTask();
		leg_dt = DWT_GetTimeline_ms() - leg_start;
						WheelMotorTask();
		leg_dt = DWT_GetTimeline_ms() - leg_start;
    osDelay(1);
  }
  /* USER CODE END StartLegMotorTask */
}

/* USER CODE BEGIN Header_StartIMUTask */
/**
* @brief Function implementing the IMUTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartIMUTask */
void StartIMUTask(void *argument)
{
  /* USER CODE BEGIN StartIMUTask */
	static float ins_start;
  static float ins_dt;
			INS_Init();
  /* Infinite loop */
  for(;;)
  {
		ins_start = DWT_GetTimeline_ms();
		INS_Task();
		ins_dt = DWT_GetTimeline_ms() - ins_start;
    osDelay(1);
  }
  /* USER CODE END StartIMUTask */
}

/* USER CODE BEGIN Header_StartBoardcommTask */
/**
* @brief Function implementing the BoardcommTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartBoardcommTask */
void StartBoardcommTask(void *argument)
{
  /* USER CODE BEGIN StartBoardcommTask */
	static float board_start;
  static float board_dt;
	 BoardCommInit(&board_comm,&hfdcan2, 0x11f);
  /* Infinite loop */
  for(;;)
  {
		board_start = DWT_GetTimeline_ms();
		boardCommunicateTask();
		board_dt = DWT_GetTimeline_ms() - board_start;
    osDelay(1);
  }
  /* USER CODE END StartBoardcommTask */
}

/* USER CODE BEGIN Header_StartVofaTask */
/**
* @brief Function implementing the VofaTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartVofaTask */
void StartVofaTask(void *argument)
{
  /* USER CODE BEGIN StartVofaTask */
  /* Infinite loop */
  for(;;)
  {
    vofaTask();
    Vofa_JustFloat(Vofa.data, 3);

  }
  /* USER CODE END StartVofaTask */
}

/* USER CODE BEGIN Header_StartET16STask */
/**
* @brief Function implementing the ET16STask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartET16STask */
void StartET16STask(void *argument)
{
  /* USER CODE BEGIN StartET16STask */
  static float ET16S_start;
  static float ET16S_dt;
  SBUS_IT_Open();
  /* Infinite loop */
  for(;;)
  {
    ET16S_start = DWT_GetTimeline_ms();
    ET16S_dt = DWT_GetTimeline_ms() - ET16S_start;
    osDelay(1);
  }
  /* USER CODE END StartET16STask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

