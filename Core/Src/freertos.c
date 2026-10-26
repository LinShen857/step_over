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
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for errorTask */
osThreadId_t errorTaskHandle;
const osThreadAttr_t errorTask_attributes = {
  .name = "errorTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for remoteCtrlTask */
osThreadId_t remoteCtrlTaskHandle;
const osThreadAttr_t remoteCtrlTask_attributes = {
  .name = "remoteCtrlTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for daemonTask */
osThreadId_t daemonTaskHandle;
const osThreadAttr_t daemonTask_attributes = {
  .name = "daemonTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for hostTask */
osThreadId_t hostTaskHandle;
const osThreadAttr_t hostTask_attributes = {
  .name = "hostTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for wdgTask */
osThreadId_t wdgTaskHandle;
const osThreadAttr_t wdgTask_attributes = {
  .name = "wdgTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for chassisTask */
osThreadId_t chassisTaskHandle;
const osThreadAttr_t chassisTask_attributes = {
  .name = "chassisTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for roboarmTask */
osThreadId_t roboarmTaskHandle;
const osThreadAttr_t roboarmTask_attributes = {
  .name = "roboarmTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for imuTask */
osThreadId_t imuTaskHandle;
const osThreadAttr_t imuTask_attributes = {
  .name = "imuTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for refereeTask */
osThreadId_t refereeTaskHandle;
const osThreadAttr_t refereeTask_attributes = {
  .name = "refereeTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void TaskStart(void *argument);
extern void TaskError(void *argument);
extern void TaskRemoteCtrl(void *argument);
extern void TaskDaemon(void *argument);
extern void TaskHost(void *argument);
extern void Taskwdg(void *argument);
extern void TaskChassis(void *argument);
extern void TaskRoboarm(void *argument);
extern void TaskImu(void *argument);
extern void TaskReferee(void *argument);

extern void MX_USB_DEVICE_Init(void);
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
  defaultTaskHandle = osThreadNew(TaskStart, NULL, &defaultTask_attributes);

  /* creation of errorTask */
  errorTaskHandle = osThreadNew(TaskError, NULL, &errorTask_attributes);

  /* creation of remoteCtrlTask */
  remoteCtrlTaskHandle = osThreadNew(TaskRemoteCtrl, NULL, &remoteCtrlTask_attributes);

  /* creation of daemonTask */
  daemonTaskHandle = osThreadNew(TaskDaemon, NULL, &daemonTask_attributes);

  /* creation of hostTask */
  hostTaskHandle = osThreadNew(TaskHost, NULL, &hostTask_attributes);

  /* creation of wdgTask */
  wdgTaskHandle = osThreadNew(Taskwdg, NULL, &wdgTask_attributes);

  /* creation of chassisTask */
  chassisTaskHandle = osThreadNew(TaskChassis, NULL, &chassisTask_attributes);

  /* creation of roboarmTask */
  roboarmTaskHandle = osThreadNew(TaskRoboarm, NULL, &roboarmTask_attributes);

  /* creation of imuTask */
  imuTaskHandle = osThreadNew(TaskImu, NULL, &imuTask_attributes);

  /* creation of refereeTask */
  refereeTaskHandle = osThreadNew(TaskReferee, NULL, &refereeTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_TaskStart */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_TaskStart */
__weak void TaskStart(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN TaskStart */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END TaskStart */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

