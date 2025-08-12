/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "dmpKey.h"
#include "dmpmap.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "MPU6050.h"
#include "MPU6050_reg.h"
/*mpu6050*/
#include "usart.h"
#include "can.h"
/*sys*/
#include "drv_can.h"
#include "calculate.h"
#include "pid.h"
#include "fly_ibus.h"
#include "control.h"
/*user*/
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
/*tool*/
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
/*串口重定向*/
int fputc(int ch,FILE *f)
{
    HAL_UART_Transmit(&huart3,(uint8_t*)&ch,1,0xFFFF);
    return ch;
}

/* USER CODE END Variables */
/* Definitions for InitTask */
osThreadId_t InitTaskHandle;
const osThreadAttr_t InitTask_attributes = {
  .name = "InitTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityRealtime7,
};
/* Definitions for PIDTask */
osThreadId_t PIDTaskHandle;
const osThreadAttr_t PIDTask_attributes = {
  .name = "PIDTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for CANTask */
osThreadId_t CANTaskHandle;
const osThreadAttr_t CANTask_attributes = {
  .name = "CANTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh1,
};
/* Definitions for RemoteTask */
osThreadId_t RemoteTaskHandle;
const osThreadAttr_t RemoteTask_attributes = {
  .name = "RemoteTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for MPU6050Task */
osThreadId_t MPU6050TaskHandle;
const osThreadAttr_t MPU6050Task_attributes = {
  .name = "MPU6050Task",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityHigh2,
};
/* Definitions for PrintTask */
osThreadId_t PrintTaskHandle;
const osThreadAttr_t PrintTask_attributes = {
  .name = "PrintTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for canQueue */
osMessageQueueId_t canQueueHandle;
const osMessageQueueAttr_t canQueue_attributes = {
  .name = "canQueue"
};
/* Definitions for printQueue */
osMessageQueueId_t printQueueHandle;
const osMessageQueueAttr_t printQueue_attributes = {
  .name = "printQueue"
};
/* Definitions for remoteQueue */
osMessageQueueId_t remoteQueueHandle;
const osMessageQueueAttr_t remoteQueue_attributes = {
  .name = "remoteQueue"
};
/* Definitions for mpu6050Queue */
osMessageQueueId_t mpu6050QueueHandle;
const osMessageQueueAttr_t mpu6050Queue_attributes = {
  .name = "mpu6050Queue"
};
/* Definitions for xMotorDataMutex */
osSemaphoreId_t xMotorDataMutexHandle;
const osSemaphoreAttr_t xMotorDataMutex_attributes = {
  .name = "xMotorDataMutex"
};
/* Definitions for RemoteEvent */
osEventFlagsId_t RemoteEventHandle;
const osEventFlagsAttr_t RemoteEvent_attributes = {
  .name = "RemoteEvent"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartInitTask(void *argument);
void pid_task(void *argument);
void can_task(void *argument);
void remote_task(void *argument);
void mpu6050_task(void *argument);
void print_task(void *argument);

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

  /* Create the semaphores(s) */
  /* creation of xMotorDataMutex */
  xMotorDataMutexHandle = osSemaphoreNew(1, 1, &xMotorDataMutex_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of canQueue */
  canQueueHandle = osMessageQueueNew (32, sizeof(uint16_t), &canQueue_attributes);

  /* creation of printQueue */
  printQueueHandle = osMessageQueueNew (32, sizeof(uint16_t), &printQueue_attributes);

  /* creation of remoteQueue */
  remoteQueueHandle = osMessageQueueNew (64, sizeof(uint16_t), &remoteQueue_attributes);

  /* creation of mpu6050Queue */
  mpu6050QueueHandle = osMessageQueueNew (16, sizeof(uint16_t), &mpu6050Queue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of InitTask */
  InitTaskHandle = osThreadNew(StartInitTask, NULL, &InitTask_attributes);

  /* creation of PIDTask */
  PIDTaskHandle = osThreadNew(pid_task, NULL, &PIDTask_attributes);

  /* creation of CANTask */
  CANTaskHandle = osThreadNew(can_task, NULL, &CANTask_attributes);

  /* creation of RemoteTask */
  RemoteTaskHandle = osThreadNew(remote_task, NULL, &RemoteTask_attributes);

  /* creation of MPU6050Task */
  MPU6050TaskHandle = osThreadNew(mpu6050_task, NULL, &MPU6050Task_attributes);

  /* creation of PrintTask */
  PrintTaskHandle = osThreadNew(print_task, NULL, &PrintTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Create the event(s) */
  /* creation of RemoteEvent */
  RemoteEventHandle = osEventFlagsNew(&RemoteEvent_attributes);

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartInitTask */
/**
  * @brief  Function implementing the InitTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartInitTask */
void StartInitTask(void *argument)
{
  /* USER CODE BEGIN StartInitTask */
  /* Infinite loop */
  for(;;)
  {
		taskENTER_CRITICAL();//进入临界区保护
		Init_Task();				 //初始化任务
		osDelay(1000);			 //适当延时
		taskEXIT_CRITICAL(); //退出临界区保护
		vTaskDelete(NULL);	 //删除本身任务
  }
  /* USER CODE END StartInitTask */
}

/* USER CODE BEGIN Header_pid_task */
/**
* @brief Function implementing the PIDTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_pid_task */
void pid_task(void *argument)
{
  /* USER CODE BEGIN pid_task */
  /* Infinite loop */
  for(;;)
  {
		Yaw_Task();
		Pitch_Task();
		Driver_Task();
		Friction_Task();
    osDelay(5);
  }
  /* USER CODE END pid_task */
}

/* USER CODE BEGIN Header_can_task */
/**
* @brief Function implementing the CANTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_can_task */
void can_task(void *argument)
{
  /* USER CODE BEGIN can_task */
  /* Infinite loop */
  for(;;)
  {
		CAN_Task();
		osDelay(5);
  }
  /* USER CODE END can_task */
}

/* USER CODE BEGIN Header_remote_task */
/**
* @brief Function implementing the RemoteTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_remote_task */
void remote_task(void *argument)
{
  /* USER CODE BEGIN remote_task */
  /* Infinite loop */
  for(;;)
  {
		uint8_t result2 = Remote_Task();
		Deinit_Task();//重新初始化
    osDelay(100);
  }
  /* USER CODE END remote_task */
}

/* USER CODE BEGIN Header_mpu6050_task */
/**
* @brief Function implementing the MPU6050Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_mpu6050_task */
void mpu6050_task(void *argument)
{
  /* USER CODE BEGIN mpu6050_task */
  /* Infinite loop */
  for(;;)
  {
		/*欧拉角*/
		uint8_t result1 = MPU_DMP_GetData(&pitch, &roll, &yaw);
		imu_calculate(&Yaw_IMU, yaw, 5);
		imu_calculate(&Pitch_IMU, pitch, 5);
		osDelay(5);
  }
  /* USER CODE END mpu6050_task */
}

/* USER CODE BEGIN Header_print_task */
/**
* @brief Function implementing the PrintTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_print_task */
void print_task(void *argument)
{
  /* USER CODE BEGIN print_task */
  /* Infinite loop */
  for(;;)
  {
		/*打印*/
		printf("%f,%f,%f\n", yaw,roll,pitch);
		osDelay(20);
		/*优化：DMA*/
  }
  /* USER CODE END print_task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

