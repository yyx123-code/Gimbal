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
/*mpu6050*/
#include "dmpKey.h"
#include "dmpmap.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "MPU6050.h"
#include "MPU6050_reg.h"
/*sys*/
#include "usart.h"
#include "can.h"
#include "queue.h"
/*user*/
#include "drv_can.h"
#include "calculate.h"
#include "pid.h"
#include "control.h"
#include "dbus.h"
/*tool*/
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

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
  .priority = (osPriority_t) osPriorityRealtime1,
};
/* Definitions for MPU6050Task */
osThreadId_t MPU6050TaskHandle;
const osThreadAttr_t MPU6050Task_attributes = {
  .name = "MPU6050Task",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityRealtime2,
};
/* Definitions for PrintTask */
osThreadId_t PrintTaskHandle;
const osThreadAttr_t PrintTask_attributes = {
  .name = "PrintTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for YawTask */
osThreadId_t YawTaskHandle;
const osThreadAttr_t YawTask_attributes = {
  .name = "YawTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal7,
};
/* Definitions for PitchTask */
osThreadId_t PitchTaskHandle;
const osThreadAttr_t PitchTask_attributes = {
  .name = "PitchTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal6,
};
/* Definitions for FrictionTask */
osThreadId_t FrictionTaskHandle;
const osThreadAttr_t FrictionTask_attributes = {
  .name = "FrictionTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal5,
};
/* Definitions for DriverTask */
osThreadId_t DriverTaskHandle;
const osThreadAttr_t DriverTask_attributes = {
  .name = "DriverTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh4,
};
/* Definitions for DR16 */
osMessageQueueId_t DR16Handle;
const osMessageQueueAttr_t DR16_attributes = {
  .name = "DR16"
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
void can_task(void *argument);
void remote_task(void *argument);
void mpu6050_task(void *argument);
void print_task(void *argument);
void yaw_task(void *argument);
void pitch_task(void *argument);
void friction_task(void *argument);
void driver_task(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName);
void vApplicationMallocFailedHook(void);

/* USER CODE BEGIN 4 */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName)
{
   /* Run time stack overflow checking is performed if
   configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook function is
   called if a stack overflow is detected. */
}
/* USER CODE END 4 */

/* USER CODE BEGIN 5 */
void vApplicationMallocFailedHook(void)
{
   /* vApplicationMallocFailedHook() will only be called if
   configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h. It is a hook
   function that will get called if a call to pvPortMalloc() fails.
   pvPortMalloc() is called internally by the kernel whenever a task, queue,
   timer or semaphore is created. It is also called by various parts of the
   demo application. If heap_1.c or heap_2.c are used, then the size of the
   heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
   FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
   to query the size of free heap space that remains (although it does not
   provide information on how the remaining heap might be fragmented). */
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
  /* creation of DR16 */
  DR16Handle = osMessageQueueNew (64, sizeof(uint16_t), &DR16_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of InitTask */
  InitTaskHandle = osThreadNew(StartInitTask, NULL, &InitTask_attributes);

  /* creation of CANTask */
  CANTaskHandle = osThreadNew(can_task, NULL, &CANTask_attributes);

  /* creation of RemoteTask */
  RemoteTaskHandle = osThreadNew(remote_task, NULL, &RemoteTask_attributes);

  /* creation of MPU6050Task */
  MPU6050TaskHandle = osThreadNew(mpu6050_task, NULL, &MPU6050Task_attributes);

  /* creation of PrintTask */
  PrintTaskHandle = osThreadNew(print_task, NULL, &PrintTask_attributes);

  /* creation of YawTask */
  YawTaskHandle = osThreadNew(yaw_task, NULL, &YawTask_attributes);

  /* creation of PitchTask */
  PitchTaskHandle = osThreadNew(pitch_task, NULL, &PitchTask_attributes);

  /* creation of FrictionTask */
  FrictionTaskHandle = osThreadNew(friction_task, NULL, &FrictionTask_attributes);

  /* creation of DriverTask */
  DriverTaskHandle = osThreadNew(driver_task, NULL, &DriverTask_attributes);

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
		CAN_Task();//can发送
		osDelay(3);
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
		uint8_t result = Remote_Task();//遥控保护
		DR16_Data(&dr16,(uint8_t*)SbusRxBuffer);//dr16数据解析
		Motor_Init();//若数值更新失败可手动更新
    osDelay(3);
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
		/*计算总共转过的角度*/
		total_angle_calculate(&yaw_angle, Yaw_Data.mechanical_angle);
		total_angle_calculate(&pitch_angle, Pitch_Data.mechanical_angle);
		total_angle_calculate(&driver_angle, Driver_Data.mechanical_angle);
		yaw_angle.speed = Yaw_Data.speed;
		pitch_angle.speed = Pitch_Data.speed;
		driver_angle.speed = Driver_Data.speed;
		osDelay(1);
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
	static TickType_t last_wake_time = 0;
	
  /* Infinite loop */
  for(;;)
  {
		/*yaw*/
		printf("%f,%f,%f,%f,%d,%d\n", yaw_target_angle, yaw_angle.total_angle, pitch_target_angle, pitch_angle.total_angle, friction_left_speed, Friction_Left_Data.speed);
		/*pitch*/
//		printf("%f,%f,%f,%f\n", pitch_target_angle, pitch_angle.total_angle, pitch_angle_pid.out, pitch_angle.speed);
		
		/*摩擦轮*/
//		printf("%d,%d,%d,%d\n", friction_left_speed, Friction_Left_Data.speed, friction_right_speed, Friction_Right_Data.speed);
		
		/*拨盘*/
//		printf("%f,%f,%f,%f\n", driver_target_angle, driver_angle.total_angle, driver_angle_pid.out, driver_angle.speed);
		osDelay(10);
  }
  /* USER CODE END print_task */
}

/* USER CODE BEGIN Header_yaw_task */
/**
* @brief Function implementing the YawTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_yaw_task */
void yaw_task(void *argument)
{
  /* USER CODE BEGIN yaw_task */
  /* Infinite loop */
  for(;;)
  {
		Yaw_Task();
    osDelay(1);
  }
  /* USER CODE END yaw_task */
}

/* USER CODE BEGIN Header_pitch_task */
/**
* @brief Function implementing the PitchTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_pitch_task */
void pitch_task(void *argument)
{
  /* USER CODE BEGIN pitch_task */
  /* Infinite loop */
  for(;;)
  {
		Pitch_Task();
    osDelay(1);
  }
  /* USER CODE END pitch_task */
}

/* USER CODE BEGIN Header_friction_task */
/**
* @brief Function implementing the FrictionTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_friction_task */
void friction_task(void *argument)
{
  /* USER CODE BEGIN friction_task */
  /* Infinite loop */
  for(;;)
  {
		Friction_Task();
    osDelay(1);
  }
  /* USER CODE END friction_task */
}

/* USER CODE BEGIN Header_driver_task */
/**
* @brief Function implementing the DriverTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_driver_task */
void driver_task(void *argument)
{
  /* USER CODE BEGIN driver_task */
  /* Infinite loop */
  for(;;)
  {
//		Driver_Task();
    osDelay(1);
  }
  /* USER CODE END driver_task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

