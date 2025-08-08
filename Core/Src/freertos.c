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
/*欧拉角*/
float pitch = 0.0f;
float roll = 0.0f;
float yaw = 0.0f;
/*遥控数据*/
extern uint16_t channel[IBUS_USER_CHANNLES];
/*can收发数据*/
uint8_t message_can1[8];//发送can1
uint8_t message_can2[8];//发送can2
uint8_t RxData_can2_id1[8];//yaw
uint8_t RxData_can2_id4[8];//pitch
uint8_t RxData_can1_id1[8];//右摩擦轮
uint8_t RxData_can1_id4[8];//左摩擦轮
uint8_t RxData_can1_id2[8];//拨盘电机
/*数据ID及长度*/
uint32_t RxID_can1_id1, RxID_can1_id2, RxID_can1_id4;//can1
uint32_t RxID_can2_id1, RxID_can2_id4;//can2
uint8_t RxLength_can1_id1, RxLength_can1_id2, RxLength_can1_id4;//can1
uint8_t RxLength_can2_id1, RxLength_can2_id4;//can2
/*电机数据*/
MotorData_t Yaw_Data;
MotorData_t Pitch_Data;
MotorData_t Friction_Left_Data;
MotorData_t Friction_Right_Data;
MotorData_t Driver_Data;
/*PID结构体*/
pid_t yaw_speed_pid;
pid_t pitch_speed_pid;
pid_t driver_speed_pid;

pid_t yaw_angle_pid;
pid_t pitch_angle_pid;
pid_t driver_angle_pid;

pid_t friction_left_pid;
pid_t friction_right_pid;
/*角度结构体*/
Angle_t yaw_angle;
Angle_t pitch_angle;
Angle_t driver_angle;
/*目标速度*/
int yaw_speed, pitch_speed, driver_speed, friction_left_speed, friction_right_speed;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/*宏定义区*/
/*电压电流*/
#define CURRENT_MAX 16384
#define CURRENT_MIN -16384
#define VOLTAGE_MAX 25000
#define VOLTAGE_MIN -25000
/*电机转速rpm*/
#define GM6020_MAX 320
#define GM6020_MIN -320
#define M3508_MAX 8911	//469 x 19
#define M3508_MIN -8911
#define M2006_MAX 500
#define M2006_MIN -500
/*角度限位*/
#define ANGLE_MAX_PITCH 27.0f
#define ANGLE_MIN_PITCH -15.0f
/*Π*/
#define M_PI 3.1415926f
/*目标角度速度*/
#define ANGLE_MAX_YAW 30.0f
#define ANGLE_MIN_YAW -30.0f
#define M3508 6000
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
/* Definitions for canMutex */
osMutexId_t canMutexHandle;
const osMutexAttr_t canMutex_attributes = {
  .name = "canMutex"
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
  /* Create the mutex(es) */
  /* creation of canMutex */
  canMutexHandle = osMutexNew(&canMutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

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
		/*初始化任务*/
		taskENTER_CRITICAL();//进入临界区保护
		MPU6050_Init();
		uint8_t result0 = MPU_DMP_Init();
		CAN1_Init();
		CAN2_Init();
		IBUS_Init();
	  /*PID初始化*/
	  pid_init(&yaw_speed_pid, 20.0f, 0.0f, 2.0f);
	  pid_init(&yaw_angle_pid, 20.0f, 0.0f, 2.0f);
	  pid_init(&pitch_speed_pid, 10.0f, 0.0f, 0.1f);
	  pid_init(&pitch_angle_pid, 10.0f, 0.0f, 0.1f);
	  pid_init(&driver_speed_pid, 1.0f, 0.0f, 0.8f);
	  pid_init(&driver_angle_pid, 1.0f, 0.0f, 0.8f);
	  pid_init(&friction_left_pid, 2.3f, 0.08f, 0.9f);
	  pid_init(&friction_right_pid, 2.3f, 0.08f, 0.9f);
		osDelay(1000);
		taskEXIT_CRITICAL();//退出临界区保护
		vTaskDelete(NULL);//删除本身任务
		/*mpu6050 can ibus*/
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
    /*提取数据*/
	  if (RxID_can2_id1 == 0x205)//YawID=1
		{ExtractMotorData_can2(RxData_can2_id1, &Yaw_Data);}
		if (RxID_can2_id4 == 0x208)//PitchID=4
		{ExtractMotorData_can2(RxData_can2_id4, &Pitch_Data);}
		if (RxID_can1_id4 == 0x204)//左摩擦轮ID=4
		{ExtractMotorData_can1(RxData_can1_id4, &Friction_Left_Data);}
		if (RxID_can1_id1 == 0x201)//右摩擦轮ID=1
		{ExtractMotorData_can1(RxData_can1_id1, &Friction_Right_Data);}
		if (RxID_can1_id2 == 0x202)//拨盘电机ID=2
		{ExtractMotorData_can1(RxData_can1_id2, &Driver_Data);}	

		/*Yaw*/
		total_angle_calculate(&yaw_angle, Yaw_Data.mechanical_angle);//累计不含初始
		int target = channel[1];//遥控
		float yaw_angle_target = rc_to_motor(target);//速度
		pid_calculate(&yaw_angle_pid, yaw_angle_target, yaw_angle.total_angle);	
		pid_calculate(&yaw_speed_pid, yaw_angle_pid.out, Yaw_Data.speed);	
		
		if(fabsf(yaw_angle_pid.err[0]) > FAST_ZONE)
		{
			/* 远距离模式：激进控制 */
			yaw_angle_pid.kp = 3.0f;  // 高比例增益
			yaw_angle_pid.kd = 0.5f;  // 基础微分
			yaw_angle_pid.ki = 0.0f;  // 禁用积分
		}
		if(fabsf(yaw_angle_pid.err[0]) > BRAKE_ZONE)
		{
    /* 过渡模式：平滑减速 */
			yaw_angle_pid.kp = 1.5f;
			yaw_angle_pid.kd = 2.0f;  // 增强微分制动
			yaw_angle_pid.ki = 0.02f; // 微量积分
		}
		else
		{
			/* 精定位模式：强阻尼 */
			yaw_angle_pid.kp = 0.8f;
			yaw_angle_pid.kd = 5.0f;  // 最大微分制动
			yaw_angle_pid.ki = 0.0f;  // 禁用积分
			yaw_angle_pid.i_out = 0.0f; // 重置历史积分
		}
		// 误差越小，死区越大
		float dynamic_deadzone = 0.5f + 2.0f * expf(-fabsf(yaw_angle_pid.err[0])/5.0f); 
		if(fabsf(yaw_angle_pid.err[0]) < dynamic_deadzone)
		{
			yaw_speed_pid.out = 0; // 完全停止输出
		}		
		/*Pitch*/
		int pitch_rc = channel[1];
		pid_calculate(&pitch_speed_pid, rc_to_motor(pitch_rc), Pitch_Data.speed);
		
		/*摩擦轮*/
		float feedforward = friction_left_speed*0.05f;
		//分方向设置死区补偿
		if (Friction_Left_Data.speed < 300 && friction_left_speed > 20)
		{
			friction_left_pid.out += 100;
		}//正向启动补偿
		if (Friction_Left_Data.speed > 300 && friction_left_speed < 20)
		{
			friction_left_pid.out += 100;
		}//正向启动补偿
		
		friction_left_speed = ((channel[2] - 1003) / 997.0f) * M3508;
		pid_calculate(&friction_left_pid, friction_left_speed, Friction_Left_Data.speed);
		friction_right_speed = ((channel[2] - 1003) / 997.0f) * M3508;
		pid_calculate(&friction_right_pid, friction_right_speed, Friction_Right_Data.speed);
		//前馈
		if(friction_left_speed > 20)
		{friction_left_pid.out += feedforward;}
		/*摩擦轮死区*/
		if (channel[2] - 1000 < 100){friction_left_pid.out = 0, friction_right_pid.out = 0;}//直接清零
		/*摩擦轮*/
		
		/*拨盘电机*/
		driver_speed = channel[3] - 1500;
		pid_calculate(&driver_speed_pid, driver_speed, Driver_Data.speed);

		/*PID输出限幅*/
		friction_left_pid.out = Limit_Min_Max(friction_left_pid.out, CURRENT_MIN, CURRENT_MAX);
		friction_right_pid.out = Limit_Min_Max(friction_right_pid.out, CURRENT_MIN, CURRENT_MAX);
		driver_speed_pid.out = Limit_Min_Max(driver_speed_pid.out, CURRENT_MIN, CURRENT_MAX);


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
  	/*ID=1 Yaw GM6020*/							
		DecimalToHex(-yaw_speed_pid.out, &message_can2[0], &message_can2[1]);
		/*ID=4 Pitch GM6020*/
		DecimalToHex(-pitch_speed_pid.out, &message_can2[6], &message_can2[7]);
		/*ID=1 右摩擦轮 M3508*/
		DecimalToHex(-friction_right_pid.out, &message_can1[0], &message_can1[1]);
		/*ID=4 左摩擦轮 M3508*/
		DecimalToHex(friction_left_pid.out, &message_can1[6], &message_can1[7]);			
		/*ID=2 拨盘电机 M2006 空载500rpm*/
		DecimalToHex(driver_speed_pid.out, &message_can1[2], &message_can1[3]);
    /*遥控保护*/
		switch(CheckRemoteSignal())
		{
			case 1:
				/*CAN发送*/
				CAN1_Send(message_can1);	
				CAN2_Send(message_can2);				
				break;
			case 0:
				break;
		}
		/*CAN接收*/
		CAN2_Receive(&RxID_can2_id1,&RxLength_can2_id1,RxData_can2_id1);//yaw接收数据
		CAN2_Receive(&RxID_can2_id4,&RxLength_can2_id4,RxData_can2_id4);//pitch接收数据
		CAN1_Receive(&RxID_can1_id1,&RxLength_can1_id1,RxData_can1_id1);//右摩擦轮接收数据
		CAN1_Receive(&RxID_can1_id4,&RxLength_can1_id4,RxData_can1_id4);//左摩擦轮接收数据
		CAN1_Receive(&RxID_can1_id2,&RxLength_can1_id2,RxData_can1_id2);//拨盘电机接收数据
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

    osDelay(1);
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
		if(HAL_CAN_GetState(&hcan2) == HAL_CAN_STATE_ERROR)
		{
			HAL_CAN_Stop(&hcan2);
			MX_CAN2_Init(); // 重新初始
		}
		osDelay(20);
		/*优化：DMA*/
  }
  /* USER CODE END print_task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

/*
 *优化空间：
 *封装进user文件 简化os函数
 *使用队列、互斥量、信号量、事件组来传递信息
*/
