#ifndef __DRV_CAN_H__
#define __DRV_CAN_H__
/*Include*/
#include <stdint.h>
#include "stm32f4xx.h"                  // Device header
#include "calculate.h"

/*宏定义区*/
/*标准格式:0x200+电机ID(1-4)*/
#define GM6020_CTRL_ID_BASE 0x1FF 	//GM6020控制指令基地址
#define M3508_CTRL_ID_BASE  0x200  //M3508控制指令基地址

#define LEFT_FRICTION_ID		0x201	 //左摩擦轮ID=1
#define RIGHT_FRICTION_ID		0x202	 //右摩擦轮ID=2
#define DRIVER_ID						0x203  //拨盘电机ID=3

#define YAW_ID							0x208  //YawID=4
#define PITCH_ID						0x206	 //PitchID=2

#define UNIVERSAL_ID        0x620	 //通用发送

/*电压电流*/
#define CURRENT_MAX  16384
#define CURRENT_MIN -16384
#define VOLTAGE_MAX  25000
#define VOLTAGE_MIN -25000
/*电机转速rpm*/
#define GM6020_MAX	 320.0f
#define GM6020_MIN  -320.0f
#define M3508_MAX		 8911.0f	//469 x 19
#define M3508_MIN		-8911.0f
#define M2006_MAX		 500.0f
#define M2006_MIN		-500.0f
/*角度限位*/
#define ANGLE_MAX_PITCH 27.0f
#define ANGLE_MIN_PITCH -15.0f
/*Π*/
#define M_PI 3.1415926f
/*目标角度速度*/
#define ANGLE_MAX_YAW 30.0f
#define ANGLE_MIN_YAW -30.0f
#define M3508_TARGET 6000.0f
#define M3508_START 0.0f
/*脉冲数*/
#define MOTOR_PPR 				8191.0f
/*拨盘电机参数*/
#define DRIVER_FIRE_MS 		50			//连发间隔50ms(20Hz)
#define ONE_SHOT 					1440.0f	//一发弹1440度
#define DRIVER_BRAKE			3.0f    //30度制动

/*CAN接收缓冲区*/
typedef struct
{
	uint32_t id;
	uint8_t data[8];
	uint8_t length;
}CAN_RxBuffer_t;

/*电机回传数据*/
typedef struct
{
  float mechanical_angle;  // 机械角度
  int16_t speed;             // 转速
  int16_t torque_current;    // 实际转矩电流
  uint8_t temperature;       // 电机温度
}MotorData_t;

/*CAN通道*/
typedef enum
{
	CAN_CHANNEL1,
	CAN_CHANNEL2
}CAN_Channel_t;

/*拨盘电机单连发*/

typedef enum
{
	SINGLE_SHOT,	//单发模式
	AUTO_FIRE,    //连发模式
	DRIVER_IDLE		//空闲模式
}Driver_Mode_t;

typedef enum
{
	FORWARD,    //正转模式
	REVERSE,		//反转模式
	IDLE_MODE		//空闲模式
}Motor_Mode_t;

/*CAN缓存*/
extern CAN_RxBuffer_t can1_rx_buffer;
extern CAN_RxBuffer_t can2_rx_buffer;
/*电机数据*/
extern MotorData_t Yaw_Data;
extern MotorData_t Pitch_Data;
extern MotorData_t Friction_Left_Data;
extern MotorData_t Friction_Right_Data;
extern MotorData_t Driver_Data;
/*拨盘电机*/
extern Driver_Mode_t Driver_Mode;
/*云台电机*/
extern Motor_Mode_t Yaw_Mode;
extern Motor_Mode_t Pitch_Mode;

void CAN_Init(void);
void CAN_Send(uint32_t motor_id, uint8_t *message);
void ProcessMotorData(CAN_RxBuffer_t* rx_buf);

#endif
