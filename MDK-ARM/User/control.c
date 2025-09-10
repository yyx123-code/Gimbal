/*New Try : 禁用mpu6050改用电机回传数据*/
#include "control.h"
#include "pid.h"
#include "drv_can.h"
#include "dbus.h"
#include "can.h"
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

/*can收发数据*/
uint8_t message_can1[8];//friction driver
uint8_t message_can2[8];//yaw
uint8_t message_can1_gm6020[8];//pitch
/*电机目标值*/
float yaw_target_angle = 0;
float pitch_target_angle = 0;
float driver_target_angle = 0;
int16_t friction_left_speed = 0;
int16_t friction_right_speed = 0;
/*消抖时间*/
const uint32_t DEBOUNCE_DELAY = 200;
/*前馈*/
float yaw_last_target = 0.0f;
float pitch_last_target = 0.0f;
float yaw_ff_out = 0;
float pitch_ff_out = 0;

/*Yaw*/
void Yaw_Task(void)
{
	static uint8_t idle = 0,last = 0;//边沿触发
	static uint32_t last_trigger_time = 0;//消抖
	if(channel[0] < CHANNEL_DEAD_ZONE && channel[0] > -CHANNEL_DEAD_ZONE) 
		idle = 1;//空闲状态
	else
	{
		idle = 0;//非空闲状态
		if(last)//边沿触发
		{
			uint32_t current_time = HAL_GetTick();//当前时间
			if(current_time - last_trigger_time > DEBOUNCE_DELAY)//消抖
			{
				if(channel[0] > CHANNEL_DEAD_ZONE)
					yaw_target_angle -= 30.0f;
				else if(channel[0] < -CHANNEL_DEAD_ZONE)
					yaw_target_angle += 30.0f;
				last_trigger_time = current_time;//上次触发时间
			}
			last = 0;
		}
	}
	/*串级PID*/
	pid_calculate(&yaw_angle_pid, yaw_target_angle, yaw_angle.total_angle);//角度环
	yaw_angle_pid.i_out = Limit_Min_Max(yaw_angle_pid.i_out, -5, 5);
	yaw_angle_pid.out = Limit_Min_Max(yaw_angle_pid.out, -90, 90);//角度环限幅

	pid_calculate(&yaw_speed_pid, yaw_angle_pid.out, yaw_angle.speed);//速度环
	
//	if(fabsf(yaw_target_angle - yaw_angle.total_angle) < 0.1f)
//		yaw_angle_pid.i_out = 0;
	
	if(yaw_target_angle != yaw_last_target)//前馈
		yaw_ff_out = yaw_angle_pid.kff * (yaw_target_angle - yaw_last_target);
	yaw_speed_pid.out += yaw_ff_out;
	
	last = idle;//更新状态
	yaw_last_target = yaw_target_angle;//更新上次目标
}

/*Pitch*/
void Pitch_Task(void)
{
	static uint8_t idle = 0,last = 0;//边沿触发
	static uint32_t last_trigger_time = 0;//消抖
	if(channel[1] < CHANNEL_DEAD_ZONE && channel[1] > -CHANNEL_DEAD_ZONE) 
		idle = 1;//空闲状态
	else
	{
		idle = 0;//非空闲状态
		if(last)//边沿触发
		{
			uint32_t current_time = HAL_GetTick();//当前时间
			if(current_time - last_trigger_time > DEBOUNCE_DELAY)//消抖
			{
				if(channel[1] > CHANNEL_DEAD_ZONE)
					pitch_target_angle -= 30.0f;
				else if(channel[1] < -CHANNEL_DEAD_ZONE)
					pitch_target_angle += 30.0f;
				last_trigger_time = current_time;//上次触发时间
			}
			last = 0;
		}
	}
	/*串级PID*/
	pid_calculate(&pitch_angle_pid, pitch_target_angle, pitch_angle.total_angle);//角度环
	pitch_angle_pid.i_out = Limit_Min_Max(pitch_angle_pid.i_out, -5, 5);
	pitch_angle_pid.out = Limit_Min_Max(pitch_angle_pid.out, -100, 100);//角度环限幅
	pid_calculate(&pitch_speed_pid, pitch_angle_pid.out, pitch_angle.speed);//速度环
	
	if(pitch_target_angle != pitch_last_target)//前馈
		pitch_ff_out = pitch_angle_pid.kff * (pitch_target_angle - pitch_last_target);
	pitch_speed_pid.out += pitch_ff_out;
	
	last = idle;//更新状态
	pitch_last_target = pitch_target_angle;//更新上次目标
}

/*拨盘电机*/
//void Driver_Task(void)
//{
//	static uint8_t idle = 0,last = 0;//边沿触发
//	static uint32_t last_trigger_time = 0;//消抖
//	
//	if(channel[2] < CHANNEL_DEAD_ZONE && channel[2] > -CHANNEL_DEAD_ZONE) 
//		idle = 1;//空闲状态
//	else
//	{
//		idle = 0;
//		if(last)
//		{
//			uint32_t current_time = HAL_GetTick();//当前时间
//			if(current_time - last_trigger_time > DEBOUNCE_DELAY)//消抖
//			{	
//				if(channel[2] < -CHANNEL_DEAD_ZONE)
//					driver_target_angle += 1440.0f;
//				if(channel[2] > CHANNEL_DEAD_ZONE)
//					//这里需要连发逻辑
//				last_trigger_time = current_time;//上次触发时间
//			}	
//			last = 0;
//		}
//	}
//	/*串级PID*/
//	pid_calculate(&driver_angle_pid, driver_target_angle, driver_angle.total_angle);
//	pid_calculate(&driver_speed_pid, driver_angle_pid.out, Driver_Data.speed);
//	
//	last = idle;
//}


/*摩擦轮 左正右反*/
void Friction_Task(void)
{
	/*目标速度*/
	if(channel[3] > CHANNEL_DEAD_ZONE)
	{
		friction_left_speed = M3508_TARGET;
		friction_right_speed = -M3508_TARGET;
	}
	else
	{
		friction_left_speed = 0;
		friction_right_speed = 0;
		pid_reset(&friction_left_pid);
		pid_reset(&friction_right_pid);
	}
	/*PID计算*/
	pid_calculate(&friction_left_pid, friction_left_speed, Friction_Left_Data.speed);
	pid_calculate(&friction_right_pid, friction_right_speed, -Friction_Right_Data.speed);
}

void PID_Init(void)
{
	/*PID初始化*/
	pid_init(&yaw_speed_pid, 2200.0f, 0.0f, 0.0f);
	pid_init(&yaw_angle_pid, 1.0f, 0.001f, 0.0f);
	
	pid_init(&pitch_speed_pid, 500.0f, 0.0f, 70.0f);
	pid_init(&pitch_angle_pid, 1.0f, 0.02f, 0.0f);
	
	pid_init(&driver_speed_pid, 0.0f, 0.0f, 0.0f);
	pid_init(&driver_angle_pid, 0.0f, 0.0f, 0.0f);
	
	pid_init(&friction_left_pid, 4.2f, 0.001f, 0.45f);
	pid_init(&friction_right_pid, 4.2f, 0.001f, 0.45f);
}

/*初始化任务*/
void Init_Task(void)
{
	CAN_Init();
	DBUS_Init();
	PID_Init();
}

void CAN_Task(void)
{
	/*统一PID限幅*/
	yaw_speed_pid.out = Limit_Min_Max(yaw_speed_pid.out, VOLTAGE_MIN, VOLTAGE_MAX);
	pitch_speed_pid.out = Limit_Min_Max(pitch_speed_pid.out, VOLTAGE_MIN, VOLTAGE_MAX);
	driver_speed_pid.out = Limit_Min_Max(driver_speed_pid.out, CURRENT_MIN, CURRENT_MAX);
	friction_left_pid.out = Limit_Min_Max(friction_left_pid.out, CURRENT_MIN, CURRENT_MAX);
	friction_right_pid.out = Limit_Min_Max(friction_right_pid.out, CURRENT_MIN, CURRENT_MAX);
	/*can发送信息*/
	DecimalToHex(yaw_speed_pid.out, &message_can2[6], &message_can2[7]);/*ID=4 Yaw GM6020*/
	DecimalToHex(pitch_speed_pid.out, &message_can1_gm6020[2], &message_can1_gm6020[3]);/*ID=2 Pitch GM6020*/
	DecimalToHex(friction_left_pid.out, &message_can1[0], &message_can1[1]);/*ID=1 左摩擦轮 M3508*/
	DecimalToHex(-friction_right_pid.out, &message_can1[2], &message_can1[3]);/*ID=2 右摩擦轮 M3508*/
	DecimalToHex(driver_speed_pid.out, &message_can1[4], &message_can1[5]);/*ID=3 拨盘电机 M2006 空载500rpm*/
  /*遥控保护*/
	switch(CheckRemoteSignal())
	{
		case 1:
			/*CAN发送*/
			CAN1_Send(UNIVERSAL_ID, message_can1);
			CAN1_Send(PITCH_ID, message_can1_gm6020);
			CAN2_Send(message_can2);
			break;
		case 0:
			break;
	}
}

void Motor_Init(void)
{
	if(channel[4] == 2 || channel[5] == 2)
	{
		yaw_angle.read_flag = 0;
		pitch_angle.read_flag = 0;
		driver_angle.read_flag = 0;
	}
}