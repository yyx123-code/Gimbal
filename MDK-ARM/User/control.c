/*user*/
#include "control.h"
#include "pid.h"
#include "drv_can.h"
#include "dbus.h"
/*sys*/
#include "can.h"
/*mpu6050*/
#include "dmpKey.h"
#include "dmpmap.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "MPU6050.h"
#include "MPU6050_reg.h"
/*tool*/
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

/*欧拉角*/
float pitch = 0.0f;
float roll = 0.0f;
float yaw = 0.0f;
/*can收发数据*/
uint8_t message_can1[8];//friction driver
uint8_t message_can2[8];//yaw
uint8_t message_can1_gm6020[8];//pitch
/*角度*/
Angle_t Yaw_IMU;
Angle_t Pitch_IMU;
Angle_t yaw_angle;
Angle_t pitch_angle;
Angle_t driver_angle;
/*dr16遥控超时保护*/
bool changed_flag = false;
uint16_t timeout_counter = 0;
int16_t last_channel[DR16_CHANNEL];
int16_t channel[DR16_CHANNEL];
/*紧急制动*/
uint8_t last_signal = 0;
/*Pitch上下统一输出*/
int16_t pitch_output = 0;
/*目标*/
float yaw_target_angle = 0;
float pitch_target_angle = 0;
float driver_target_angle = 0;
int16_t friction_left_speed = 0;
int16_t friction_right_speed = 0;
float yaw_last_target = 0.0f;

/*初始化任务*/
void Init_Task(void)
{
	__disable_irq();//禁用全局中断
	MPU6050_Init();
	uint8_t result0 = MPU_DMP_Init();
	__enable_irq();//启用所有中断
	CAN_Init();
	DBUS_Init();
	ValueAllInit();
}

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

/*拨盘电机*/
void Driver_Task(void)
{
	/*计算总共转过的角度*/
	total_angle_calculate(&driver_angle, Driver_Data.mechanical_angle, 5);
	if(channel[2] < CHANNEL_DEAD_ZONE && channel[2] > -CHANNEL_DEAD_ZONE) 
	{
		Motor_Init(&driver_angle, Driver_Data.mechanical_angle);//重置累计角度
		driver_target_angle = 0.0f;
	}
	if(channel[2] > CHANNEL_DEAD_ZONE)
		driver_target_angle = 1440.0f;
	/*串级PID*/
	pid_calculate(&driver_angle_pid, driver_target_angle, driver_angle.total_angle);
	pid_calculate(&driver_speed_pid, driver_angle_pid.out, Driver_Data.speed);
}

/*Yaw*/
void Yaw_Task(void)
{
	static uint8_t idle = 1,last = 1;//边沿触发
	static uint32_t last_trigger_time = 0;//消抖
	const uint32_t DEBOUNCE_DELAY = 200;//消抖时间
	if(Yaw_IMU.target_init)//初始化目标值
	{
		yaw_target_angle = Yaw_IMU.initial_angle;
		Yaw_IMU.target_init = 0;
	}
	if(channel[0] < CHANNEL_DEAD_ZONE && channel[0] > -CHANNEL_DEAD_ZONE) 
		idle = 1;//空闲状态
	else
	{
		idle = 0;//设定目标值状态
		if(last)
		{
			uint32_t current_time = HAL_GetTick();
			if(current_time - last_trigger_time > DEBOUNCE_DELAY)
			{
				if(channel[0] > CHANNEL_DEAD_ZONE)
					yaw_target_angle -= 30.0f;
				else if(channel[0] < -CHANNEL_DEAD_ZONE)
					yaw_target_angle += 30.0f;
				last_trigger_time = current_time;
			}
			last = 0;
		}
	}
	/*串级PID*/

//	pid_calculate(&yaw_angle_pid, yaw_target_angle, LowPassFilter(Yaw_IMU.total_angle));
//	yaw_angle_pid.out = Limit_Min_Max(yaw_angle_pid.out, -100, 100);
//	pid_calculate(&yaw_speed_pid, yaw_angle_pid.out, Yaw_IMU.speed);
	
	/*角度单环*/

	pid_calculate(&yaw_angle_pid, yaw_target_angle - Yaw_IMU.total_angle, Yaw_IMU.speed);
	
	if(yaw_target_angle != yaw_last_target)//前馈
		yaw_angle_pid.out += yaw_angle_pid.kff * (yaw_target_angle - yaw_last_target);
	last = idle;
	yaw_last_target = yaw_target_angle;
}

/*Pitch*/
void Pitch_Task(void)
{
	static uint8_t idle = 1,last = 1;
	static uint32_t last_trigger_time = 0;
	const uint32_t DEBOUNCE_DELAY = 200;
	if(Yaw_IMU.target_init)//初始化目标值
	{
		yaw_target_angle = Yaw_IMU.initial_angle;
		Yaw_IMU.target_init = 0;
	}
	if(channel[1] < CHANNEL_DEAD_ZONE && channel[1] > -CHANNEL_DEAD_ZONE) 
	{
		idle = 1;
	}
	else
	{
		idle = 0;
		if(last)
		{
			uint32_t current_time = HAL_GetTick();
			if(current_time - last_trigger_time > DEBOUNCE_DELAY)
			{
				/*软限位*/
//				if(channel[1] > CHANNEL_DEAD_ZONE && pitch < -3.0f)
//					pitch_target_angle -= 30.0f;
//				else
//					pitch_target_angle -= ANGLE_MAX_PITCH - pitch;
//				if(channel[1] < -CHANNEL_DEAD_ZONE && pitch >15.0f)
//					pitch_target_angle += 30.0f;
//				else
//					pitch_target_angle += pitch -ANGLE_MIN_PITCH;
				if(channel[1] > CHANNEL_DEAD_ZONE)
					pitch_target_angle -= 30.0f;
				else
					pitch_target_angle -= ANGLE_MAX_PITCH - pitch;
				if(channel[1] < -CHANNEL_DEAD_ZONE)
					pitch_target_angle += 30.0f;
				last_trigger_time = current_time;
			}
			last = 0;
		}
	}
	/*串级PID*/
	pid_calculate(&pitch_angle_pid, pitch_target_angle, Pitch_IMU.total_angle);
	pid_calculate(&pitch_speed_pid, pitch_angle_pid.out, Pitch_IMU.speed);
	last = idle;
}

void CAN_Task(void)
{
	/*统一PID限幅*/
	yaw_angle_pid.out = Limit_Min_Max(yaw_angle_pid.out, VOLTAGE_MIN, VOLTAGE_MAX);
//	yaw_speed_pid.out = Limit_Min_Max(yaw_speed_pid.out, VOLTAGE_MIN, VOLTAGE_MAX);
	pitch_speed_pid.out = Limit_Min_Max(pitch_speed_pid.out, VOLTAGE_MIN, VOLTAGE_MAX);
	driver_speed_pid.out = Limit_Min_Max(driver_speed_pid.out, CURRENT_MIN, CURRENT_MAX);
	friction_left_pid.out = Limit_Min_Max(friction_left_pid.out, CURRENT_MIN, CURRENT_MAX);
	friction_right_pid.out = Limit_Min_Max(friction_right_pid.out, CURRENT_MIN, CURRENT_MAX);
	/*ID=4 Yaw GM6020*/							
	DecimalToHex(yaw_angle_pid.out, &message_can2[6], &message_can2[7]);
	/*ID=2 Pitch GM6020*/
	DecimalToHex(pitch_speed_pid.out, &message_can1_gm6020[2], &message_can1_gm6020[3]);
	/*ID=1 左摩擦轮 M3508*/
	DecimalToHex(friction_left_pid.out, &message_can1[0], &message_can1[1]);
	/*ID=2 右摩擦轮 M3508*/
	DecimalToHex(-friction_right_pid.out, &message_can1[2], &message_can1[3]);
	/*ID=3 拨盘电机 M2006 空载500rpm*/
	DecimalToHex(driver_speed_pid.out, &message_can1[4], &message_can1[5]);
	
  /*遥控保护*/
	uint8_t last_signal = CheckRemoteSignal();
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

/*角度计算*/
void total_angle_calculate(Angle_t *motor, float new_angle,float dt)
{
  if(motor->InitFlag)//初始化
	{
		motor->last_angle = new_angle;//上次角度
		motor->initial_angle = new_angle;//初始角度
		motor->InitFlag = 0;//初始化完毕
		return;//不进行后续计算
  }
	
  float delta = new_angle - motor->last_angle;//计算角度变化量(处理360度跳变)
  if(delta < -180.0f)//正向跨越360度
		delta += 360.0f;
  else if(delta > 180.0f)//反向跨越360度
		delta -= 360.0f;
	
//  motor->total_angle += delta;//更新累计角度
	
	motor->total_angle = new_angle - motor->initial_angle;//累计等于当前减初始
	motor->delta = delta;//增量
	motor->speed = motor->delta / dt;//角速度对时间微分
  motor->last_angle = new_angle;// 更新上一次角度值
}

/*遥控超时保护*/
uint8_t Remote_Task(void)
{
	GetValue();
	for(int i = 0; i < DR16_CHANNEL;i++)
	{
		if(channel[i] != last_channel[i])
		{
			changed_flag = true;
			timeout_counter = 0;
			break;
		}
	}
	if(changed_flag)
	{
		for(int i = 0; i < DR16_CHANNEL;i++)
		{
			last_channel[i] = channel[i];
		}
	}
	else 
		timeout_counter++;
	if(timeout_counter >= 10)
		return 1;
	else
		return 0;
}

/*遥控信号检测*/
uint8_t CheckRemoteSignal(void)
{
	/*未上电*/
	if(channel[4] == 0 && channel[5] == 0)
	 return 0;
	/*超时*/
	else if(Remote_Task())
		return 0;
	/*正常工作*/
	else
		return 1;
}

/*整合遥控器数据为数组 用于遥控保护*/
void GetValue(void)
{
	channel[0] = dr16.ch1;
	channel[1] = dr16.ch2;
	channel[2] = dr16.ch3;
	channel[3] = dr16.ch4;
	channel[4] = dr16.sw1;
	channel[5] = dr16.sw2;
}

void ValueAllInit(void)
{
	Yaw_IMU.InitFlag = 1;//yaw待初始化状态
	Yaw_IMU.target_init = 1;
	
	/*PID初始化*/
//	pid_init(&yaw_speed_pid, 0.0f, 0.0f, 0.0f);
//	pid_init(&yaw_angle_pid, 0.0f, 0.0f, 0.0f);

	/*角度单环*/
	pid_init(&yaw_speed_pid, 0.0f, 0.0f, 0.0f);
	pid_init(&yaw_angle_pid, 60.0f, 0.0f, 600.0f);
	yaw_angle_pid.kff = 9000.0f;
	
	pid_init(&pitch_speed_pid, 0.0f, 0.0f, 0.0f);
	pid_init(&pitch_angle_pid, 0.0f, 0.0f, 0.0f);
	
	pid_init(&pitch_speed_pid, 10.0f, 0.0f, 0.2f);
	pid_init(&pitch_angle_pid, 10.0f, 0.0f, 0.0f);
	
	pid_init(&driver_speed_pid, 0.0f, 0.0f, 0.0f);
	pid_init(&driver_angle_pid, 0.0f, 0.0f, 0.0f);
	
	pid_init(&friction_left_pid, 3.0f, 0.00100000005f, 0.449999988f);
	pid_init(&friction_right_pid, 3.0f, 0.00100000005f, 0.449999988f);
}

void Motor_Init(Angle_t *motor, float CurrentAngle)
{
	motor->InitFlag = 1;
	motor->total_angle = 0.0f;//累计角度置0
	motor->last_angle = CurrentAngle;//更新当前角度
	motor->delta = 0.0f;
	motor->speed = 0.0f;
}
