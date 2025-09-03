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
/*IMU数据*/
IMU_t Yaw_IMU;
IMU_t Pitch_IMU;
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
float target_pos = 0;
uint8_t read_flag = 1;
/*摩擦轮反向pid重置*/
uint16_t fri_channel = 0;

/*初始化任务*/
void Init_Task(void)
{
	__disable_irq();//禁用全局中断
	/*MPU6050 CAN IBUS*/
	MPU6050_Init();
	uint8_t result0 = MPU_DMP_Init();
	__enable_irq();//启用所有中断
	CAN_Init();
	DBUS_Init();
	/*PID初始化*/
	pid_init(&yaw_speed_pid, 2.0f, 0.0f, 2.0f);
	pid_init(&yaw_angle_pid, 2.0f, 0.0f, 2.0f);
	
	pid_init(&pitch_up_speed_pid, 1.0f, 0.0f, 0.1f);
	pid_init(&pitch_up_angle_pid, 1.0f, 0.0f, 0.1f);
	
	pid_init(&pitch_down_speed_pid, 1.0f, 0.0f, 0.1f);
	pid_init(&pitch_down_angle_pid, 1.0f, 0.0f, 0.1f);
	
	pid_init(&driver_speed_pid, 1.0f, 0.0f, 0.8f);
	pid_init(&driver_angle_pid, 1.0f, 0.0f, 0.8f);
	
	pid_init(&friction_left_pid, 2.3f, 0.08f, 0.9f);
	pid_init(&friction_right_pid, 2.3f, 0.08f, 0.9f);
}

/*摩擦轮 左正右反*/
void Friction_Task(void)
{
	static float friction_left_speed = 0;
	static float friction_right_speed = 0;
	if(fri_channel > CHANNEL_DEAD_ZONE && channel[3] < CHANNEL_DEAD_ZONE)
	{
		pid_reset(&friction_left_pid);
		pid_reset(&friction_right_pid);
	}
	/*目标速度*/
	if(channel[3] > CHANNEL_DEAD_ZONE)
	{
		friction_left_speed = M3508_TARGET;
		friction_right_speed = -friction_left_speed;
	}
	else if (channel[3] < CHANNEL_DEAD_ZONE)
	{
		friction_left_speed = 0;
		friction_right_speed = 0;
	}
	fri_channel = channel[3];
	/*PID计算*/
	pid_calculate(&friction_left_pid, friction_left_speed, Friction_Left_Data.speed);
	pid_calculate(&friction_right_pid, friction_right_speed, Friction_Right_Data.speed);
}

/*Yaw*/
void Yaw_Task(void)
{
	/*模式切换*/
	if(channel[0] < CHANNEL_DEAD_ZONE && channel[0] > -CHANNEL_DEAD_ZONE)
	Yaw_Mode = IDLE_MODE;
	if(channel[0] > CHANNEL_DEAD_ZONE)
	Yaw_Mode = FORWARD;
	else if(channel[0] < -CHANNEL_DEAD_ZONE)
	Yaw_Mode = REVERSE;
	else
	Yaw_Mode = IDLE_MODE;
	switch(Yaw_Mode)
	{
		case IDLE_MODE:/*空闲状态 等待指令*/
			if(fabsf(yaw_target_angle - Yaw_IMU.total_angle) > 1.0f)//如果电机未到达目标位置
			{
				pid_calculate(&yaw_angle_pid, yaw_target_angle, Yaw_IMU.total_angle);
				pid_calculate(&yaw_speed_pid, yaw_angle_pid.out, Yaw_IMU.speed);
			}
			else
			{
				yaw_speed_pid.out = 0; // 停止电机
			}
			break;
		case FORWARD:
			/*设定目标值*/
			yaw_target_angle += ANGLE_MAX_YAW;
			/*串级PID*/
			pid_calculate(&yaw_angle_pid, yaw_target_angle, Yaw_IMU.total_angle);
			pid_calculate(&yaw_speed_pid, yaw_angle_pid.out, Yaw_IMU.speed);
			
			if(fabsf(Yaw_IMU.total_angle - yaw_target_angle) < 1.0f) // 如果电机到达目标位置
			{
				yaw_speed_pid.out = 0;//停止电机
				Yaw_Mode = IDLE_MODE;//切换到空闲模式
			}
			break;
		case REVERSE:
		/*设定目标值*/
		yaw_target_angle += ANGLE_MIN_YAW;
		/*串级PID*/
		pid_calculate(&yaw_angle_pid, yaw_target_angle, Yaw_IMU.total_angle);
		pid_calculate(&yaw_speed_pid, yaw_angle_pid.out, Yaw_IMU.speed);
		
		if(fabsf(Yaw_IMU.total_angle - yaw_target_angle) < 1.0f) // 如果电机到达目标位置
		{
			yaw_speed_pid.out = 0;//停止电机
			Yaw_Mode = IDLE_MODE;//切换到空闲模式
		}
		break;
	}
}

/*Pitch*/
void Pitch_Task(void)
{
	/*目标值设定*/
	if(channel[1] > CHANNEL_DEAD_ZONE)
	{Pitch_Mode = FORWARD;}
	else if(channel[1] < -CHANNEL_DEAD_ZONE)
	{Pitch_Mode = REVERSE;}
	else
	{Pitch_Mode = IDLE_MODE;}
	switch(Pitch_Mode)
	{
		case IDLE_MODE:/*空闲状态 等待指令*/
			if(fabsf(pitch_target_angle - Pitch_IMU.total_angle) > 1.0f)//如果电机未到达目标位置
			{
				if(pitch_target_angle == ANGLE_MAX_YAW)
				{
					pid_calculate(&pitch_up_angle_pid, pitch_target_angle, Pitch_IMU.total_angle);
					pid_calculate(&pitch_up_speed_pid, pitch_up_angle_pid.out, Pitch_IMU.speed);
					pitch_output = pitch_up_speed_pid.out;
				}
				else if(pitch_target_angle == ANGLE_MIN_YAW)
				{
					pid_calculate(&pitch_down_angle_pid, pitch_target_angle, Pitch_IMU.total_angle);
					pid_calculate(&pitch_down_speed_pid, pitch_down_angle_pid.out, Pitch_IMU.speed);
					pitch_output = pitch_down_speed_pid.out;
				}
			}
			else
			{
				pitch_up_speed_pid.out = 0;//停止电机
				pitch_down_speed_pid.out = 0;
			}
			break;
		case FORWARD:
			/*设定目标值*/
			pitch_target_angle += ANGLE_MAX_YAW;
			/*软限位(提前限制目标值)*/
			pitch_target_angle = fmaxf(ANGLE_MIN_PITCH, fminf(pitch_target_angle, ANGLE_MAX_PITCH));
			/*串级PID*/
			pid_calculate(&pitch_up_angle_pid, pitch_target_angle, Pitch_IMU.total_angle);
			pid_calculate(&pitch_up_speed_pid, pitch_up_angle_pid.out, Pitch_IMU.speed);
			
			if(fabsf(Pitch_IMU.total_angle - pitch_target_angle) < 1.0f) // 如果电机到达目标位置
			{
				pitch_up_speed_pid.out = 0;//停止电机
				Pitch_Mode = IDLE_MODE;//切换到空闲模式
			}
			pitch_output = pitch_up_speed_pid.out;
			break;
		case REVERSE:
			/*设定目标值*/
			pitch_target_angle += ANGLE_MIN_YAW;
			/*软限位(提前限制目标值)*/
			pitch_target_angle = fmaxf(ANGLE_MIN_PITCH, fminf(pitch_target_angle, ANGLE_MAX_PITCH));
			/*串级PID*/
			pid_calculate(&pitch_down_angle_pid, pitch_target_angle, Pitch_IMU.total_angle);
			pid_calculate(&pitch_down_speed_pid, pitch_down_angle_pid.out, Pitch_IMU.speed);
			
			if(fabsf(Pitch_IMU.total_angle - pitch_target_angle) < 1.0f) // 如果电机到达目标位置
			{
				pitch_down_speed_pid.out = 0;//停止电机
				Pitch_Mode = IDLE_MODE;//切换到空闲模式
			}
			pitch_output = pitch_down_speed_pid.out;
		break;
	}
	/*动态死区参数*/
	float base_deadzone = 0.5f;
	float dynamic_range = 2.0f;
	float sensitivity = 5.0f;
//	/*动态死区*/
//	apply_dynamic_deadzone(&pitch_up_speed_pid, base_deadzone, dynamic_range, sensitivity);
//	apply_dynamic_deadzone(&pitch_down_speed_pid, base_deadzone, dynamic_range, sensitivity);
}

/*拨盘电机*/
void Driver_Task(void)
{
	/*判断模式*/
	if(channel[2] < CHANNEL_DEAD_ZONE)
	{Driver_Mode = SINGLE_SHOT;}//单发
	else if(channel[2] > CHANNEL_DEAD_ZONE)
	{Driver_Mode = AUTO_FIRE;}//连发
	else
	{Driver_Mode = DRIVER_IDLE;}//空闲
	/*多圈可采用 拨盘电机试用*/
	if(fabsf(driver_angle_pid.err[0]) > FAST_ZONE)
	{
		/* 远距离模式：激进控制 */
		driver_angle_pid.kp = 3.0f;  // 高比例增益
		driver_angle_pid.kd = 0.5f;  // 基础微分
		driver_angle_pid.ki = 0.0f;  // 禁用积分
	}
	if(fabsf(driver_angle_pid.err[0]) > BRAKE_ZONE)
	{
	/* 过渡模式：平滑减速 */
		driver_angle_pid.kp = 1.5f;
		driver_angle_pid.kd = 2.0f;  // 增强微分制动
		driver_angle_pid.ki = 0.02f; // 微量积分
	}
	else
	{
		/* 精定位模式：强阻尼 */
		driver_angle_pid.kp = 0.8f;
		driver_angle_pid.kd = 5.0f;  // 最大微分制动
		driver_angle_pid.ki = 0.0f;  // 禁用积分
		driver_angle_pid.i_out = 0.0f; // 重置历史积分
	}//误差越小，死区越大
	/*Test*/
	switch(Driver_Mode)
	{
		case DRIVER_IDLE:/*空闲状态 等待指令*/
			if(fabsf(target_pos - driver_angle.total_angle) > 1.0f)//如果电机未到达目标位置
			{
				pid_calculate(&driver_angle_pid, target_pos, driver_angle.total_angle);
				pid_calculate(&driver_speed_pid, driver_angle_pid.out, Driver_Data.speed);
			}
			else
			{
				driver_speed_pid.out = 0; // 停止电机
			}
			break;
		case SINGLE_SHOT:
			target_pos += ONE_SHOT;
			/*计算总共转过的角度*/
			total_angle_calculate(&driver_angle, Driver_Data.mechanical_angle);
			
			pid_calculate(&driver_angle_pid, target_pos, driver_angle.total_angle);
			pid_calculate(&driver_speed_pid, driver_angle_pid.out, Driver_Data.speed);
			
			if(fabsf(driver_angle.total_angle - target_pos) < 1.0f) // 如果电机到达目标位置
			{
				driver_speed_pid.out = 0;//停止电机
				Driver_Mode = DRIVER_IDLE;//切换到空闲模式
			}
			break;
			
		case AUTO_FIRE:
			target_pos += ONE_SHOT;
			total_angle_calculate(&driver_angle, Driver_Data.mechanical_angle);
			
			/*串级PID*/
			pid_calculate(&driver_angle_pid, target_pos, driver_angle.total_angle);
			pid_calculate(&driver_speed_pid, driver_angle_pid.out, Driver_Data.speed);
			
			if(fabsf(driver_angle.total_angle - target_pos) < 1.0f)//如果电机到达目标位置
			{
				target_pos += ONE_SHOT;//更新目标位置,继续下一次装弹
			}
			
			if(channel[2] < 1600)//如果遥控器指令结束
			{
				Driver_Mode = DRIVER_IDLE; // 切换到空闲模式
			}
			break;
	}
}

void CAN_Task(void)
{
	/*统一PID限幅*/
	yaw_speed_pid.out = Limit_Min_Max(yaw_speed_pid.out, VOLTAGE_MIN, VOLTAGE_MAX);
	pitch_up_speed_pid.out = Limit_Min_Max(pitch_up_speed_pid.out, VOLTAGE_MIN, VOLTAGE_MAX);
	pitch_down_speed_pid.out = Limit_Min_Max(pitch_down_speed_pid.out, VOLTAGE_MIN, VOLTAGE_MAX);
	driver_speed_pid.out = Limit_Min_Max(driver_speed_pid.out, CURRENT_MIN, CURRENT_MAX);
	friction_left_pid.out = Limit_Min_Max(friction_left_pid.out, CURRENT_MIN, CURRENT_MAX);
	friction_right_pid.out = Limit_Min_Max(friction_right_pid.out, CURRENT_MIN, CURRENT_MAX);
	/*ID=4 Yaw GM6020*/							
	DecimalToHex(yaw_speed_pid.out, &message_can2[6], &message_can2[7]);
	/*ID=2 Pitch GM6020*/
	DecimalToHex(pitch_output, &message_can1_gm6020[2], &message_can1_gm6020[3]);
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
			CAN_Transmit();
			break;
		case 0:
//			if(last_signal - CheckRemoteSignal())//启用可能影响拨盘位置环
//			{
//				EmergencyBrake();
//				CAN_Transmit();
//			}
//			else
//				break;
			break;
	}
}

/*重新初始*/
void Deinit_Task(void)
{
	if(HAL_CAN_GetState(&hcan1) == HAL_CAN_STATE_ERROR)
	{
		HAL_CAN_Stop(&hcan1);
		MX_CAN1_Init();
	}
	if(HAL_CAN_GetState(&hcan2) == HAL_CAN_STATE_ERROR)
	{
		HAL_CAN_Stop(&hcan2);
		MX_CAN2_Init(); // 重新初始
	}
}

/*数据使用MPU6050替换*/
void imu_calculate(IMU_t *motor, float new_angle,float dt)
{
  static uint8_t flag = 1; 
  if(flag)
	{
      motor->last_angle = new_angle;
      flag = 0;
  }//首次运行
  motor->delta = new_angle - motor->last_angle;
  if(motor->delta < -180.0f)
		motor->delta += 360.0f; 
  else if(motor->delta > 180.0f)
		motor->delta -= 360.0f;
  motor->total_angle += motor->delta;//积分转过的角度
	motor->speed = motor->delta / dt;//角速度对时间微分
	motor->last_angle = new_angle;//更新
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

/*初始化角度*/
void Set_Value(void)
{
	if(read_flag)
	{
		yaw_target_angle = yaw;
		pitch_target_angle = pitch;
		target_pos = Driver_Data.mechanical_angle;
		read_flag = 0;
	}
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

/*整合can发送*/
void CAN_Transmit(void)
{
	CAN_Send(UNIVERSAL_ID, message_can1);
	CAN_Send(PITCH_ID, message_can1_gm6020);
	CAN_Send(YAW_ID, message_can2);
}

/*紧急制动*/
void EmergencyBrake(void)
{
	for(int i = 0; i < 8; i++ )
	{
		message_can1[i] = 0;
		message_can2[i] = 0;
		message_can1_gm6020[i] = 0;
	}
}