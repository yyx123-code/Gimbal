#ifndef __PID_H_
#define __PID_H_

#include "calculate.h"

#define FAST_ZONE     30.0f  // 快速响应区阈值(度)
#define BRAKE_ZONE    10.0f  // 制动区阈值(度)

/*两种PID模式*/
typedef enum
{
	PID_POSITION,    //位置式PID
	PID_INCREMENTAL  //增量式PID
}pid_mode_t;

typedef struct
{
  float kp;
	float ki;
	float kd;
	
	float target_value;//目标值
	float feedback_value;//返回值
	float err[3];//误差值
	
	float p_out;
	float i_out;
	float d_out;
	float out;
	
	/*加入前馈*/
	float feedforward;
	float feedforward_out;
	
	/*加入增量式和位置式PID供选择*/
	pid_mode_t mode;
	
	/*加入限幅和死区*/
	float i_max;          // 积分项限幅值
	float dead_zone;      // 误差死区（小于此值视为0）

	/*加入启动及制动补偿*/
	float startup;      	// 启动补偿系数
  float startup_zone;   // 启动补偿阈值
	float brake;        	// 制动补偿系数
	float brake_zone;   	// 制动判断阈值
	uint8_t braking;      // 制动状态标志
	float last_target;		//上次目标值
}pid_t;

void pid_init(pid_t *pid, float kp, float ki, float kd, float feedforward, pid_mode_t mode);
void pid_calculate(pid_t *pid,float target,float current);
void pid_reset(pid_t *pid);
void pid_set_compensation(pid_t *pid, float startup, float startup_zone, float brake, float brake_zone, float dead_zone, float i_max);
float calculate_dynamic_deadzone(float error, float base_deadzone, float dynamic_range, float sensitivity);
void apply_dynamic_deadzone(pid_t *pid, float base_deadzone, float dynamic_range, float sensitivity);

extern pid_t yaw_speed_pid;
extern pid_t yaw_angle_pid;
extern pid_t driver_speed_pid;
extern pid_t driver_angle_pid;
extern pid_t friction_left_pid;
extern pid_t friction_right_pid;
extern pid_t pitch_up_speed_pid;
extern pid_t pitch_up_angle_pid;
extern pid_t pitch_down_speed_pid;
extern pid_t pitch_down_angle_pid;

#endif
