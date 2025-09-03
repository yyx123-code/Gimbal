#ifndef __PID_H_
#define __PID_H_

#define FAST_ZONE     30.0f  // 快速响应区阈值(度)
#define BRAKE_ZONE    10.0f  // 制动区阈值(度)

typedef struct
{
  float kp;
	float ki;
	float kd;
	
	float target_value;//目标值
	float feedback_value;//返回值
	float err[2];//误差值
	
	float p_out;
	float i_out;
	float d_out;
	float out;

}pid_t;


void pid_init(pid_t *pid, float kp, float ki, float kd);
void pid_calculate(pid_t *pid,float target,float current);
void pid_reset(pid_t *pid);

extern pid_t yaw_speed_pid, yaw_angle_pid;
extern pid_t pitch_up_speed_pid, pitch_up_angle_pid;
extern pid_t pitch_down_speed_pid, pitch_down_angle_pid;
extern pid_t driver_speed_pid, driver_angle_pid;
extern pid_t friction_left_pid, friction_right_pid;



#endif
