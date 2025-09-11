#include "pid.h"
#include "calculate.h"
#include "math.h"

pid_t yaw_speed_pid, yaw_angle_pid;
pid_t pitch_speed_pid, pitch_angle_pid;
pid_t driver_speed_pid, driver_angle_pid;
pid_t friction_left_pid, friction_right_pid;
pid_t pitch_speed_pid_down, pitch_angle_pid_down;

//pid参数初始化
void pid_init(pid_t *pid, float kp, float ki, float kd)
{
	pid->kp=kp;
	pid->ki=ki;
  pid->kd=kd;
}

//速度环
void pid_calculate(pid_t *pid,float target,float current)
{
  pid->target_value = target;
	pid->feedback_value = current;
	pid->err[1] = pid->err[0]; //上次误差
	pid->err[0] = pid->target_value - pid->feedback_value; //这次误差
	
	pid->p_out = pid->kp * pid->err[0]; //比例
	pid->i_out += pid->ki * pid->err[0]; //积分
	pid->d_out = pid->kd * (pid->err[0]-pid->err[1]);  //微分
	pid->out = pid->p_out + pid->i_out + pid->d_out;
	
	if(fabsf(pid->out) > 20000) {  // 接近输出限幅时
        pid->i_out *= 0.5f;        // 衰减积分项
    }

  Limit_Min_Max(pid->out, -25000,25000); //输出限幅
}

void pid_reset(pid_t *pid)
{
    pid->err[0] = pid->err[1] = 0;
    pid->i_out = 0;  // 清空积分项
    pid->out = 0;    // 重置输出
}