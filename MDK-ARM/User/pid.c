#include "pid.h"
#include "math.h"

/*PID结构体*/
pid_t yaw_speed_pid;
pid_t pitch_speed_pid;
pid_t driver_speed_pid;
pid_t yaw_angle_pid;
pid_t pitch_angle_pid;
pid_t driver_angle_pid;
pid_t friction_left_pid;
pid_t friction_right_pid;

//pid参数初始化
void pid_init(pid_t *pid, float kp, float ki, float kd, float feedforward, pid_mode_t mode)
{
	pid->kp=kp;
	pid->ki=ki;
  pid->kd=kd;
	pid->feedforward=feedforward;
	pid->mode=mode;
}

/*PID计算函数*/
void pid_calculate(pid_t *pid,float target,float current)
{
  pid->target_value = target;
	pid->feedback_value = current;
	pid->err[1] = pid->err[0]; //上次误差
	pid->err[0] = pid->target_value - pid->feedback_value; //这次误差
	
	if(fabsf(pid->err[0]) < pid->dead_zone){pid->err[0] = 0;}//小误差直接忽略
	
	pid->feedforward_out = target * pid->feedforward;//前馈控制
	
	/*制动状态检测*/
	float speed_change = target - pid->last_target;
	/*速度快速下降*/
	if(speed_change < -pid->brake_zone)
	{pid->braking = 1;}
	/*停止*/
	else
	{pid->braking = 0;}
	
	/*启动补偿 (目标速度>0且当前速度很低)*/
	if(target > 3.0f && fabsf(current) < pid->startup_zone)
	{pid->out += pid->startup * target;}
	
	/*制动补偿*/
	else if(pid->braking)
	{pid->out -= pid->brake * fabsf(speed_change);}
	
	/*位置式*/
	if(pid->mode == PID_POSITION)
	{
		pid->p_out = pid->kp * pid->err[0];//比例
		pid->i_out += pid->ki * pid->err[0];//积分
		pid->d_out = pid->kd * (pid->err[0]-pid->err[1]);//微分
		
		Limit_Min_Max(pid->i_out, -pid->i_max, pid->i_max);
		
		pid->out = pid->p_out + pid->i_out + pid->d_out + pid->feedforward_out;
	}
	/*增量式*/
	else
	{
		float delta = pid->kp * (pid->err[0] - pid->err[1]) 
				+ pid->ki * pid->err[0]
				+ pid->kd * (pid->err[0] - 2*pid->err[1] + pid->err[2]);
        pid->out += delta;//输出增量
        pid->err[2] = pid->err[1];//保存上上次误差
	}
	/*抗饱和*/
	if(fabsf(pid->out) > 20000)//接近输出限幅时衰减积分项
	{pid->i_out *= 0.5f;}
	
	/*保存上次目标值*/
	pid->last_target = target;
	
	/*输出限幅*/
  Limit_Min_Max(pid->out, -25000,25000); //输出限幅
}
/*调参自行设置积分限幅和死区
 *i_max;          			// 积分项限幅值
 *float dead_zone;      // 误差死区
 *float startup;      	// 启动补偿系数
 *float startup_zone;   // 启动补偿阈值
 *float brake;        	// 制动补偿系数
 *float brake_zone;   	// 制动判断阈值
 */

void pid_reset(pid_t *pid)
{
    pid->err[0] = pid->err[1] = pid->err[2] = 0;
    pid->i_out = 0;  // 清空积分项
    pid->out = 0;    // 重置输出
}

/*设置补偿参数*/
void pid_set_compensation(pid_t *pid, float startup, float startup_zone, float brake, float brake_zone, float dead_zone, float i_max)
{
	pid->startup = startup;
	pid->startup_zone = 
	pid->brake = brake;
	pid->brake_zone = brake_zone;
	pid->dead_zone = dead_zone;
	pid->i_max = i_max;
}

float calculate_dynamic_deadzone(float error, float base_deadzone, float dynamic_range, float sensitivity)
{
    return base_deadzone + dynamic_range * expf(-fabsf(error) / sensitivity);
}

void apply_dynamic_deadzone(pid_t *pid, float base_deadzone, float dynamic_range, float sensitivity)
{
    float dynamic_deadzone = calculate_dynamic_deadzone(pid->err[0], base_deadzone, dynamic_range, sensitivity);
    if (fabsf(pid->err[0]) < dynamic_deadzone)
    {
        pid->out = 0; // 完全停止输出
    }
}
