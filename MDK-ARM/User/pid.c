#include "pid.h"
#include "calculate.h"
#include "math.h"

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
/*本文件代码优化空间：
 *提供位置PID和增量PID作为选择
 *提供拓展函数例如前馈
 */