#include <stdint.h>
#include <stdio.h>
#include "calculate.h"

#define FILTER_ALPHA 0.3f // 滤波系数，取值范围0到1

/**
 * @brief 将一个16位的十进制数拆分为高八位和低八位
 * @param value 十进制输入值
 * @param high_byte 输出高八位
 * @param low_byte 输出低八位
 */
void DecimalToHex(int16_t value, uint8_t *high_byte, uint8_t *low_byte)
{
    // 将输入值转换为无符号类型，避免符号扩展问题
    uint16_t unsigned_value = (uint16_t)value;

    // 提取高八位
    *high_byte = (unsigned_value >> 8) & 0xFF;

    // 提取低八位
    *low_byte = unsigned_value & 0xFF;
}
/**
 * @brief 十六进制转十进制
 * @param high_byte 输入高八位
 * @param low_byte 入低八位
 */
int16_t HexToDecimal(uint8_t high_byte, uint8_t low_byte)
{
    return (int16_t)((high_byte << 8) | low_byte);
}

/**
 * @brief 限幅函数
 * @param output_data 输出数据
 * @param min_limit 最小限幅值
 * @param max_limit 最大限幅值
 * @return 返回限幅后的结果
 */
int32_t Limit_Min_Max(int32_t output_data, int32_t min_limit, int32_t max_limit)
{
    // 如果输出数据小于最小限幅值，则返回最小限幅值
    if (output_data < min_limit)
    {
        return min_limit;
    }
    // 如果输出数据大于最大限幅值，则返回最大限幅值
    else if (output_data > max_limit)
    {
        return max_limit;
    }
    // 否则，直接返回输出数据
    else
    {
        return output_data;
    }
}

/**
 * @brief 一阶低通滤波函数
 * @param new_angle新速度/角度
 * @return 返回滤波后结果
 */
static float filter_value = 0; // 滤波后的角度值

float LowPassFilter(float new_value)
{
    filter_value = FILTER_ALPHA * new_value + (1 - FILTER_ALPHA) * filter_value;
    return filter_value;
}

int rc_to_motor(int rc_value)
{
    // 遥控器范围
    const int RC_MIN = 1000;
    const int RC_MAX = 2000;

    // 电机速度范围
    const int MOTOR_MIN = -30;
    const int MOTOR_MAX = 30;

    // 计算映射后的电机速度
    int motor_speed = (rc_value - RC_MIN) * (MOTOR_MAX - MOTOR_MIN) / (RC_MAX - RC_MIN) + MOTOR_MIN;

    // 限制输出范围（防止超出范围）
    if (motor_speed < MOTOR_MIN) motor_speed = MOTOR_MIN;
    if (motor_speed > MOTOR_MAX) motor_speed = MOTOR_MAX;

    return motor_speed;
}

/*摩擦轮专属遥控通道值转速度*/
int rc_to_friction(int rc_value)
{
    // 遥控器范围
    const int RC_MIN = 1000;
    const int RC_MAX = 2000;

    // 电机速度范围
    const int MOTOR_MIN = 0;
    const int MOTOR_MAX = 8911;

    // 计算映射后的电机速度
    int motor_speed = (rc_value - RC_MIN) * (MOTOR_MAX - MOTOR_MIN) / (RC_MAX - RC_MIN) + MOTOR_MIN;

    // 限制输出范围（防止超出范围）
    if (motor_speed < MOTOR_MIN) motor_speed = MOTOR_MIN;
    if (motor_speed > MOTOR_MAX) motor_speed = MOTOR_MAX;

    return motor_speed;
}
/*遥控通道值转拨盘*/
int rc_to_driver(int rc_value)
{
    // 遥控器范围
    const int RC_MIN = 1000;
    const int RC_MAX = 2000;

    // 电机速度范围
    const int MOTOR_MIN = -500;
    const int MOTOR_MAX = 500;

    // 计算映射后的电机速度
    int motor_speed = (rc_value - RC_MIN) * (MOTOR_MAX - MOTOR_MIN) / (RC_MAX - RC_MIN) + MOTOR_MIN;

    // 限制输出范围（防止超出范围）
    if (motor_speed < MOTOR_MIN) motor_speed = MOTOR_MIN;
    if (motor_speed > MOTOR_MAX) motor_speed = MOTOR_MAX;

    return motor_speed;
}

/**
 * @brief 实际转过角度累计(用于角度环多圈处理)
* @param Angle_t *motor电机结构体指针
 * @param new_angle新角度(0-360)
 * @return total_angle返回转过角度
 */
void total_angle_calculate(Angle_t *motor, float new_angle)
{
  static uint8_t first_run = 1;      // 首次运行标志 
  // 第一次调用时初始化
  if(first_run) {
      motor->last_angle = new_angle;
      motor->total_angle = 0;
      first_run = 0;
  }
  // 计算角度变化量(处理360度跳变)
  float delta = new_angle - motor->last_angle;
  // 处理正向跨越360度的情况
  if(delta < -180.0f)
	{delta += 360.0f;} 
  // 处理反向跨越360度的情况
  else if(delta > 180.0f)
	{delta -= 360.0f;}
  // 更新累计角度
  motor->total_angle += delta;
  // 更新上一次角度值
  motor->last_angle = new_angle;
}
/*本文件代码优化空间：
 *遥控通道值转控制量合并
 */