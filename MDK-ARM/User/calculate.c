#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include "calculate.h"
#include "drv_can.h"

#define FILTER_ALPHA 0.9f // 滤波系数，取值范围0到1

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
   return filter_value = FILTER_ALPHA * new_value + (1 - FILTER_ALPHA) * filter_value;
}
