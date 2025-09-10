#ifndef __CALCULATE_H_
#define __CALCULATE_H_

#include <stdint.h>

/*typedef*/
typedef struct
{
	float last_angle;//上次角度
	float total_angle;//累计角度
	float delta;//增量
	float speed;//速度
	float initial_angle;//初始角度
	uint8_t read_flag;//首次初始
}Angle_t;

void DecimalToHex(int16_t value, uint8_t *high_byte, uint8_t *low_byte);
int16_t HexToDecimal(uint8_t high_byte, uint8_t low_byte);
int32_t Limit_Min_Max(int32_t output_data, int32_t min_limit, int32_t max_limit);
float LowPassFilter(float new_value);
void total_angle_calculate(Angle_t *motor, float new_angle);

extern Angle_t yaw_angle;
extern Angle_t pitch_angle;
extern Angle_t driver_angle;

#endif
