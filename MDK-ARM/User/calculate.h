#ifndef __CALCULATE_H_
#define __CALCULATE_H_

#include <stdint.h>

void DecimalToHex(int16_t value, uint8_t *high_byte, uint8_t *low_byte);
int16_t HexToDecimal(uint8_t high_byte, uint8_t low_byte);
int32_t Limit_Min_Max(int32_t output_data, int32_t min_limit, int32_t max_limit);
float LowPassFilter(float new_value);

#endif
