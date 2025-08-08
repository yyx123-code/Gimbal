#ifndef __DRV_CAN_H__
#define __DRV_CAN_H__

#include <stdint.h>
#include "stm32f4xx.h"                  // Device header

// 标准格式：0x200 + 电机ID（1-4）
#define M3508_CTRL_ID_BASE  0x200  // M3508控制指令基地址

typedef struct
{
  float mechanical_angle;  // 机械角度
  int16_t speed;             // 转速
  int16_t torque_current;    // 实际转矩电流
  uint8_t temperature;       // 电机温度
}MotorData_t;

void CAN1_Init(void);//can初始化
void CAN2_Init(void);
void CAN1_Send(uint8_t *message);//can发送函数
void CAN2_Send(uint8_t *message);
void CAN1_Receive(uint32_t *ID, uint8_t *Length, uint8_t *Data);
void CAN2_Receive(uint32_t *ID, uint8_t *Length, uint8_t *Data);
void ExtractMotorData_can1(uint8_t *RxData, MotorData_t *motor_data);
void ExtractMotorData_can2(uint8_t *RxData, MotorData_t *motor_data);

#endif
