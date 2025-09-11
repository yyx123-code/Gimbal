#ifndef __CONTROL_H_
#define __CONTROL_H_

#include <stdint.h>
#include "calculate.h"

#define CHANNEL_DEAD_ZONE		200

typedef enum
{
	up,
	down
}Pitch_Mode_t;

void Yaw_Task(void);
void Pitch_Task(void);
//void Driver_Task(void);
void Friction_Task(void);

void PID_Init(void);
void Init_Task(void);
void CAN_Task(void);
void Motor_Init(void);

/*can收发数据*/
extern uint8_t message_can1[8];//friction driver
extern uint8_t message_can2[8];//yaw
extern uint8_t message_can1_gm6020[8];//pitch
/*欧拉角*/
extern float pitch;
extern float roll;
extern float yaw;
/*电机目标值*/
extern int16_t friction_left_speed;
extern int16_t friction_right_speed;
extern float yaw_target_angle;
extern float pitch_target_angle;
extern float driver_target_angle;

#endif
