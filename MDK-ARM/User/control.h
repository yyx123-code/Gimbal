#ifndef __CONTROL_H_
#define __CONTROL_H_

#include <stdint.h>
#include <stdbool.h>
#include "fly_ibus.h"
/*摩擦轮*/
#define CHANNEL2_MAX 			2000.0f
#define CHANNEL2_MIN 			1003.0f
#define CHANNEL2_DELTA 		CHANNEL2_MAX - CHANNEL2_MIN
#define CHANNEL_MIDDLE		1500.0f
/*遥控超时保护*/
extern bool changed_flag;
extern uint16_t timeout_counter;
extern uint16_t last_channel[IBUS_USER_CHANNLES];

/*can收发数据*/
extern uint8_t message_can1[8];//发送can1
extern uint8_t message_can2[8];//发送can2
/*欧拉角*/
extern float pitch;
extern float roll;
extern float yaw;

typedef struct
{
	float last_angle;
	float total_angle;
	float delta;
	float speed;
}IMU_t;

extern IMU_t Yaw_IMU;
extern IMU_t Pitch_IMU;

void Init_Task(void);
void Friction_Task(void);
void Yaw_Task(void);
void Pitch_Task(void);
void Driver_Task(void);
void CAN_Task(void);
void Deinit_Task(void);
void imu_calculate(IMU_t *motor, float new_angle, float dt);
uint8_t Remote_Task(void);
uint8_t CheckRemoteSignal(void);

#endif
