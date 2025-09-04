#ifndef __CONTROL_H_
#define __CONTROL_H_

#include <stdint.h>
#include <stdbool.h>
/*遥控通道中间值*/
//#define CHANNEL_MIDDLE		1500.0f
#define DR16_CHANNEL				6
#define CHANNEL_DEAD_ZONE		200
#define DEFAULT							10

typedef struct
{
	float last_angle;
	float total_angle;
	float delta;
	float speed;
}Angle_t;

void Init_Task(void);
void Friction_Task(void);
void Yaw_Task(void);
void Pitch_Task(void);
void Driver_Task(void);
void CAN_Task(void);
//void Deinit_Task(void);
void total_angle_calculate(Angle_t *motor, float new_angle,float dt);
void driver_angle_calculate(Angle_t *motor, float new_angle);
uint8_t Remote_Task(void);
uint8_t CheckRemoteSignal(void);
void GetValue(void);
void CAN_Transmit(void);
void ValueAllInit(void);
void Motor_Init(Angle_t *imu, float initial_angle);

/*遥控超时保护*/
extern bool changed_flag;
extern uint16_t timeout_counter;
//extern uint16_t last_channel[IBUS_USER_CHANNLES];
/*can收发数据*/
extern uint8_t message_can1[8];//friction driver
extern uint8_t message_can2[8];//yaw
extern uint8_t message_can1_gm6020[8];//pitch
/*欧拉角*/
extern float pitch;
extern float roll;
extern float yaw;
extern uint8_t read_flag;
/*IMU数据*/
extern Angle_t Yaw_IMU;
extern Angle_t Pitch_IMU;

extern int16_t friction_left_speed;
extern int16_t friction_right_speed;
/*角度结构体*/
extern Angle_t yaw_angle;
extern Angle_t pitch_angle;
extern Angle_t driver_angle;
#endif
