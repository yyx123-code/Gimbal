#ifndef __FLY_IBUS_H_
#define __FLY_IBUS_H_

#include "usart.h"
#include "main.h"

//通道
#define IBUS_UART							(&huart4)
#define IBUS_UART_INSTANCE		(UART4)
#define IBUS_USER_CHANNLES		10					//Use 6 channels
//帧字节
#define IBUS_LENGTH						0x20				//32 bytes
#define IBUS_COMMAND					0x40				//set servo/motor
#define IBUS_MAX_CHANNLES			14
/*防止接收器甩飞*/
#define REMOTE_TIMEOUT_MS 3000 // 3秒超时

void IBUS_Init(void);
void IBUS_Read_Channles(uint8_t user_channels);

/*遥控数据*/
extern uint16_t channel[IBUS_USER_CHANNLES];

#endif
