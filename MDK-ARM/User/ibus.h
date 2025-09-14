#ifndef __IBUS_H_
#define __IBUS_H_

#include "usart.h"
#include "main.h"

//通道
#define IBUS_UART							(&huart1)
#define IBUS_UART_INSTANCE		(USART1)
#define IBUS_USER_CHANNLES		10					//Use 6 channels
//帧字节
#define IBUS_LENGTH						0x20				//32 bytes
#define IBUS_COMMAND					0x40				//set servo/motor
#define IBUS_MAX_CHANNLES			14
#define IBUS_CHANNEL_MID			1500

void IBUS_Init(void);
void IBUS_Read_Channles(uint8_t user_channels);
uint8_t Remote_Task(void);
uint8_t CheckRemoteSignal(void);

extern uint16_t channel[IBUS_USER_CHANNLES];

#endif
