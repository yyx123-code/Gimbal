#include "ibus.h"
#include <stdbool.h>

uint8_t rx_buffer[32] = {0};
uint16_t channel[IBUS_USER_CHANNLES] = {0};
uint16_t checksum_cal, checksum_ibus;
bool changed_flag = false;
uint16_t timeout_counter = 0;
uint16_t last_channel[IBUS_USER_CHANNLES];

void IBUS_Init()
{
	HAL_UART_Receive_IT(IBUS_UART, rx_buffer, 32);
}

void IBUS_Read_Channles(uint8_t user_channels)
{
	uint16_t channel_buffer[IBUS_MAX_CHANNLES] = {0};

	if(rx_buffer[0] == IBUS_LENGTH && rx_buffer[1] == IBUS_COMMAND)
	{
		checksum_cal = 0xffff - rx_buffer[0] - rx_buffer[1];

		for(int i = 0; i < IBUS_MAX_CHANNLES; i++)
		{
			channel_buffer[i] = (uint16_t)(rx_buffer[i * 2 + 3] << 8 | rx_buffer[i * 2 + 2]);
			checksum_cal = checksum_cal - rx_buffer[i * 2 + 3] - rx_buffer[i * 2 + 2];
		}

		checksum_ibus = rx_buffer[31] << 8 | rx_buffer[30];

		if(checksum_cal == checksum_ibus)
		{
			for(int j = 0; j < user_channels; j++)
			{
				channel[j] = channel_buffer[j];
			}
		}
	}

	HAL_UART_Receive_IT(IBUS_UART, rx_buffer, 32);
}

/*中断接收回调函数*/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == IBUS_UART_INSTANCE)
	{
		IBUS_Read_Channles(IBUS_USER_CHANNLES);
	}
}

uint8_t Remote_Task(void)
{
	for(int i = 0; i < IBUS_USER_CHANNLES;i++)
	{
		if(channel[i] != last_channel[i])
		{
			changed_flag = true;
			timeout_counter = 0;
			break;
		}
	}
	if(changed_flag)
	{
		for(int i = 0; i < IBUS_USER_CHANNLES;i++)
		{
			last_channel[i] = channel[i];
		}
	}
	else 
	{timeout_counter++;}
	if(timeout_counter >= 10)
	{return 1;}
	else
	{return 0;}
}

/*遥控信号检测*/
uint8_t CheckRemoteSignal(void)
{
	if(channel[1] == 1501 && channel[2] == 1500 && channel[3] == 1498)//遥控数据跳变
		return 0;
	else if(channel[0] == 0 &&channel[1] == 0)//未上电
		return 0;
	else if(Remote_Task())//超时
		return 0;
	else
		return 1;
}