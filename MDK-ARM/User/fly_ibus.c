/*Include*/
#include "fly_ibus.h"
/*PV*/
uint8_t rx_buffer[32] = {0};//接收数据数组
uint16_t channel[IBUS_USER_CHANNLES] = {0};//通道
uint16_t checksum_cal, checksum_ibus;//校验
/*IBUS初始化*/
void IBUS_Init(void)
{
	HAL_UART_Receive_IT(IBUS_UART, rx_buffer, 32);//开启串口接收中断
}
/*读取数据*/
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
	HAL_UART_Receive_IT(IBUS_UART, rx_buffer, 32);//重新开启中断
}

/*中断接收回调函数*/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == IBUS_UART_INSTANCE)
	{
		IBUS_Read_Channles(IBUS_USER_CHANNLES);
	}
}
