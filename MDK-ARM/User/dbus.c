/*------------------Include-------------------*/
#include "dbus.h"
#include "usart.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "queue.h"
#include "main.h"
#include <stdlib.h>  //abs()
#include <string.h>  //memset()
#include <math.h>    //abs()
/* -------------- Internal Data ------------- */ 
volatile uint8_t SbusRxBuffer[RC_FRAME_LENGTH];
volatile uint8_t Ready = 0;
uint8_t transflag = 0;
rc_info_t dr16;
/* --------------- Extern Define ------------ */
extern osMessageQueueId_t DR16Handle;
extern DMA_HandleTypeDef hdma_usart2_rx;
/* ----------- Function Implements ---------- */ 
static HAL_StatusTypeDef DMA_Rec(UART_HandleTypeDef* huart, uint8_t* pData, uint32_t Size);

/*初始化*/
void DBUS_Init(void)
{
	__HAL_UART_CLEAR_IDLEFLAG(&DBUS_UART);//清除UART空闲标志
	__HAL_UART_ENABLE_IT(&DBUS_UART, UART_IT_IDLE);//使能空闲中断
	HAL_StatusTypeDef status = DMA_Rec(&DBUS_UART, (uint8_t*)SbusRxBuffer, RC_FRAME_LENGTH);
	__HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);//禁用半传输中断
	if(status != HAL_OK)
		Error_Handler();
}

/*空闲中断回调函数*/
void HAL_UART_IdleCallBack(UART_HandleTypeDef *huart)
{
	if(huart->Instance == DBUS_UART.Instance)
	{
		__HAL_UART_CLEAR_IDLEFLAG(huart);//清除空闲标志
		HAL_UART_DMAStop(huart);//停止当前DMA传输
		uint16_t received_len = RC_FRAME_LENGTH - __HAL_DMA_GET_COUNTER(huart->hdmarx);//计算接收到的数据长度
		if(received_len == RC_FRAME_LENGTH)
		{
			Ready = 1;//设置数据就绪标志
			DR16_Data(&dr16,(uint8_t*)SbusRxBuffer);
			BaseType_t xHigherPriorityTaskWoken = pdFALSE;
//			BaseType_t TransRes = xQueueSendFromISR(DR16Handle, (void*)SbusRxBuffer, &xHigherPriorityTaskWoken);
//			if(TransRes == pdTRUE)
//				transflag = 1;
			HAL_UART_Receive_DMA(huart, (uint8_t*)SbusRxBuffer, RC_FRAME_LENGTH);//重新启动DMA
			portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
		}
		else
			HAL_UART_Receive_DMA(huart, (uint8_t*)SbusRxBuffer, RC_FRAME_LENGTH);//重新启动
	}
}

/*错误处理回调*/
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == DBUS_UART.Instance)
	{
		if(huart->ErrorCode & HAL_UART_ERROR_ORE)
		{
			__HAL_UART_CLEAR_OREFLAG(huart);
			HAL_UART_Receive_DMA(huart, (uint8_t*)SbusRxBuffer, RC_FRAME_LENGTH);
		}
	}
}

/*DMA接收数据*/
static HAL_StatusTypeDef DMA_Rec(UART_HandleTypeDef* huart, uint8_t* pData, uint32_t Size)
{
	if(huart->RxState == HAL_UART_STATE_READY)//UART就绪
	{
		if ((pData == NULL) || (Size == 0))
			return HAL_ERROR;
		
		return HAL_UART_Receive_DMA(&DBUS_UART, pData, Size);
	}
		return HAL_BUSY;
}

/*dr16数据解析*/
void DR16_Data(rc_info_t *rc, uint8_t *buff)
{
	static rc_info_t last_rc_info;//保存上一次数据
	memcpy(&last_rc_info, rc, sizeof(rc_info_t));
  
  //数据的分离和拼接,减去1024让数据的中间值变为0
  rc->ch1 = (buff[0] | buff[1] << 8) & 0x07FF;
  rc->ch1 -= 1024;
  rc->ch2 = (buff[1] >> 3 | buff[2] << 5) & 0x07FF;
  rc->ch2 -= 1024;
  rc->ch3 = (buff[2] >> 6 | buff[3] << 2 | buff[4] << 10) & 0x07FF;
  rc->ch3 -= 1024;
  rc->ch4 = (buff[4] >> 1 | buff[5] << 7) & 0x07FF;
  rc->ch4 -= 1024;
  /* prevent remote control zero deviation *//*防止数据零漂,设置正负5的死区*/
  if(rc->ch1 <= 5 && rc->ch1 >= -5)
    rc->ch1 = 0;
  if(rc->ch2 <= 5 && rc->ch2 >= -5)
    rc->ch2 = 0;
  if(rc->ch3 <= 5 && rc->ch3 >= -5)
    rc->ch3 = 0;
  if(rc->ch4 <= 5 && rc->ch4 >= -5)
    rc->ch4 = 0;
  
  rc->sw1 = ((buff[5] >> 4) & 0x000C) >> 2;
  rc->sw2 = (buff[5] >> 4) & 0x0003;
  
  //防止数据溢出
  if ((abs(rc->ch1) > 660) || \
      (abs(rc->ch2) > 660) || \
      (abs(rc->ch3) > 660) || \
      (abs(rc->ch4) > 660))
  {
    memset(rc, 0, sizeof(rc_info_t));
    return ;
  }

//  rc->mouse.x = buff[6] | (buff[7] << 8); // x axis
//  rc->mouse.y = buff[8] | (buff[9] << 8);
//  rc->mouse.z = buff[10] | (buff[11] << 8);

//  rc->mouse.l = buff[12];
//  rc->mouse.r = buff[13];

//  rc->kb.key_code = buff[14] | buff[15] << 8; // key borad code
//  rc->wheel = (buff[16] | buff[17] << 8) - 1024;
}
