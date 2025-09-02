/*------------------Include-------------------*/
#include "dbus.h"
#include "usart.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "queue.h"
#include "main.h"

/* -------------- Internal Data ------------- */ 
volatile uint8_t SbusRxBuffer[2][RC_FRAME_LENGTH];//double sbus双缓冲机制 
static RC_Ctrl_t RC_CtrlData;
volatile uint8_t Active = 0;
volatile uint8_t Ready = 0;

extern osMessageQueueId_t DR16Handle;
extern DMA_HandleTypeDef hdma_usart2_rx;
/* ----------- Function Implements ---------- */ 
static HAL_StatusTypeDef DMA_Rec(UART_HandleTypeDef* huart, uint8_t* pData, uint32_t Size);

/*初始化*/
void DBUS_Init(void)
{
	__HAL_UART_CLEAR_IDLEFLAG(&DBUS_UART);//清除UART空闲标志
	__HAL_UART_ENABLE_IT(&DBUS_UART, UART_IT_IDLE);//使能空闲中断
	HAL_StatusTypeDef status = DMA_Rec(&DBUS_UART, (uint8_t*)SbusRxBuffer[Active], RC_FRAME_LENGTH);
	
	__HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);//禁用半传输中断
	
	if(status != HAL_OK)
		Error_Handler();
}

void Get_DR16_Data(uint8_t *Buff)
{
	RC_CtrlData.remote .ch0 =(Buff[0]|Buff[1]<<8)&0x07FF;
	RC_CtrlData.remote .ch0 -=1024;
  RC_CtrlData.remote .ch1 =(Buff [1]>>3|Buff[2]<<5)&0x07FF;
	RC_CtrlData.remote .ch1 -=1024;
	RC_CtrlData.remote .ch2 =(Buff[2]>>6|Buff[3]<<2|Buff[4]<<10)&0x07FF;
	RC_CtrlData.remote .ch2 -=1024;
	RC_CtrlData.remote .ch3 =(Buff[4]>>1|Buff[5]<<7)&0x07FF;
	RC_CtrlData.remote .ch3 -=1024;
	RC_CtrlData.remote .s1 =(Buff[5]>>4&0x000C)>>2;
	RC_CtrlData.remote .s2 =(Buff[5]>>4&0x003);
	RC_CtrlData.mouse	.x =(Buff[6]|Buff[7]<<8);
	RC_CtrlData.mouse .y =(Buff[8]|Buff[9]<<8);
	RC_CtrlData.mouse .z =(Buff[10]|Buff[11]<<8);
	RC_CtrlData.mouse .press_left =(Buff[12]);
	RC_CtrlData.mouse .press_right =(Buff[13]);
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
			BaseType_t xHigherPriorityTaskWoken = pdFALSE;
			BaseType_t TransRes = xQueueSendFromISR(DR16Handle, (void*)SbusRxBuffer[1 - Active], &xHigherPriorityTaskWoken);
			Active = 1 - Active;//切换到另一个缓冲区

			HAL_UART_Receive_DMA(huart, (uint8_t*)SbusRxBuffer[Active], RC_FRAME_LENGTH);//重新启动DMA
			portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
		}
		else
			HAL_UART_Receive_DMA(huart, (uint8_t*)SbusRxBuffer[Active], RC_FRAME_LENGTH);//重新启动

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
