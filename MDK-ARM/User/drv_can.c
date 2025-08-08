#include "stm32f4xx.h"                  // Device header
#include "can.h"
#include "drv_can.h"
#include <stdint.h>
#include "calculate.h"

/*can初始化*/
void CAN1_Init()
{
	CAN_FilterTypeDef CAN1_Filter0;
	/*过滤器初始化*/
	CAN1_Filter0.FilterFIFOAssignment=CAN_FILTER_FIFO0;
	CAN1_Filter0.FilterScale=CAN_FILTERSCALE_32BIT;
	CAN1_Filter0.FilterBank=0;
	CAN1_Filter0.FilterMode=CAN_FILTERMODE_IDMASK;
	CAN1_Filter0.FilterActivation=CAN_FILTER_ENABLE;
	CAN1_Filter0.FilterIdHigh=0x0000;
	CAN1_Filter0.FilterIdLow=0x0000;
	CAN1_Filter0.FilterMaskIdHigh=0x0000;
	CAN1_Filter0.FilterMaskIdLow=0x0000;
	HAL_CAN_ConfigFilter(&hcan1, &CAN1_Filter0);
	HAL_CAN_Start(&hcan1);//开启can1
}

void CAN2_Init()
{
	CAN_FilterTypeDef CAN2_Filter14;
	/*过滤器初始化*/
	CAN2_Filter14.FilterFIFOAssignment=CAN_FILTER_FIFO0;
	CAN2_Filter14.FilterScale=CAN_FILTERSCALE_32BIT;
	CAN2_Filter14.FilterBank=14;
	CAN2_Filter14.FilterMode=CAN_FILTERMODE_IDMASK;
	CAN2_Filter14.FilterActivation=CAN_FILTER_ENABLE;
	CAN2_Filter14.FilterIdHigh=0x0000;
	CAN2_Filter14.FilterIdLow=0x0000;
	CAN2_Filter14.FilterMaskIdHigh=0x0000;
	CAN2_Filter14.FilterMaskIdLow=0x0000;
	HAL_CAN_ConfigFilter(&hcan2, &CAN2_Filter14);
	HAL_CAN_Start(&hcan2);//开启can2
}

/*发送函数*/
void CAN1_Send(uint8_t *message)
{
	CAN_TxHeaderTypeDef CAN1_TxHeader0;
	uint32_t TxMailBox0;
	CAN1_TxHeader0.DLC = 8;//数据长度为8字节
	CAN1_TxHeader0.ExtId = 0;//扩展ID为0
	CAN1_TxHeader0.IDE = CAN_ID_STD;//标准ID
	CAN1_TxHeader0.RTR = CAN_RTR_DATA;//发送数据帧
	CAN1_TxHeader0.StdId = M3508_CTRL_ID_BASE;//电机ID 1-4的标识符
	CAN1_TxHeader0.TransmitGlobalTime = DISABLE;//关闭全局时间发送
	TxMailBox0=CAN_TX_MAILBOX0;//指定邮箱
	HAL_CAN_AddTxMessage(&hcan1,&CAN1_TxHeader0,message,&TxMailBox0);//发送信息
}

void CAN2_Send(uint8_t *message)
{
	CAN_TxHeaderTypeDef CAN2_TxHeader0;
	uint32_t TxMailBox1;
	CAN2_TxHeader0.DLC = 8;//数据长度为8字节
	CAN2_TxHeader0.ExtId = 0;//扩展ID为0
	CAN2_TxHeader0.IDE = CAN_ID_STD;//标准ID
	CAN2_TxHeader0.RTR = CAN_RTR_DATA;//发送数据帧
	CAN2_TxHeader0.StdId = 0x1ff;//电机ID 1-4的标识符
	CAN2_TxHeader0.TransmitGlobalTime = DISABLE;//关闭全局时间发送
	TxMailBox1=CAN_TX_MAILBOX1;//指定邮箱
	HAL_CAN_AddTxMessage(&hcan2,&CAN2_TxHeader0,message,&TxMailBox1);//发送信息
}

/*接收函数*/
void CAN1_Receive(uint32_t *ID, uint8_t *Length, uint8_t *Data)//使用数组与指针接收数据
{
  CAN_RxHeaderTypeDef RxMessage;
  uint8_t             Rx_data[8];
  HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxMessage, Rx_data); 
	
  if(RxMessage.IDE == CAN_ID_STD)
		{*ID = RxMessage.StdId;}
  else
		{*ID = RxMessage.ExtId;}
  if (RxMessage.RTR == CAN_RTR_DATA)
  {
     *Length = RxMessage.DLC;
     for (uint8_t i = 0; i < *Length; i ++)
     {
         Data[i] = Rx_data[i];
     }
  }
  else{/*...*/}
}

void CAN2_Receive(uint32_t *ID, uint8_t *Length, uint8_t *Data)
{
	CAN_RxHeaderTypeDef RxMessage;
  uint8_t             Rx_data2[8];
  HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO0, &RxMessage, Rx_data2); 
  
  if(RxMessage.IDE == CAN_ID_STD)
		{*ID = RxMessage.StdId;}
  else
		{*ID = RxMessage.ExtId;}
  if (RxMessage.RTR == CAN_RTR_DATA)
  {
     *Length = RxMessage.DLC;
     for (uint8_t i = 0; i < *Length; i ++)
     {
         Data[i] = Rx_data2[i];
     }
  }
  else{/*...*/}
}

void ExtractMotorData_can1(uint8_t *RxData, MotorData_t *motor_data)
{
    // 提取机械角度
    motor_data->mechanical_angle = HexToDecimal(RxData[0], RxData[1])/8191.0f*360.0f;
    // 提取转速
    motor_data->speed = HexToDecimal(RxData[2], RxData[3]);
    // 提取实际转矩电流
    motor_data->torque_current = HexToDecimal(RxData[4], RxData[5]);
    // 提取电机温度
    motor_data->temperature = RxData[6];
}

void ExtractMotorData_can2(uint8_t *RxData, MotorData_t *motor_data)
{
    // 提取机械角度
    motor_data->mechanical_angle = HexToDecimal(RxData[0], RxData[1])/8191.0f*360.0f;
    // 提取转速
    motor_data->speed = HexToDecimal(RxData[2], RxData[3]);
    // 提取实际转矩电流
    motor_data->torque_current = HexToDecimal(RxData[4], RxData[5]);
    // 提取电机温度
    motor_data->temperature = RxData[6];
}

/*本文件代码优化空间：
 *轮询接收改为中断
 *合并相似函数使用指针传参
 */
