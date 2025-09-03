#include "stm32f4xx.h"                  // Device header
#include "can.h"
#include "drv_can.h"
#include <stdint.h>

/*CAN缓存*/
CAN_RxBuffer_t can1_rx_buffer;
CAN_RxBuffer_t can2_rx_buffer;
/*电机数据*/
MotorData_t Yaw_Data;
MotorData_t Pitch_Data;
MotorData_t Friction_Left_Data;
MotorData_t Friction_Right_Data;
MotorData_t Driver_Data;
/*拨盘电机*/
Driver_Mode_t Driver_Mode;
/*云台电机*/
Motor_Mode_t Yaw_Mode;
Motor_Mode_t Pitch_Mode;

/*can初始化*/
void CAN_Init(void)
{
	/*CAN1*/
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
	CAN1_Filter0.SlaveStartFilterBank = 14;//从过滤器组(仅can1)
	HAL_CAN_ConfigFilter(&hcan1, &CAN1_Filter0);
	/*错误处理*/
	if((HAL_CAN_ConfigFilter(&hcan1, &CAN1_Filter0)) != HAL_OK)
	{Error_Handler();}
	/*开启can1以及中断*/
	HAL_CAN_Start(&hcan1);
	HAL_CAN_ActivateNotification(&hcan1,
			CAN_IT_RX_FIFO0_MSG_PENDING |	//FIFO0消息中断
			CAN_IT_ERROR |								//错误中断
			CAN_IT_BUSOFF |								//总线关闭中断
			CAN_IT_LAST_ERROR_CODE);			//最后错误代码中断
	/*CAN2*/
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
	/*错误处理*/
	if((HAL_CAN_ConfigFilter(&hcan2, &CAN2_Filter14)) != HAL_OK)
	{Error_Handler();}	
	/*开启can2以及中断*/
	HAL_CAN_Start(&hcan2);//开启can2
	HAL_CAN_ActivateNotification(&hcan2,
			CAN_IT_RX_FIFO0_MSG_PENDING |	//FIFO0消息中断
			CAN_IT_ERROR |								//错误中断
			CAN_IT_BUSOFF |								//总线关闭中断
			CAN_IT_LAST_ERROR_CODE);			//最后错误代码中断
}

/**
  * @brief  CAN发送函数
  * @param  motor_id: 电机ID
  * @param  message: 待发送的8字节数据
  */
void CAN_Send(uint32_t motor_id, uint8_t *message)
{
	CAN_HandleTypeDef *hcan;
	uint32_t std_id;
	uint32_t mailbox;
	/*CAN总线和发送ID*/
	if(motor_id == LEFT_FRICTION_ID || motor_id == RIGHT_FRICTION_ID || motor_id == DRIVER_ID)
	{
		hcan = &hcan1;
		std_id = M3508_CTRL_ID_BASE;
		mailbox = CAN_TX_MAILBOX0;	//指定邮箱
  }
	else if(motor_id == PITCH_ID)
	{
		hcan = &hcan1;
		std_id = GM6020_CTRL_ID_BASE;
		mailbox = CAN_TX_MAILBOX0;	//指定邮箱
	}
	else if(motor_id == YAW_ID)
	{
		hcan = &hcan2;
		std_id = GM6020_CTRL_ID_BASE;
		mailbox = CAN_TX_MAILBOX1;
	}
	/* 统一报文头配置 */
	CAN_TxHeaderTypeDef tx_header =
	{
		.StdId = std_id,							//电机ID1-4的标识符
		.ExtId = 0,										//扩展ID为0
		.DLC = 8,											//数据长度为8字节
		.IDE = CAN_ID_STD,						//标准ID
		.RTR = CAN_RTR_DATA,					//发送数据帧
		.TransmitGlobalTime = DISABLE	//关闭全局时间发送
	};
	HAL_CAN_AddTxMessage(hcan, &tx_header, message, &mailbox);
}

/*接收函数*/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  CAN_RxHeaderTypeDef rx_header;
  uint8_t             rx_data[8];
	if(hcan == &hcan1)
	{
		/*FIFO0读取信息*/
		HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data); 
		can1_rx_buffer.length = rx_header.DLC;
		can1_rx_buffer.id = (rx_header.IDE == CAN_ID_STD) ? rx_header.StdId : rx_header.ExtId;
		/*储存数据(使用安全拷贝)*/
		ARM_MPU_OrderedMemcpy((volatile uint32_t*)can1_rx_buffer.data,
                     (const uint32_t*)rx_data,
                     (rx_header.DLC + 3) / 4);
		/*提取数据*/
		ProcessMotorData(&can1_rx_buffer);
	}
	/*CAN2*/
	else if(hcan == &hcan2)
	{
		/*FIFO0读取信息*/
		HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data); 
		can2_rx_buffer.length = rx_header.DLC;
		can2_rx_buffer.id = (rx_header.IDE == CAN_ID_STD) ? rx_header.StdId : rx_header.ExtId;
		/*储存数据(使用安全拷贝)*/
		ARM_MPU_OrderedMemcpy((volatile uint32_t*)can2_rx_buffer.data,
                     (const uint32_t*)rx_data,
                     (rx_header.DLC + 3) / 4);
		/*提取数据*/
		ProcessMotorData(&can2_rx_buffer);
	}
}

/* 统一电机数据处理函数 */
void ProcessMotorData(CAN_RxBuffer_t* rx_buf)
{
    MotorData_t* target_data = NULL;
    /* 根据ID选择目标数据结构 */
    switch(rx_buf->id)
    {
        case YAW_ID:
            target_data = &Yaw_Data;
            break;
        case PITCH_ID:
            target_data = &Pitch_Data;
            break;
        case LEFT_FRICTION_ID:
            target_data = &Friction_Left_Data;
            break;
        case RIGHT_FRICTION_ID:
            target_data = &Friction_Right_Data;
            break;
        case DRIVER_ID:
            target_data = &Driver_Data;
            break;
        default:
            Error_Handler(); // 未知ID处理
            return;
    }
    /* 统一数据提取 */
    target_data->mechanical_angle = HexToDecimal(rx_buf->data[0], rx_buf->data[1]) / MOTOR_PPR * 360.0f;
    target_data->speed = HexToDecimal(rx_buf->data[2], rx_buf->data[3]);
    target_data->torque_current = HexToDecimal(rx_buf->data[4], rx_buf->data[5]);
    target_data->temperature = rx_buf->data[6];
}
