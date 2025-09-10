/***********
*本部分代码为MPU6050相关底层代码实现
*已经移植DMP库，能够完成姿态解算
************/


#include "stm32f4xx.h"                  // Device header
#include "i2c.h"
#include "MPU6050_reg.h"
#include "MPU6050.h"
#include "..\MPU6050\inv_mpu.h"
#include "..\MPU6050\inv_mpu_dmp_motion_driver.h"
#include "math.h"

#define p30 1073741824.0f


/************************************MPU6050基本功能实现**硬件IIC读取数据*****************************************/
/**
  * 函    数：MPU6050写寄存器
  * 参    数：RegAddress 寄存器地址，范围：参考MPU6050手册的寄存器描述
  * 参    数：Data 要写入寄存器的数据，范围：0x00~0xFF
  * 返 回 值：无
  */
void MPU6050_WriteReg(uint8_t RegAddress, uint8_t Data)
{
	
	HAL_I2C_Mem_Write(&hi2c1 ,MPU6050_ADDRESS,RegAddress,I2C_MEMADD_SIZE_8BIT ,&Data,1,1000);							
}

/**
  * 函    数：MPU6050读寄存器
  * 参    数：RegAddress 寄存器地址，范围：参考MPU6050手册的寄存器描述
  * 返 回 值：读取寄存器的数据，范围：0x00~0xFF
  */
uint8_t MPU6050_ReadReg(uint8_t RegAddress)
{
	uint8_t Data;
	
	HAL_I2C_Mem_Read(&hi2c1 ,MPU6050_ADDRESS,RegAddress,I2C_MEMADD_SIZE_8BIT ,&Data,1,1000);			
	
	return Data;
}

void MPU6050_Init(void)
{
	MPU6050_WriteReg(MPU6050_PWR_MGMT_1, 0x01);				//电源管理寄存器1，取消睡眠模式，开启温度传感器，选择时钟源为X轴陀螺仪
	MPU6050_WriteReg(MPU6050_PWR_MGMT_2, 0x00);				//电源管理寄存器2，保持默认值0，所有轴均不待机
	MPU6050_WriteReg(MPU6050_SMPLRT_DIV, 0x09);				//采样率分频寄存器，配置采样率
	MPU6050_WriteReg(MPU6050_CONFIG, 0x06);					//配置寄存器，配置DLPF
	MPU6050_WriteReg(MPU6050_GYRO_CONFIG, 0x18);			//陀螺仪配置寄存器，选择满量程为±2000°/s
	MPU6050_WriteReg(MPU6050_ACCEL_CONFIG, 0x18);			//加速度计配置寄存器，选择满量程为±16g
}

/**
  * 函    数：MPU6050获取ID号
  * 参    数：无
  * 返 回 值：MPU6050的ID号
  */
uint8_t MPU6050_GetID(void)
{
	return MPU6050_ReadReg(MPU6050_WHO_AM_I);		//返回WHO_AM_I寄存器的值
}

/**
  * 函    数：MPU6050获取数据
  * 参    数：AccX AccY AccZ 加速度计X、Y、Z轴的数据，使用输出参数的形式返回，范围：-32768~32767
  * 参    数：GyroX GyroY GyroZ 陀螺仪X、Y、Z轴的数据，使用输出参数的形式返回，范围：-32768~32767
  * 参    数：Temp 温度传感器的数据，使用输出参数的形式返回，范围：-32768~32767
  * 返 回 值：无
  */
void MPU6050_GetData(int16_t *AccX, int16_t *AccY, int16_t *AccZ, 
						int16_t *GyroX, int16_t *GyroY, int16_t *GyroZ, int16_t *Temp)
{
	uint8_t DataH, DataL;								//定义数据高8位和低8位的变量
	
	DataH = MPU6050_ReadReg(MPU6050_ACCEL_XOUT_H);		//读取加速度计X轴的高8位数据
	DataL = MPU6050_ReadReg(MPU6050_ACCEL_XOUT_L);		//读取加速度计X轴的低8位数据
	*AccX = (DataH << 8) | DataL;						//数据拼接，通过输出参数返回
	
	DataH = MPU6050_ReadReg(MPU6050_ACCEL_YOUT_H);		//读取加速度计Y轴的高8位数据
	DataL = MPU6050_ReadReg(MPU6050_ACCEL_YOUT_L);		//读取加速度计Y轴的低8位数据
	*AccY = (DataH << 8) | DataL;						//数据拼接，通过输出参数返回
	
	DataH = MPU6050_ReadReg(MPU6050_ACCEL_ZOUT_H);		//读取加速度计Z轴的高8位数据
	DataL = MPU6050_ReadReg(MPU6050_ACCEL_ZOUT_L);		//读取加速度计Z轴的低8位数据
	*AccZ = (DataH << 8) | DataL;						//数据拼接，通过输出参数返回
	
	DataH = MPU6050_ReadReg(MPU6050_GYRO_XOUT_H);		//读取陀螺仪X轴的高8位数据
	DataL = MPU6050_ReadReg(MPU6050_GYRO_XOUT_L);		//读取陀螺仪X轴的低8位数据
	*GyroX = (DataH << 8) | DataL;						//数据拼接，通过输出参数返回
	
	DataH = MPU6050_ReadReg(MPU6050_GYRO_YOUT_H);		//读取陀螺仪Y轴的高8位数据
	DataL = MPU6050_ReadReg(MPU6050_GYRO_YOUT_L);		//读取陀螺仪Y轴的低8位数据
	*GyroY = (DataH << 8) | DataL;						//数据拼接，通过输出参数返回
	
	DataH = MPU6050_ReadReg(MPU6050_GYRO_ZOUT_H);		//读取陀螺仪Z轴的高8位数据
	DataL = MPU6050_ReadReg(MPU6050_GYRO_ZOUT_L);		//读取陀螺仪Z轴的低8位数据
	*GyroZ = (DataH << 8) | DataL;						//数据拼接，通过输出参数返回
	
	DataH = MPU6050_ReadReg(MPU6050_TEMP_OUT_H);		//读取温度传感器的高8位数据
	DataL = MPU6050_ReadReg(MPU6050_TEMP_OUT_L);		//读取温度传感器的低8位数据
	*Temp = (DataH << 8) | DataL;						//数据拼接，通过输出参数返回
	
	
}
/************************************MPU6050基本功能实现**硬件IIC读取数据*****************************************/


/**********************************DMP库移植与初始化************************************************************************/
//MPU6050自测试
//返回值:0,正常
//    其他,失败
uint8_t run_self_test(void)
{
	int result;
	//char test_packet[4] = {0};
	long gyro[3], accel[3]; 
	result = mpu_run_self_test(gyro, accel);
	if (result == 0x3) 
	{
		/* Test passed. We can trust the gyro data here, so let's push it down
		* to the DMP.
		*/
		float sens;
		unsigned short accel_sens;
		mpu_get_gyro_sens(&sens);
		gyro[0] = (long)(gyro[0] * sens);
		gyro[1] = (long)(gyro[1] * sens);
		gyro[2] = (long)(gyro[2] * sens);
		dmp_set_gyro_bias(gyro);
		mpu_get_accel_sens(&accel_sens);
		accel[0] *= accel_sens;
		accel[1] *= accel_sens;
		accel[2] *= accel_sens;
		dmp_set_accel_bias(accel);
		return 0;
	}else return result;
}

unsigned short inv_row_2_scale(const signed char *row)
{
	unsigned short b;
	if(row[0]>0) b=0;
	else if(row[0]<0) b=4;
	else if(row[1]>0) b=1;
	else if(row[1]<0) b=5;
	else if(row[2]>0) b=2;
	else if(row[2]<0) b=6;
	else b=7;
	
	return b;
}


unsigned short inv_orientation_matrix_to_scalar(const signed char *mtx)
{
	unsigned short scalar;
	scalar = inv_row_2_scale(mtx);
	scalar |= inv_row_2_scale(mtx+3)<<3;
	scalar |= inv_row_2_scale(mtx+6)<<6;
	
	return scalar ;
}


//mpu6050,dmp初始化
//返回值:0,正常
//    其他,失败
uint8_t  MPU_DMP_Init(void)
{
	uint8_t  res=0;
	struct int_param_s int_param;//这个没什么用，就是为了能给他实参调用起来
	
	static signed char gyro_orientation[9] = { 1, 0, 0,
                                           0, 1, 0,
                                           0, 0, 1};

	
	if(mpu_init(&int_param)==0)	//初始化MPU6050
	{	 
		res=mpu_set_sensors(INV_XYZ_GYRO|INV_XYZ_ACCEL);//设置所需要的传感器
		if(res)return 1; 
		res=mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);//设置FIFO
		if(res)return 2; 
		res=mpu_set_sample_rate(200);	//设置采样率
		if(res)return 3; 
		res=dmp_load_motion_driver_firmware();		//加载dmp固件
		if(res)return 4; 
		res=dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation));//设置陀螺仪方向
		if(res)return 5; 
		res=dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT|DMP_FEATURE_TAP|	//设置dmp功能
		    DMP_FEATURE_ANDROID_ORIENT|DMP_FEATURE_SEND_RAW_ACCEL|DMP_FEATURE_SEND_CAL_GYRO|
		    DMP_FEATURE_GYRO_CAL);
		if(res)return 6; 
		res=dmp_set_fifo_rate(100);	//设置DMP输出速率(最大不超过200Hz)
		if(res)return 7;   
		res=run_self_test();		//自检
		if(res)return 8;    
		res=mpu_set_dmp_state(1);	//使能DMP
		if(res)return 9;     
	}
	return 0;
}
//得到dmp处理后的数据(注意,本函数需要比较多堆栈,局部变量有点多)
//pitch:俯仰角 精度:0.1°   范围:-90.0° <---> +90.0°
//roll:横滚角  精度:0.1°   范围:-180.0°<---> +180.0°
//yaw:航向角   精度:0.1°   范围:-180.0°<---> +180.0°
uint8_t  MPU_DMP_GetData(float *pitch,float *roll,float *yaw)
{
	float fquat[4]={0.0f,0.0f,0.0f,0.0f};
	short gyro[3];
	short accel[3]; 
	long quat[4];
    unsigned long timestamp; 
	short sensors; 
	unsigned char more;
	if(dmp_read_fifo (gyro,accel,quat,&timestamp,&sensors ,&more))//错误
	{
		return 1;
	}
	if(sensors &INV_WXYZ_QUAT ) //有返回值
	{
		for(int i=0;i<4; i++)
		{
			fquat [i]= quat [i]/p30;
		}
		
		*pitch = asin (-2*fquat [1]*fquat[3]+2*fquat[0]*fquat[2])*57.3;
		*roll = atan2 (2*fquat[2]*fquat[3]+2*fquat[0]*fquat[1],-2*fquat[1]*fquat[1]-2*fquat[2]*fquat[2]+1)*57.3;
		*yaw = atan2(2*(fquat[1]*fquat[2]+fquat[0]*fquat[3]),fquat[0]*fquat[0]+fquat[1]*fquat[1]-fquat[2]*fquat[2]-fquat[3]*fquat[3])*57.3;
	}
	return 0;
}

/**********************************DMP库移植与初始化************************************************************************/

