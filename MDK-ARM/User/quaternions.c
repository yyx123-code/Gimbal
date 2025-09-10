#include "quaternions.h"
#include "dmpKey.h"
#include "dmpmap.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "MPU6050.h"
#include "MPU6050_reg.h"

/*四元数乘法*/
Quaternion Quaternion_Multiply(Quaternion q1, Quaternion q2)
{
    Quaternion result;
    result.w = q1.w*q2.w - q1.x*q2.x - q1.y*q2.y - q1.z*q2.z;
    result.x = q1.w*q2.x + q1.x*q2.w + q1.y*q2.z - q1.z*q2.y;
    result.y = q1.w*q2.y - q1.x*q2.z + q1.y*q2.w + q1.z*q2.x;
    result.z = q1.w*q2.z + q1.x*q2.y - q1.y*q2.x + q1.z*q2.w;
    return result;
}

/*四元数共轭*/
Quaternion Quaternion_Conjugate(Quaternion q)
{
	Quaternion result;
	result.w = q.w;
	result.x = -q.x;
	result.y = -q.y;
	result.z = -q.z;
	return result;
}

/*从欧拉角构建四元数*/
Quaternion euler_to_quaternion(float pitch, float roll, float yaw)
{
    // 将角度转换为弧度
    float pitch_rad = pitch * 3.14159f / 180.0f * 0.5f;
    float roll_rad = roll * 3.14159f / 180.0f * 0.5f;
    float yaw_rad = yaw * 3.14159f / 180.0f * 0.5f;
    
    float cy = cosf(yaw_rad);
    float sy = sinf(yaw_rad);
    float cp = cosf(pitch_rad);
    float sp = sinf(pitch_rad);
    float cr = cosf(roll_rad);
    float sr = sinf(roll_rad);
    
    Quaternion q;
    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;
    
    return q;
}

BodyRates quaternion_body_rates(int16_t gx, int16_t gy, int16_t gz) {
    BodyRates rates = {0};
    float pitch, roll, yaw;
    
    // 获取当前姿态欧拉角
    MPU_DMP_GetData(&pitch, &roll, &yaw);
    
    // 从欧拉角构建四元数
    Quaternion q = euler_to_quaternion(pitch, roll, yaw);
    
    // 将机体角速度转换为纯四元数
    Quaternion omega_body = {0, (float)gx, (float)gy, (float)gz};
    
    // 计算共轭
    Quaternion q_conj = quaternion_conjugate(q);
    
    // 转换到世界坐标系：ω_world = q * ω_body * q_conj
    Quaternion temp = quaternion_multiply(q, omega_body);
    Quaternion omega_world = quaternion_multiply(temp, q_conj);
    
    // 对于云台控制，我们通常关心的是世界坐标系下的角速度
    // 在世界系中：X→东, Y→北, Z→天
    // 所以：绕Z轴旋转 = Yaw, 绕Y轴旋转 = Pitch, 绕X轴旋转 = Roll
    rates.yaw_rate = omega_world.z;   // 绕世界Z轴
    rates.pitch_rate = omega_world.y; // 绕世界Y轴  
    rates.roll_rate = omega_world.x;  // 绕世界X轴
    
    return rates;
}