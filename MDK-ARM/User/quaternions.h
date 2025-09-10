#ifndef __QUATERNIONS_H_
#define __QUATERNIONS_H_

typedef struct
{
	float w;
	float x;
	float y;
	float z;
}Quaternion;

typedef struct
{
	float yaw_rate;
	float pitch_rate;
	float roll_rate;
}BodyRates;

#endif
