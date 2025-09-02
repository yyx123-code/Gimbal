#ifndef __DBUS_H_
#define __DBUS_H_
/*------------------Include--------------------*/
#include <stdint.h>
/*---------------------PD----------------------*/
/* ----------- RC Channel Definition --------- */
#define REMOTE_CH_MAX 		((uint16_t)1684)
#define REMOTE_CH_MIN 		((uint16_t)364)
#define REMOTE_CH_OFFSET ((uint16_t)1024)
#define RC_FRAME_LENGTH 	18u
#define DBUS_UART       	huart2
/* ----------- RC Switch Definition ---------- */
#define RC_SW_UP					((uint16_t)1) 
#define RC_SW_DOWN 				((uint16_t)2) 
#define RC_SW_MID 				((uint16_t)3) 
/* ------------ PC Key Definition ------------ */
#define KEY_PRESSED_OFFSET_W ((uint16_t)0x01<<0) 
#define KEY_PRESSED_OFFSET_S ((uint16_t)0x01<<1) 
#define KEY_PRESSED_OFFSET_A ((uint16_t)0x01<<2) 
#define KEY_PRESSED_OFFSET_D ((uint16_t)0x01<<3) 
#define KEY_PRESSED_OFFSET_Q ((uint16_t)0x01<<4) 
#define KEY_PRESSED_OFFSET_E ((uint16_t)0x01<<5) 
#define KEY_PRESSED_OFFSET_SHIFT 	((uint16_t)0x01<<6) 
#define KEY_PRESSED_OFFSET_CTRL 	((uint16_t)0x01<<7) 
/* ------------ Data Struct ----------------- */
typedef struct 
{
	struct 
	{
		int16_t ch0;
		int16_t ch1;
		int16_t ch2;
		int16_t ch3;
		int8_t s1;
		int8_t s2;
	}remote;

	struct 
	{
		int16_t x;
		int16_t y;
		int16_t z;
		int8_t press_left;
		int8_t press_right;		
	}mouse;

	union
	{
		uint16_t key_code;
		struct
		{
			uint16_t W :1;
			uint16_t S :1;
			uint16_t A :1;
			uint16_t D :1;
			uint16_t Q :1;
			uint16_t E :1;
			uint16_t Shift :1;
			uint16_t Ctrl :1;
		}bit;
	}key;
}RC_Ctrl_t;

void DBUS_Init(void);
void Get_DR16_Data(uint8_t *Buff);

extern RC_Ctrl_t RC_CtrlData;

#endif
