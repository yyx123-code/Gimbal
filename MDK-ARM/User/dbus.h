#ifndef __DBUS_H_
#define __DBUS_H_
/*------------------Include--------------------*/
#include <stdint.h>
/*---------------------PD----------------------*/
/* ----------- RC Channel Definition --------- */
//#define REMOTE_CH_MAX 		((uint16_t)1684)
//#define REMOTE_CH_MIN 		((uint16_t)364)
//#define REMOTE_CH_OFFSET 	((uint16_t)1024)
#define RC_FRAME_LENGTH 	18u
#define DBUS_UART       	huart2
#define DR16_CHANNEL			6
/* ----------- RC Switch Definition ---------- */
//#define RC_SW_UP					((uint16_t)1) 
//#define RC_SW_DOWN 				((uint16_t)2) 
//#define RC_SW_MID 				((uint16_t)3) 
/* ------------ PC Key Definition ------------ */
//#define KEY_PRESSED_OFFSET_W ((uint16_t)0x01<<0) 
//#define KEY_PRESSED_OFFSET_S ((uint16_t)0x01<<1) 
//#define KEY_PRESSED_OFFSET_A ((uint16_t)0x01<<2) 
//#define KEY_PRESSED_OFFSET_D ((uint16_t)0x01<<3) 
//#define KEY_PRESSED_OFFSET_Q ((uint16_t)0x01<<4) 
//#define KEY_PRESSED_OFFSET_E ((uint16_t)0x01<<5) 
//#define KEY_PRESSED_OFFSET_SHIFT 	((uint16_t)0x01<<6) 
//#define KEY_PRESSED_OFFSET_CTRL 	((uint16_t)0x01<<7) 
/* ------------ Data Struct ----------------- */
typedef struct
{
	/* rocker channel information */
  int16_t ch1;
  int16_t ch2;
  int16_t ch3;
  int16_t ch4;
  /* left and right lever information */
  uint8_t sw1;
  uint8_t sw2;
  /* mouse movement and button information */
//  struct
//  {
//    int16_t x;
//    int16_t y;
//    int16_t z;

//    uint8_t l;
//    uint8_t r;
//  } mouse;
//  /* keyboard key information */
//  union {
//    uint16_t key_code;
//    struct
//    {
//      uint16_t W : 1;
//      uint16_t S : 1;
//      uint16_t A : 1;
//      uint16_t D : 1;
//      uint16_t SHIFT : 1;
//      uint16_t CTRL : 1;
//      uint16_t Q : 1;
//      uint16_t E : 1;
//      uint16_t R : 1;
//      uint16_t F : 1;
//      uint16_t G : 1;
//      uint16_t Z : 1;
//      uint16_t X : 1;
//      uint16_t C : 1;
//      uint16_t V : 1;
//      uint16_t B : 1;
//    } bit;
//  } kb;
//  int16_t wheel;
}rc_info_t;

void DBUS_Init(void);
void Get_DR16_Data(uint8_t *Buff);
void DR16_Data(rc_info_t *rc, uint8_t *buff);
void GetValue(void);
uint8_t Remote_Task(void);
uint8_t CheckRemoteSignal(void);

extern volatile uint8_t SbusRxBuffer[RC_FRAME_LENGTH];
extern rc_info_t dr16;
extern int16_t channel[DR16_CHANNEL];

#endif
