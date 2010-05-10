#include <inttypes.h>
#include "bioloid.h"
#include "robot.h"
#include "motion.h"
#include "comms.h"
#include "BLC-Protocol.h"
#include "led.h"
#include "buggy.h"
#include "dynamixel.h"

#include <stdio.h>

// ----------- extern functions from led.c ---------------
extern void LED_Init(void);
extern void Set_LED(unsigned char LEDno, unsigned char state);

// ----------- extern functions from comms.c ---------------
extern byte RxD81(void);

extern byte gbpParameter[128];
extern byte gbpRxBuffer[128]; 
extern byte gbpTxBuffer[128]; 
extern byte TxPacket(byte bID, byte bInstruction, byte bParameterLength);
extern byte RxPacket(byte bRxLength);
extern void PrintBuffer(byte *bpPrintBuffer, byte bLength);
extern volatile unsigned int ROBOTSTATUS;

volatile uint8_t BUGGY_MODE='S';

void move_servo(int ServoNo, uint16_t OmegaServo)
{
	printf("%s: %d -> %d\n", __func__, ServoNo, OmegaServo);
	dxl_write_word(ServoNo, P_GOAL_SPEED_L, OmegaServo);
}




void buggy_move(unsigned char dir)
{
	BUGGY_MODE=dir;
	int ServoNo = 1;
	uint16_t S[5];

	uint16_t V = 512;
	uint16_t T = 300;
	
	if(dir=='F'){
		S[1]=NEG+V;	S[2]=V;S[3]=NEG+V;	S[4]=V;	// forward	
	}
	if(dir=='B'){
		S[1]=V;S[2]=NEG+V; S[3]=V;     S[4]=NEG+V; // backward
	}
	if(dir=='A'){
		//S[1]=0;S[2]=NEG+T;S[3]=0;S[4]=NEG+T; // Anti Clockwise
		S[1]=NEG+T;S[2]=NEG+T;S[3]=NEG+T;S[4]=NEG+T;
	}
	if(dir=='C'){
		//S[1]=NEG+T;S[2]=0;S[3]=NEG+T;S[4]=0;	//clockwise
		S[1]=T;S[2]=T;S[3]=T;S[4]=T;
	}
	
	
	if(dir=='S'){
		S[1]=0;S[2]=0;S[3]=0;S[4]=0;
		 printf("[pg%02X]\n",1);
	}
	
	for(ServoNo=1;ServoNo<=4;ServoNo++){
		move_servo(ServoNo,S[ServoNo]);
	}
}


void buggy_timer(void){
	static int time=0;
	if((BUGGY_MODE=='A')||(BUGGY_MODE=='C')){
		time++;
		if(time>93){
			time=0;
			 buggy_move('S');
		}
	}else{
		time=0;
	}
}

