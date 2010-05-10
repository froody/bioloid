#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>

#include "bioloid.h"
#include "robot.h"
#include "motion.h"
#include "comms.h"
#include "BLC-Protocol.h"
#include "led.h"
#include "gyro.h"

// ----------- extern functions from led.c ---------------
extern void LED_Init(void);
extern void Set_LED(unsigned char LEDno, unsigned char state);

// ----------- extern functions from comms.c ---------------
extern void TxDString(byte *bData);
extern byte RxD81(void);

extern byte gbpParameter[128];
extern byte gbpRxBuffer[128]; 
extern byte gbpTxBuffer[128]; 
extern byte TxPacket(byte bID, byte bInstruction, byte bParameterLength);
extern byte RxPacket(byte bRxLength);
extern void PrintBuffer(byte *bpPrintBuffer, byte bLength);
extern volatile unsigned int ROBOTSTATUS;

int A2D(unsigned char channel);

int A2DData[HIS_MAX][2];
double A2D_Neutral[2];	// Neutral

volatile int AngleGyro = 0;


int A2D(unsigned char channel){
	 // Read AD Converter AD0
	  //ADMUX= (1 << REFS0)|(1 << ADLAR)|channel;
	  ADMUX= (1 << REFS0)|channel;
	  ADCSRA=(1 << ADEN)|(1 << ADSC)|(1 << ADPS1)|(1 << ADPS2);
	  do{  }while(ADIF==0);
	  int r = ADCH;
	  r = r << 8;
	  r = r + ADCL;
	  return r;
}


void InitFilter(int Initial_A2DFront_Reading,int Initial_A2DLeft_Reading)
{
	int i;

	A2D_Neutral[A2D_FRONT] = Initial_A2DFront_Reading;
	A2D_Neutral[A2D_LEFT] = Initial_A2DLeft_Reading;

	//Set all History to 0
	for(i = NOW; i >= 0; i--)
	{
		A2DData[i][A2D_FRONT] = Initial_A2DFront_Reading;
		A2DData[i][A2D_LEFT] = Initial_A2DLeft_Reading;
	}
}

void AddFilterData(int A2DValFront , int A2DValLeft){
	int i;
	// Shift the history (newest entry (t-now) is at the end of the buffer
	for(i = 0; i < NOW; i++)
	{
		A2DData[i][A2D_FRONT] = A2DData[i+1][A2D_FRONT];
		A2DData[i][A2D_LEFT] = A2DData[i+1][A2D_LEFT];
	}
	
	A2DData[NOW][A2D_FRONT] = A2DValFront;
	A2DData[NOW][A2D_LEFT]  = A2DValLeft;

	// the average consists of 98.8% of the las and 1.2 percent of the current A2D value
	A2D_Neutral[A2D_FRONT] = (A2D_Neutral[A2D_FRONT]*(1.0 - MOVING_NEUTRAL_POSITON_COMPONENT) + A2DData[NOW][A2D_FRONT]*MOVING_NEUTRAL_POSITON_COMPONENT);
	A2D_Neutral[A2D_LEFT] = (A2D_Neutral[A2D_LEFT]*(1.0 - MOVING_NEUTRAL_POSITON_COMPONENT) + A2DData[NOW][A2D_LEFT]*MOVING_NEUTRAL_POSITON_COMPONENT);
}

void ResetGyro(void)
{
	AngleGyro = 0;
}

void InitGyro(void)
{
	// initialise A2D, needs a dry run to enable
	A2D(1);
	A2D(2);
	InitFilter(A2D(A2D_CHANNEL_FRONT),A2D(A2D_CHANNEL_SIDE)); // front and left
	AngleGyro = 0;
}



void MeasureGyro(void)
{
	static int pt=0;	
	// is there a channel 0 ? this could be pin 62
	AddFilterData(A2D(A2D_CHANNEL_FRONT) ,A2D(A2D_CHANNEL_SIDE));
	
	// Acc should now contain acceleratio between -512 to +512 for 0 to 5V where 2.55 V is 0
	// instead of taking A2DData directly, a lowpass filter should be applied to A2DData
	// A2DAccFiltered = Filter();   Acc = (A2DAccFiltered - A2D_Neutral...
	int OmegaGyro = (A2DData[NOW][A2D_FRONT] - A2D_Neutral[A2D_FRONT]);
	// amplify noise , oh sorry i meant integrate 2 times to get position
	AngleGyro = AngleGyro + OmegaGyro;
	// Maybe there should be a moving average on the Angle
	
	
	pt++;
	if(pt==10){
		pt=0;
		printf("gr=%i\n",AngleGyro);
	}
}

