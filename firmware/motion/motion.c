/* 
 * University of Plymouth Bioloid Robot Controller
 * for Atmel Mega 128
 * FIRA England Robot Football Team
 * - Based on Dynamixel Evaluation Example (Original Author : Byoung Soo Kim)
 * - Initial support for executing Motion Editor Pages
 * Author : Joerg Wolf, joerg.wolf -xaxtx- plymouth.ac.uk
 * 
 * Version 0.11
 */
#define ENABLE_BIT_DEFINITIONS 
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <inttypes.h>
#include <string.h>
#include <stdio.h>
#include "motion.h"
#include "robot-dash.h"

#include "dynamixel.h"
#include <stdlib.h>
#include <math.h>

#define LO_BYTE(x) ((x) & 0xff)
#define HI_BYTE(x) (((x) >> 8) & 0xff)

#if 0
struct RECORD {
  uint16_t posData[31];
  byte delay;
  byte speed;
  } _record;
  
struct PHEADER {
  char name[14];
  byte res1[6];
  byte pageStep;
  byte playCode;
  byte pageSpeed;
  byte dxlSetup;
  byte accelTime;
  byte nextPage;
  byte exitPage;
  byte linkedPage1;
  byte linkedPage1PlayCode;
  byte linkedPage2;
  byte linkedPage2PlayCode;
  byte checkSum;
  byte res2[32];
  } _PHEADER;
  
struct PAGE {
  struct PHEADER header;
  struct RECORD rec[7];
  } _PAGE;
#endif

volatile MOTION_HEARTBEAT_DATA MHD;
volatile int ServoPos[NUM_OF_SERVOS_ATTACHED];
volatile int ServoTrim[NUM_OF_SERVOS_ATTACHED];

volatile uint8_t Balance;

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



uint16_t doPose(int page,char pose);
uint16_t doPage(int page);
void ContinueMotion_HeartBeat(void);
void MotionInitialise(void);


int getCurrentServoPos(uint8_t ServoNo)
{
	return dxl_read_word(ServoNo, P_PRESENT_POSITION_L);
}


void MotionInitialise(void){
	memset((void *)&MHD, 0, sizeof(MOTION_HEARTBEAT_DATA));
	MHD.CurrentPage = -1;
	uint8_t ServoNo;
	
	for( ServoNo=1;ServoNo < NUM_OF_SERVOS_ATTACHED; ServoNo++){
			ServoPos[ServoNo] = 0;
			ServoTrim[ServoNo] = 0;
			ServoPos[ServoNo] =  getCurrentServoPos(ServoNo);
			if(ServoPos[ServoNo] == SERVO_NOT_CONNECTED){
				printf("E:Servo %i not connected\n",ServoNo);
			}
			/*
			// Get Lower Byte of Current Position
			uint16_t POS_LB=0,POS_HB=0;
			gbpParameter[0] = P_PRESENT_POSITION_L;
			gbpParameter[1] = 1; //Read Length
			TxPacket(ServoNo,INST_READ,2);
			uint8_t bRxPacketLength = RxPacket(DEFAULT_RETURN_PACKET_SIZE+gbpParameter[1]);
			if(bRxPacketLength == DEFAULT_RETURN_PACKET_SIZE+gbpParameter[1]){
				if(gbpRxBuffer[4]==0){
					POS_LB=gbpRxBuffer[5];
				}else{
					// error
					ServoPos[ServoNo] = SERVO_NOT_CONNECTED;
				}
			}else{
				//error
				ServoPos[ServoNo] = SERVO_NOT_CONNECTED;
			}
			
			// Get Higher Byte of Current Position
			gbpParameter[0] = P_PRESENT_POSITION_H;
			gbpParameter[1] = 1; //Read Length
			TxPacket(ServoNo,INST_READ,2);
			bRxPacketLength = RxPacket(DEFAULT_RETURN_PACKET_SIZE+gbpParameter[1]);
			if(bRxPacketLength == DEFAULT_RETURN_PACKET_SIZE+gbpParameter[1]){
				if(gbpRxBuffer[4]==0){
					POS_HB=gbpRxBuffer[5];
				}else{
					// error
					ServoPos[ServoNo] = SERVO_NOT_CONNECTED;
				}
			}else{
				//error
				ServoPos[ServoNo] = SERVO_NOT_CONNECTED;
			}
			
			// CODE 9999 means the servo is not attached
			if(ServoPos[ServoNo] != SERVO_NOT_CONNECTED){
				ServoPos[ServoNo] = (POS_HB<<8) + POS_LB;
				//char temp[50]; sprintf(temp," %i",ServoPos[ServoNo]); TxDString(temp);
			}else{
				char temp[50]; sprintf(temp,"E:Servo %i not connected\n\r",ServoNo); TxDString(temp);
			}
			*/
	}
}

void ContinueMotion_HeartBeat(void)
{
	uint8_t MOVING = FALSE;

	if(MHD.CurrentPage != -1){

		// Check if Servos are still in Motion
		/*
		   gbpParameter[0] = P_MOVING;
		   gbpParameter[1] = 1; //Read Length
		   for( ServoNo=1;ServoNo < NUM_OF_SERVOS_ATTACHED; ServoNo++){	
		   TxPacket(ServoNo,INST_READ,2);
		   uint8_t bRxPacketLength = RxPacket(DEFAULT_RETURN_PACKET_SIZE+gbpParameter[1]);
		   if(bRxPacketLength == DEFAULT_RETURN_PACKET_SIZE+gbpParameter[1]){
		   if(gbpRxBuffer[4]==0){
		//TxDString("M:");TxD8Hex(gbpRxBuffer[5]);
		if(gbpRxBuffer[5]!=0){
		// Servo is still moving
		MOVING = TRUE;
		}
		}else{
		TxDString("Error:");//gbpRxBuffer[4]
		}
		}
		}
		*/


		if(MHD.MoveFinishTime>0){MHD.MoveFinishTime--;}

		if(MHD.MoveFinishTime==0){
			MOVING = FALSE;
			//TxDString("M");
		}else{
			MOVING = TRUE;
		}

		if(MOVING == FALSE){ 
			if(MHD.PosePause>0){MHD.PosePause--;}

			if(MHD.PosePause==0){
				//TxDString("+\n\r");
				if(ROBOTSTATUS!=STOP_POSE){
					if(MHD.CurrentPose+1<MHD.NoOfPoses){
						// next pose
						MHD.CurrentPose++;
						//TxDString("---Next Pose---\n\r");
						MHD.PosePause = doPose(MHD.CurrentPage,MHD.CurrentPose);
					}else{
						// finished
						//TxDString("Done Page\n\r");
						//printf("[pg%02X]\n",MHD.CurrentPage);
#ifdef ROBOT_DASH_MODE
						if(MHD.CurrentPage==FORWARD_STEP_COMPLETED_PAGE_NO){
							Forward_Dash_StepNo++;
						}
						if(MHD.CurrentPage==BACKWARD_STEP_COMPLETED_PAGE_NO){
							Backward_Dash_StepNo++;
						}
#endif
						cli();
						uint8_t NextPage= pgm_read_byte_far  ((uint32_t)POSE_BASE_ADR + (uint32_t)POSE_PAGE_SIZE*(uint32_t)MHD.CurrentPage + (uint32_t)POSE_PAGE_NEXT_PAGE);
						sei();
						if(ROBOTSTATUS!=STOP_PAGE){
							//TxDString("NEXT PAGE\n\r");//NextPage;
							if(NextPage != 0){
								doPage(NextPage);
							}else{
								MHD.CurrentPage=-1;
							}
						}else{
							uint8_t ExitPage= pgm_read_byte_far  ((uint32_t)POSE_BASE_ADR + (uint32_t)POSE_PAGE_SIZE*(uint32_t)MHD.CurrentPage + (uint32_t)POSE_PAGE_EXIT_PAGE);
							if(ExitPage != 0){
								doPage(ExitPage);
							}else{
								MHD.CurrentPage=-1;
							}
						}
					}
				}
			}
		}
	}
}

void delay(unsigned long ms);

void beep(void);

int
motorIsMoving(void)
{
	int i;
	for(i=1;i<NUM_OF_SERVOS_ATTACHED;i++)
	{
		if(dxl_read_byte(i, P_MOVING))
			return 1;
	}

	return 0;
}

uint16_t doPage(int page)
{
	//char temp[50];
	//TxDString("\n\rRequest for doPage\n\r");
	if(page>127)page=127;//(MHD.CurrentPage != page)
	if(page!=-1){
		// New request
		MHD.CurrentPage = page;
		cli();
		MHD.NoOfPoses= pgm_read_byte_far ((uint32_t)POSE_BASE_ADR +(uint32_t)POSE_PAGE_SIZE*(uint32_t)MHD.CurrentPage + (uint32_t)POSE_PAGE_NUM_OF_MOTIONS);
		sei();
		MHD.TotalTime=0;
		MHD.CurrentPose = 0;

		beep();
		printf("new page %d: poses=%d\n", MHD.CurrentPage, MHD.NoOfPoses);

		int i;
		for(i=0;i<MHD.NoOfPoses;i++) {
			MHD.PosePause = doPose(MHD.CurrentPage,MHD.CurrentPose);
			//printf("last PosePause=%i\n",MHD.PosePause);
			while(motorIsMoving())
				;
		
			delay(MHD.PosePause);
		}

		return MHD.TotalTime;
	}
	return 0;
}



void SendServoTargetPos(uint8_t ServoNo, int16_t targetPos,uint16_t targetSpeed)
{
	//printf("setting servo %d pos to %d @ %d\n", ServoNo, targetPos, targetSpeed);
	dxl_set_txpacket_id(ServoNo);
	dxl_set_txpacket_instruction(INST_WRITE);
	dxl_set_txpacket_parameter(0, P_GOAL_POSITION_L);
	dxl_set_txpacket_parameter(1, LO_BYTE(targetPos));
	dxl_set_txpacket_parameter(2, HI_BYTE(targetPos));
	dxl_set_txpacket_parameter(3, LO_BYTE(targetSpeed));
	dxl_set_txpacket_parameter(4, HI_BYTE(targetSpeed));
	dxl_set_txpacket_length(7);

	dxl_txrx_packet();
}

#define UNITS_PER_MS 0.6721


uint16_t doPose(int page,char pose)
{
	int i;

	uint32_t pageAddr = POSE_BASE_ADR + page*POSE_PAGE_SIZE;
	uint32_t poseAddr = pageAddr + POSE_0_OFFSET + pose*POSE_SIZE;

	uint16_t PageSpeed = pgm_read_byte_far(pageAddr + POSE_PAGE_MOTION_SPEED);
	uint16_t PageFast  = pgm_read_byte_far(pageAddr + POSE_PAGE_FAST_FLAG);
	uint16_t PosePause = pgm_read_byte_far(poseAddr + POSE_PAUSE_ADR);
	uint8_t PoseSpeed = pgm_read_byte_far(poseAddr + POSE_SPEED_ADR);

	struct {
		uint16_t pos;
		uint16_t dist;
	} servoControl[NUM_OF_SERVOS_ATTACHED];


	//printf("PageSpeed: %d, PoseSpeed: %d\n", PageSpeed, PoseSpeed);
	
	if(MHD.CurrentPage!=-1){
		cli();
		uint8_t NoOfPosesinPage = pgm_read_byte_far (pageAddr + POSE_PAGE_NUM_OF_MOTIONS);
		sei();
		if( pose+1 > NoOfPosesinPage){
			//printf("E:Pose not on this page\n\r");
			return 0;
		}
		cli();

		
		#ifdef ROBOT_DASH_MODE
			if((PageFast==10)&&(PageSpeed>=10)){PageSpeed=280;}
		#else
			if((PageFast==10)&&(PageSpeed>=10)){PageSpeed=160;}
		#endif
		sei();

		//uint8_t AccelTime = pgm_read_byte_far  ((uint32_t)POSE_BASE_ADR + (uint32_t)POSE_PAGE_SIZE*(uint32_t)page + (uint32_t)POSE_PAGE_ACCEL_TIME);
		//uint16_t Ta_T0 = (float)AccelTime*k;
		//sprintf(temp,"\r\nPageNo=%i,PageSpeed=%i,PoseSpeed=%i,PosePause=%i\r\n",MHD.CurrentPage,PageSpeed,PoseSpeed,PosePause);TxDString(temp);
		//sprintf(temp,"#%i,%i Sp%i,%i,Pa%i\n\r",page,pose,PageSpeed,PoseSpeed,PosePause);TxDString(temp);
		/**
			Find the Servo that has to move the longest path (ThetaMax)
			This servo will be the only one that moves at the specified speed
			The other servos will move slower so that all servos finish the move at the same time
		*/
		uint16_t longestDistance = 0;
		for(i=1;i < NUM_OF_SERVOS_ATTACHED; i++){
			if(ServoPos[i] != SERVO_NOT_CONNECTED){
				cli();
				int16_t targetPos = pgm_read_word_far(poseAddr + (uint32_t)i*2 );
				sei();

				//ThetaTarget += ServoTrim[i];
				
				if(targetPos < 0) {
					//printf("pos clipped from %d\n", targetPos);
					targetPos = 0;
				} else if(targetPos > 1023 ) {
					//printf("pos clipped from %d\n", targetPos);
					targetPos = 1023;
				}
				
				uint16_t distance = abs((int16_t)targetPos - (int16_t)ServoPos[i]);

				servoControl[i].pos = targetPos;
				servoControl[i].dist = distance;

				if(distance > longestDistance){
					longestDistance = distance;
				}
			}
		}
		/**
			Calculate Omega-PageSpeed-PoseSpeed-Normalised:
			Servos move with 90 deg/sec when PageSpeed=32 and PoseSpeed=32
			At this point OmegaPPN is 1.00
		*/

		
		float OmegaPPN = ((float)(PageSpeed*PoseSpeed))/1024.0;

		int i;
		for(i=1;i < NUM_OF_SERVOS_ATTACHED; i++){

			//printf("dist %d / dist %d, max %d\n", servoControl[i].dist, longestDistance);
			// calculate servo speed
			double speed = (double)servoControl[i].dist/(double)longestDistance;

			speed  *= (double)505.67300322796609L;

			if(speed < 0)
				speed = 10;

			SendServoTargetPos(i,servoControl[i].pos,(int)speed);
			ServoPos[i] = servoControl[i].pos; // remember new target pose
		}
		
		/**
			Calculate how many time steps it will take to finish the move.
			The servo that moves furthest is used with the formulae t = theta/omega
			The conversion factors have to be taken into account:
			- for ThetaMax: one degree in servo position is 3.41 servo units
			- for OmegaPPN: OmegaPPN is 1 at 90 deg/sec
			Using the conversion above gives the time in seconds 
			- in order to change to timesteps divide by 7.8msec
			All in all results in 0.41707
		*/
		MHD.MoveFinishTime = (uint16_t)(0.41707*((float)longestDistance/OmegaPPN));
		if(MHD.MoveFinishTime > 30*128){printf("[E:FinishTime too long?]");}
		//sprintf(temp,"MFT=%i,TM=%i,OPPM=%i\n\r",MHD.MoveFinishTime,ThetaMax,(int)(OmegaPPN*1024.0));TxDString(temp); 
			
		/* if PageSpeed is set to 32, a Pause in the Pose of 128 will last 1 second */
		PosePause = (PosePause * 32)/PageSpeed;	

	}else{
			// Page is -1
			return 0;
	}
			
	return PosePause;
}
