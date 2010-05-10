#include "bioloid.h"

#include "robot.h"
#include "BLC-Protocol.h"
#include "led.h"


#define POSE_BASE_ADR		(0x1E000UL)
#define	POSE_0_OFFSET		(0x00040UL)
#define POSE_SIZE			(0x00040UL)
#define POSE_NUMBER_OF_POSES_PER_PAGE		7

// within a page
#define POSE_PAGE_SIZE					(0x200UL)
#define POSE_PAGE_FAST_FLAG				(0x00010UL)
#define POSE_PAGE_PLAYCOUNT				(0x0000FUL)
#define POSE_PAGE_NUM_OF_MOTIONS		(0x00014)
#define POSE_PAGE_MOTION_SPEED			(0x00016UL)
#define POSE_PAGE_ACCEL_TIME			(0x00018UL)
#define POSE_PAGE_NEXT_PAGE				(0x00019UL)
#define POSE_PAGE_EXIT_PAGE				(0x0001AUL)

// Within a pose
#define POSE_PAUSE_ADR			0x0003E
#define POSE_SPEED_ADR			0x0003F



#define	SERVO_NOT_CONNECTED			9999


#ifndef MOTION_H
#define MOTION_H
typedef struct
{
	uint16_t CurrentPage;
	uint16_t TotalTime;
	uint16_t PosePause;
	uint16_t MoveFinishTime;		// move finishes, pause starts
	uint8_t NoOfPoses;
	uint8_t CurrentPose;

} MOTION_HEARTBEAT_DATA;

#endif

#define BALANCE_OFF	0
#define BALANCE_ON	1

int getCurrentServoPos(uint8_t ServoNo);
void SendServoTargetPos(uint8_t ServoNo, int16_t ThetaTarget,uint16_t OmegaServo);

extern volatile int ServoTrim[NUM_OF_SERVOS_ATTACHED];
extern volatile int ServoPos[NUM_OF_SERVOS_ATTACHED];
extern volatile uint8_t Balance;
extern volatile MOTION_HEARTBEAT_DATA MHD;
