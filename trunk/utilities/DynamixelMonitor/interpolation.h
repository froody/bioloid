#ifndef __INTERPOLATION

#define __INTERPOLATION

#include "DynamixelComm.h"

// IMPORTANT: Interpolation
//-------------------------

// interpolation type
// can be set from console via setrobot 2 x, where x = 
enum INTERPOLATION_TYPES
{
	IPO_LINEAR,// 0: linear (works good on medium loads)
	IPO_PTP,// 1: ptp, constant acceleration (works good on medium loads)
	IPO_SIMPLE,// 2: simple (works best on heavy loads, f.e. humanoid, reason: you can use higher baudrates and better interpolation techniques on pc)
	IPO_PTP_SINE// 3: ptp, sinus wave acceleration
};

/* Smoothness Chart
Max 	Type 	Pause 	Ranking
---		----	-----	-------
10		3		57		okay
8	    0		99		good
*/

#define BEST_TYPE 0
#define BEST_MAX 8
#define BEST_PAUSE 0 // automatic
// number of keyframes (interpolation points between two target positions sent from pc) 
// can be set from console via setrobot 0 x
#define IPOMAX BEST_MAX

// pause between two keyframes
// can be set from console via setrobot 1 x
#define IPOPAUSE BEST_PAUSE // 0 = autoset 

#define IPOTYPE BEST_TYPE
#define IPOPARAM 0 // readOp == 1 -> 0 async 1 sync ptp

#define IPOVMAX 750 // setrobot 3 x
#define IPOBMAX 50 // setrobot 4 x

#define IPOTOTALTIME 800
#define IPOAUTOADJUSTPARAMETER 400

#define WRITETIMEDIFFMAX 2000
#define WRITETIMEDIFFLEN 11
#define WRITETIMEDIFFREMOVE 3
#define WRITEBUFFERMAXPAUSE 2000

// size of position data cache
#define WRITEBUFFERMAX 12


struct InterpolationData
{
	unsigned long ipoCounter; // counts write cycles last read
	unsigned long ipoMax;  // max number of write cycles between two read cycles
	unsigned long ipoType; // interpolation type
	unsigned long ipoVMax; // max velocity
	unsigned long ipoBMax; // max accleration
	unsigned long ipoPause; // pause in Milliseconds between two write cycles + time to write/read
	bool 			ipoPauseAutoSet; // set ipoPause automatically based on ipoTotalTime and writeTimeDiffAverage
	unsigned long ipoParam;
	unsigned long ipoTotalTime; // total time of one p2p move
	float          ipoAutoAdjustParameter;
	bool preparationDone; // true if parameters above were intialized
};

struct RobotData
{
	// read/write Buffer
	// stores current, torque and position of every ax12, 
	// updated approx. every (ipoMax * ipoPause) Milliseconds
	byte readBuffer[CRC_LEN + AX12_COUNT*AX12_DATA_READ]; 
 
	// cyclic writebuffer (stores target values for speed, torque and position)
	byte writeBuffer[WRITEBUFFERMAX][AX12_COUNT*AX12_DATA_WRITE];
	byte writeLastBuffer[AX12_COUNT*AX12_DATA_WRITE];
	bool writeLastBufferIsValid;
	
	byte* writeStartBuffer;
	byte writeBufferIndex; // current index in writeBuffer
	byte writeBufferLength; // number of valid items in writeBuffer
	
	 unsigned long readTime; // timestamp of last read action (from ax12s)
	unsigned long writeTime; // timestamp of last write action (to ax12s)
	unsigned int writeTimeDiffAverage; // average length of one p2p move
	unsigned int writeTimeDiffCounter; // current index of writeTimeDiffs
	unsigned int writeTimeDiffs[WRITETIMEDIFFLEN]; // stores lengths of multiple p2p movs
	unsigned int readTimeDiff; // time difference between last two read actions
	
	
};

struct SCurveParameters
{
	// scurve parameters
	float tb[AX12_COUNT];
	float te[AX12_COUNT];
	int   sgn[AX12_COUNT];
	float vmax[AX12_COUNT];
	float bmax[AX12_COUNT];
};

void setInterpolationData(struct InterpolationData &ipoData, struct RobotData &robotData, byte id, byte *buffer, int len);
void initInterpolationData(struct InterpolationData &ipoData);
void initRobotData(struct RobotData &robotData);
void initSCurveParameters(struct SCurveParameters &params);
byte doInterpolation(DynamixelComm *dc, struct RobotData &robotData, struct InterpolationData &ipoData, struct SCurveParameters &params);
void doPreparation(struct RobotData &robotData, struct InterpolationData &ipoData, struct SCurveParameters &params);
void doWriteData(struct RobotData &robotData, struct InterpolationData &ipoData, struct SCurveParameters &params);

// read from ALL ax
void readPositionData(DynamixelComm *dc, byte* buffer, byte &failure);

// write position data to all ax
// simple linear interpolation (NOT very smooth)
void writePositionData(DynamixelComm *dc, byte* readBuffer, byte* writeBuffer, long int pos, long int max);

// simple ptp interpolation (conststant acceleration)
void writePositionDataPTPPrepare(unsigned long  ipoTime, unsigned long  totalTime, unsigned long  time, unsigned long  max, unsigned long param, byte* readBuffer, byte* writeBuffer, float* tbs, float* tes, int* sgns, float* vmaxs, float* bmaxs);
void writePositionDataPTP(DynamixelComm *dc, unsigned long  readPause, byte* readBuffer,byte* writeBuffer, float pos, float* tb, float* te, int* sgn, float* vmax, float* bmax);


// simple ptp interpolation (sinus wave acceleration)
void writePositionDataPTPPrepareSinus(unsigned long  ipoTime, unsigned long  totalTime,unsigned long  time, unsigned long  max, unsigned long param, byte* readBuffer, byte* writeBuffer, float* tbs, float* tes, int* sgns, float* vmaxs, float* bmaxs);
void writePositionDataPTPSinus(DynamixelComm *dc, unsigned long  readPause, byte* readBuffer,byte* writeBuffer, float pos, float* tb, float* te, int* sgn, float* vmax, float* bmax);

#endif
