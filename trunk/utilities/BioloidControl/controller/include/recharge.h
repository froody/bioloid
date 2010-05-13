#ifndef __RECHARGE

#define __RECHARGE

#include "types.h"

// still testing
#define RECHARGECYCLES 30
#define RECHARGEREMOVECYCLES 4
#define RECHARGEPAUSE 100000
#define RECHARGEMAX 10 // 10 cycles a 30 seconds 5mins = 300 seconds without change in batterie voltage

// recharge batteries functions
// thanks to kess and pieddemamouth (robosavvy-forums)
unsigned int readADC(byte channel);
float getTrimmedAverage(unsigned int *voltage, int len, int trim);
void rechargeBatteries(unsigned int maxCycles);


#endif