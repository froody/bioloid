// system
#include <libc.h>
#include <mach/mach_time.h>

#include <math.h>
// User
#include "constants.h"
#include "types.h"
#include "communication.h"
#include "timer.h"
#include "crc.h"
#include "uart.h"
#include "interpolation.h"
#include "recharge.h"



void initInterpolationData(struct InterpolationData &ipoData)
{
	ipoData.ipoMax = IPOMAX;  // max number of write cycles between two read cycles
	ipoData.ipoCounter = ipoData.ipoMax + 1; // counts write cycles last read
	//ipoData.ipoType = IPOTYPE; // interpolation type
	ipoData.ipoType = IPO_PTP_SINE; // interpolation type
	ipoData.ipoVMax = IPOVMAX; // max velocity
	ipoData.ipoBMax = IPOBMAX; // max accleration
	ipoData.ipoPause = IPOPAUSE; // pause in Milliseconds between two write cycles + time to write/read
	ipoData.ipoPauseAutoSet = (ipoData.ipoPause == 0); // set ipoData.ipoPause automatically based on ipoData.ipoTotalTime and robotData.writeTimeDiffAverage
	ipoData.ipoParam = IPOPARAM;
	ipoData.ipoTotalTime = IPOTOTALTIME; // total time of one p2p move
	ipoData.preparationDone = false; // true if parameters above were intialized
	
	ipoData.ipoAutoAdjustParameter = IPOAUTOADJUSTPARAMETER / 1000.0;
}

void initRobotData(struct RobotData &robotData)
{
	robotData.writeStartBuffer = robotData.readBuffer;
	robotData.writeBufferIndex = 0; // current index in robotData.writeBuffer
	robotData.writeBufferLength = 0; // number of valid items in robotData.writeBuffer
	
	robotData.readTime = gettickcount(); // timestamp of last read action (from ax12s)
	robotData.writeTime = robotData.readTime; // timestamp of last write action (to ax12s)
	robotData.writeTimeDiffAverage = 0; // average length of one p2p move
	robotData.writeTimeDiffCounter = 0; // current index of robotData.writeTimeDiffs
	
	robotData.writeLastBufferIsValid = false;
	
	robotData.readTimeDiff = 0; // time difference between last two read actions
}

void setInterpolationData(struct InterpolationData &ipoData, struct RobotData &robotData, byte id, byte *buffer, int len)
{
	switch (id)
	{
		case 0: ipoData.ipoMax = buffer[0] + 256 * buffer[1]; break;
		case 1: 
		ipoData.ipoPause = buffer[0] + 256 * buffer[1]; 
		ipoData.ipoPauseAutoSet = (ipoData.ipoPause == 0);
		break;
		case 2: ipoData.ipoType =buffer[0] + 256 * buffer[1];break;
		case 3: ipoData.ipoVMax =buffer[0] + 256 * buffer[1];break;
		case 4: ipoData.ipoBMax = buffer[0] + 256 * buffer[1];break;
		case 5: ipoData.ipoParam = buffer[0] + 256 * buffer[1];break;
		case 6: ipoData.ipoTotalTime = buffer[0] + 256 *buffer[1];break;
		
		case 7: ipoData.ipoAutoAdjustParameter = ((float)(buffer[0] + 256 *buffer[1])) / 1000.0;break;
	}
	
	switch (id)
	{
		case 0:
		case 1:
		case 5:
		case 6:
		case 7:
		robotData.writeTimeDiffAverage = ipoData.ipoTotalTime;
		robotData.writeTimeDiffCounter = 0;
		break;
	}					
}

void initSCurveParameters(struct SCurveParameters &params)
{
	// nothing
}

void doPreparation(struct RobotData &robotData, struct InterpolationData &ipoData, struct SCurveParameters &params)
{
	if (robotData.writeBufferLength > 0 && !ipoData.preparationDone)
	{
		switch (ipoData.ipoType)
		{
			case IPO_PTP:
			writePositionDataPTPPrepare(ipoData.ipoPause, ipoData.ipoTotalTime, ipoData.ipoVMax, ipoData.ipoBMax, ipoData.ipoParam, robotData.writeStartBuffer, &robotData.writeBuffer[robotData.writeBufferIndex][0], params.tb, params.te, params.sgn, params.vmax, params.bmax);
			break;
						
			case IPO_PTP_SINE:
			writePositionDataPTPPrepareSinus(ipoData.ipoPause, ipoData.ipoTotalTime, ipoData.ipoVMax, ipoData.ipoBMax, ipoData.ipoParam, robotData.writeStartBuffer, &robotData.writeBuffer[robotData.writeBufferIndex][0], params.tb, params.te, params.sgn, params.vmax, params.bmax);
			break;
			
			case IPO_LINEAR:
			//nothing
			break;
		}
	
		ipoData.preparationDone = true;
	}
}

static void doWriteData(DynamixelComm *dc, struct RobotData &robotData, struct InterpolationData &ipoData, struct SCurveParameters &params)
{
	//if (robotData.writeBufferLength > 0 && ipoData.preparationDone)
	switch (ipoData.ipoType)
	{
		case IPO_PTP:
		writePositionDataPTP(dc, ipoData.ipoPause, robotData.writeStartBuffer, &robotData.writeBuffer[robotData.writeBufferIndex][0], (float)ipoData.ipoCounter/(float)ipoData.ipoMax, params.tb, params.te, params.sgn, params.vmax, params.bmax); 
		break;
					
		case IPO_PTP_SINE:
		writePositionDataPTPSinus(dc, ipoData.ipoPause, robotData.writeStartBuffer, &robotData.writeBuffer[robotData.writeBufferIndex][0], (float)ipoData.ipoCounter/(float)ipoData.ipoMax, params.tb, params.te, params.sgn, params.vmax, params.bmax); 
		break;
		
		case IPO_LINEAR:
		writePositionData(dc, robotData.writeStartBuffer, &robotData.writeBuffer[robotData.writeBufferIndex][0], ipoData.ipoCounter, ipoData.ipoMax);
		break;
	}
}

void setWriteStartBuffer(struct RobotData &robotData, struct InterpolationData &ipoData, struct SCurveParameters &params)
{				
	if (robotData.writeLastBufferIsValid && 
		((gettickcount() - robotData.writeTime) < WRITEBUFFERMAXPAUSE))
	{
		
		//robotData.writeStartBuffer = robotData.writeLastBuffer;
	} else
	robotData.writeStartBuffer = robotData.readBuffer;
}

byte doInterpolation(DynamixelComm *dc, struct RobotData &robotData, struct InterpolationData &ipoData, struct SCurveParameters &params)
{
	byte failure = AX12_NOERROR;
	unsigned long tmpTime = gettickcount();
	
	if (tmpTime >= robotData.readTime + ipoData.ipoPause)
	{
		robotData.readTimeDiff = tmpTime - robotData.readTime;
		robotData.readTime = tmpTime;

		printf("past the time - %d, pause = %d\n", tmpTime, ipoData.ipoPause);
		
		// read or write every ipoData.ipoPause Milliseconds
		if (ipoData.ipoCounter <= ipoData.ipoMax)
		{
			// if at least two points are available
			if ((robotData.writeBufferLength > ipoData.ipoParam) && ipoData.preparationDone)
			{
				/*
				if (!ipoData.preparationDone)
				{
					setWriteStartBuffer(robotData, ipoData, params);
				
					// preprocessing
					doPreparation(robotData, ipoData, params);
				}
				*/
				
				// write (interpolated) data to servos
				doWriteData(dc, robotData, ipoData, params);
				
				// remove data from robotData.writeBuffer at end of interpolation cycle
				if (ipoData.ipoCounter >= ipoData.ipoMax)
				{
					ipoData.preparationDone = false;
					
					robotData.writeBufferLength--;
					byte oldWriteBufferIndex = robotData.writeBufferIndex;
					robotData.writeBufferIndex = (robotData.writeBufferIndex + 1) % WRITEBUFFERMAX;
					
					// copy processed write target to last buffer
					// modified:
					//memcpy(robotData.writeLastBuffer, &robotData.writeBuffer[oldWriteBufferIndex][0], AX12_COUNT*AX12_DATA_WRITE*sizeof(byte));
					robotData.writeStartBuffer = &robotData.writeBuffer[oldWriteBufferIndex][0];
					robotData.writeLastBufferIsValid = true;
					
					if (robotData.writeBufferLength > 0)
					{
						// modified:
						//robotData.writeStartBuffer = robotData.writeLastBuffer;
						
						// preprocessing
						doPreparation(robotData, ipoData, params);
					}
					
					// stuff read op between this and the next write op
					
					// modified:
					robotData.readTime -= ipoData.ipoPause / 2;
					//robotData.readTime -= (ipoData.ipoPause * 3)/ 4;
					
					// calc average length of one write cycle
					tmpTime = robotData.writeTime;
					robotData.writeTime = gettickcount();
					
					if ((robotData.writeTime - tmpTime) < WRITEBUFFERMAXPAUSE)
					{
						robotData.writeTimeDiffAverage = (unsigned int) (0.7 * (float)robotData.writeTimeDiffAverage + 
															0.3 * (float) (robotData.writeTime - tmpTime));
		
						if (ipoData.ipoPauseAutoSet)
						{
							float tmp = ((float)ipoData.ipoTotalTime * (float)ipoData.ipoTotalTime)/ ((float)ipoData.ipoMax * (float)robotData.writeTimeDiffAverage);
							if (robotData.writeBufferLength > 1)
							{	
								float tmp2 = (float) (robotData.writeBufferLength-1);
							
								tmp *= 1.0 - ipoData.ipoAutoAdjustParameter + 
								ipoData.ipoAutoAdjustParameter*(1-(tmp2 * tmp2)/64.0);
							}
							ipoData.ipoPause = (unsigned long) (tmp + 0.5);
						}
					}
					
					/*
					if ((robotData.writeTime - tmpTime) < WRITEBUFFERMAXPAUSE)
					{
						robotData.writeTimeDiffs[robotData.writeTimeDiffCounter] = (robotData.writeTime - tmpTime);
		
						if (robotData.writeTimeDiffs[robotData.writeTimeDiffCounter] > WRITETIMEDIFFMAX)
							robotData.writeTimeDiffs[robotData.writeTimeDiffCounter] = WRITETIMEDIFFMAX;
							
						robotData.writeTimeDiffCounter++;
						if (robotData.writeTimeDiffCounter == WRITETIMEDIFFLEN)
						{
							robotData.writeTimeDiffCounter = 0;
							robotData.writeTimeDiffAverage = (unsigned int) (getTrimmedAverage(robotData.writeTimeDiffs, WRITETIMEDIFFLEN, WRITETIMEDIFFREMOVE));
						}
						
						if (ipoData.ipoPauseAutoSet)
						{
							float tmp = ((float)ipoData.ipoTotalTime * (float)ipoData.ipoTotalTime)/ ((float)ipoData.ipoMax * (float)robotData.writeTimeDiffAverage);
							if (robotData.writeBufferLength > 1)
							{	
								float tmp2 = (float) (robotData.writeBufferLength-1);
							
								tmp *= 1.0 - ipoData.ipoAutoAdjustParameter + 
								ipoData.ipoAutoAdjustParameter*(1-(tmp2 * tmp2)/64.0);
							}
							ipoData.ipoPause = (unsigned int) (tmp + 0.5);
						}
						robotData.writeTimeDiffAverage = ((unsigned int)((float)robotData.writeTimeDiffAverage / (float)ipoData.ipoPause + 0.5)) * ipoData.ipoPause;
					}
					*/
				}
			}
		} 
		// modified:
		else
		// read ax12 data every (ipoData.ipoMax + 1) +ipoData.ipoPause Milliseconds
		//if (ipoData.ipoCounter >= ipoData.ipoMax)
		{
			failure = AX12_NOERROR;
			if (!ipoData.preparationDone) // comment this line in order to produce position feedback during write actions
				readPositionData(dc, robotData.readBuffer, failure);
			
			
			if (failure != AX12_NOERROR)
			{
				robotData.readTime = gettickcount() - ipoData.ipoPause + AX12_READERROR_TIMEOUT;
				return failure;
			}
				
			
			ipoData.ipoCounter = 0;
			
			if (failure == AX12_NOERROR)
			if (!ipoData.preparationDone)
			{
				setWriteStartBuffer(robotData, ipoData, params);
			
				// preprocessing
				doPreparation(robotData, ipoData, params);
			} 
			// stuff read op between two write ops 
			
			// modified:
			robotData.readTime -= ipoData.ipoPause / 2;
			//robotData.readTime -= ipoData.ipoPause / 4;
		} 
		
		ipoData.ipoCounter++;
	}	
	
	return failure;
}

// read from ALL ax
void readPositionData(DynamixelComm *dc, byte* buffer, byte &failure)
{		
	printf("%s! %d != %d\n", __func__, AX12_DATA_READ, AX12_DATA_WRITE);
	int id, i, len;
	// command: read AX12_DATA_READ bytes from address P_PRESENT_POSITION_L /ax12
	gbpParameter[0] = P_PRESENT_POSITION_L; //Address
	gbpParameter[1] = AX12_DATA_READ;  //Read Length
	
	// send command to every ax12 and store result in readBuffer[]
	// failure = 1 signals a read failure
	failure = AX12_NOERROR;
	for(id=0; id<AX12_COUNT; id++)
	{
		failure = AX12_ERROR;
		i = 0;
		// on failure: retry AX12_RETRY times
		while ( (failure == AX12_ERROR) && (i < AX12_RETRY))
		{ 
			TxPacket(dc, id + AX12_STARTID,INST_READ,2);
			
			
			int len = RxPacket(dc, DEFAULT_RETURN_PACKET_SIZE+gbpParameter[1]);
			//if(RxPacket(dc, DEFAULT_RETURN_PACKET_SIZE+gbpParameter[1]) == DEFAULT_RETURN_PACKET_SIZE+gbpParameter[1])
			if(len == DEFAULT_RETURN_PACKET_SIZE+gbpParameter[1])
			{	
				for (len=0; len<AX12_DATA_READ; len++) {
					buffer[id*AX12_DATA_READ + len] = gbpRxBuffer[5 + len];
					printf("%d:%x ", id*AX12_DATA_READ + len, buffer[id*AX12_DATA_READ + len]);
				}
					
				failure = AX12_NOERROR;
			} else 
			{
				i++;
			}
		}
		
		if (failure == AX12_ERROR)
			break;
	}

	printf("\n");
}

// write position data to all ax
// simple linear interpolation (NOT very smooth)
void writePositionData(DynamixelComm *dc, byte* readBuffer, byte* writeBuffer, long int pos, long int max)
{
	// sync write AX12_DATA_WRITE bytes starting at address P_GOAL_POSITION_L
	byte op = INST_SYNC_WRITE;
	gbpParameter[0] = P_GOAL_POSITION_L; //Address
	gbpParameter[1] = AX12_DATA_WRITE;
			
	// conversion variables
	int i;
	long int current, target;
	long int tmp;
	double cp = (double)pos / (double)max;

	printf("%s joints: ", __func__);
	// prepare write buffer -> unpack data from serial and store bytewise in buffer
	for (i=0;i<AX12_COUNT;i++)
	{
		gbpParameter[2 + i * (AX12_DATA_WRITE+1)] = i + AX12_STARTID; 
			
		// simple linear interpolation 
		target = writeBuffer[i * AX12_DATA_WRITE + 0] + 256 * writeBuffer[i * AX12_DATA_WRITE + 1];
		current = readBuffer[i * AX12_DATA_WRITE + 0] + 256 * readBuffer[i * AX12_DATA_WRITE + 1];
		
		tmp = target - current;
		
		double tmp2 = (double)tmp * cp;			
		current += (long int)tmp2;
		
		if (current > 1023)
			current = 1023;
		else if (current < 0)
			current = 0;
		
		gbpParameter[3 + i * (AX12_DATA_WRITE+1) + 0] = current & 0xFF; 
		gbpParameter[3 + i * (AX12_DATA_WRITE+1) + 1] = (current & 0xFF00) >> 8;

		printf("%04x ", current);
		
		// store target speed and torque in write buffer
		gbpParameter[3 + i * (AX12_DATA_WRITE+1) + 2] = writeBuffer[i * AX12_DATA_WRITE + 2]; 
		gbpParameter[3 + i * (AX12_DATA_WRITE+1) + 3] = writeBuffer[i * AX12_DATA_WRITE + 3];
		gbpParameter[3 + i * (AX12_DATA_WRITE+1) + 4] = writeBuffer[i * AX12_DATA_WRITE + 4]; 
		gbpParameter[3 + i * (AX12_DATA_WRITE+1) + 5] = writeBuffer[i * AX12_DATA_WRITE + 5];
		
	}
	printf("\n");
				
	// send all data to ax12s AT ONCE
	TxPacket(dc, BROADCASTING_ID,op,2+AX12_COUNT*(AX12_DATA_WRITE + 1));
}  

// ptp movement (sinus wave acceleration)
void writePositionDataPTPPrepareSinus(unsigned long  ipoTime, unsigned long totalTime, unsigned long  time, unsigned long  max, unsigned long param, byte* readBuffer, byte* writeBuffer, float* tbs, float* tes, int* sgns, float* vmaxs, float* bmaxs)
{
	printf("%s!\n", __func__);
   int i;
   float current, target;
   
  /* float MAXS = max / 10.0;
   float TIME = totalTime;
   float BTIME = (float)time / 100.0; 
   float VMAX = (MAXS / ((1.0 - BTIME) * TIME)) ;
   float BMAX = (2.0 * VMAX / (BTIME * TIME));
   */
   float TIME = totalTime;
   float VMAX = time / 1000.0;
   float BMAX = max / 1000.0;
   
   float vmax, bmax, se;
   long int te, tb, tv, sgn;
   int ipo = ipoTime;
      
   // prepare write buffer -> unpack data from serial and store bytewise in buffer
   for (i=0;i<AX12_COUNT;i++)
   {   
	  target = writeBuffer[i * AX12_DATA_WRITE + 0] + 256 * writeBuffer[i * AX12_DATA_WRITE + 1];
	  current = readBuffer[i * AX12_DATA_WRITE + 0] + 256 * readBuffer[i * AX12_DATA_WRITE + 1];

	  printf("t %04x c %04x ", target, current);
	
      vmax = VMAX;
      bmax = BMAX;
      se = (target - current);
      if (se < 0)
      {
         sgn = -1;
         se = -se;
      }
      else sgn = 1;
      
      if (vmax*vmax > bmax * se * 0.5)
      {
         vmax =  sqrt(bmax * se * 0.5);
         
         tb = ((long int)( 2.0 * vmax / (bmax * ipo))) * ipo;
         tv = (long int) (TIME - tb);
      } else
      {
         tb = ((long int)( 2.0 * vmax / (bmax * ipo) + 0.5)) * ipo;
         tv = (long int)(TIME - tb);
      }
      
      if (tb == 0)
         tb = 1;
        if (tv == 0)
           tv = 1;

      te = tv + tb;
      vmax = se / (float)tv;
      bmax = 2.0 * vmax / (float)tb;
      
      tes[i] = te;
      sgns[i] = sgn;
      tbs[i] = tb;
      vmaxs[i] = vmax;
      bmaxs[i] = bmax;
   }
   printf("\n");
}

// ptp movement (sinus wave acceleration)
void writePositionDataPTPSinus(DynamixelComm *dc, unsigned long  readPause, byte* readBuffer,byte* writeBuffer, float pos, float* tb, float* te, int* sgn, float* vmax, float* bmax)
{
   int i;
   long int current;
   float se, t, tv;
   
	// sync write AX12_DATA_WRITE bytes starting at address P_GOAL_POSITION_L
	byte op = INST_SYNC_WRITE;
	gbpParameter[0] = P_GOAL_POSITION_L; //Address
	gbpParameter[1] = AX12_DATA_WRITE;
	
	printf("%s joints: ", __func__);
	// prepare write buffer -> unpack data from serial and store bytewise in buffer
	for (i=0;i<AX12_COUNT;i++)
	{   
	    gbpParameter[2 + i * (AX12_DATA_WRITE+1)] = i + 1; 
		
		current = readBuffer[i * AX12_DATA_WRITE + 0] + 256 * readBuffer[i * AX12_DATA_WRITE + 1];
		  
      t = pos * te[i];
      tv = te[i] - tb[i];
      if (t <= tb[i]) {
		  printf("A");
         se = bmax[i] * (0.25 * t * t + (tb[i] * tb[i] / (8.0 * M_PI * M_PI)) * (cos(2.0*M_PI*t/tb[i])-1.0));
	  } else if (t <= te[i] - tb[i]) {
		  printf("B");
         se = vmax[i] * (t - 0.5 * tb[i]);
	  } else {
		  printf("C");
         se = 0.5 * bmax[i] * (te[i] * (t + tb[i]) - 0.5*(t*t + te[i]*te[i] + 2.0 * tb[i]*tb[i]) + (tb[i]*tb[i]/(4.0 * M_PI * M_PI))*(1.0 - cos((2.0*M_PI/tb[i])*(t - tv))));
	  }
         
      current +=  sgn[i] * (long int) (se);
   
      if (current > 1023)
         current = 1023;
      else if (current < 0)
         current = 0;
         

    	gbpParameter[3 + i * (AX12_DATA_WRITE+1) + 0] = current & 0xFF; 
		gbpParameter[3 + i * (AX12_DATA_WRITE+1) + 1] = (current & 0xFF00) >> 8;
		//printf("%04x ", current);
		printf("%0d/%04lx:%04x", sgn[i], (long int) (se), current);
		
		/*
		if (i == 13)
		{
		uart1_putc(current & 0xFF);
		uart1_putc((current & 0xFF00) >> 8);
		}
		*/
		
    	// store target speed and torque in write buffer
		gbpParameter[3 + i * (AX12_DATA_WRITE+1) + 2] = writeBuffer[i * AX12_DATA_WRITE + 2]; 
		gbpParameter[3 + i * (AX12_DATA_WRITE+1) + 3] = writeBuffer[i * AX12_DATA_WRITE + 3];
		gbpParameter[3 + i * (AX12_DATA_WRITE+1) + 4] = writeBuffer[i * AX12_DATA_WRITE + 4]; 
		gbpParameter[3 + i * (AX12_DATA_WRITE+1) + 5] = writeBuffer[i * AX12_DATA_WRITE + 5];
		
    }
	printf("\n");
				
	// send all data to ax12s AT ONCE
	TxPacket(dc, BROADCASTING_ID,op,2+AX12_COUNT*(AX12_DATA_WRITE + 1));
}

// ptp movement (constant acceleration)
void writePositionDataPTPPrepare(unsigned long  ipoTime, unsigned long totalTime, unsigned long  time, unsigned long  max, unsigned long param, byte* readBuffer, byte* writeBuffer, float* tbs, float* tes, int* sgns, float* vmaxs, float* bmaxs)
{
   // conversion variables
   int i;
   float current, target;
   
  /* float MAXS = max / 10.0;
   float TIME = totalTime;
   float BTIME = (float)time / 100.0; 
   float VMAX = (MAXS / ((1.0 - BTIME) * TIME)) ;
   float BMAX = (VMAX / (BTIME * TIME));
   */
   float TIME = totalTime;
   float VMAX = time / 1000.0;
   float BMAX = max / 1000.0;
          
       
   float vmax, bmax, se;
   long int te, tb, tv, sgn;
   int ipo = ipoTime;
      
   // prepare write buffer -> unpack data from serial and store bytewise in buffer
   for (i=0;i<AX12_COUNT;i++)
   {   
		target = writeBuffer[i * AX12_DATA_WRITE + 0] + 256 * writeBuffer[i * AX12_DATA_WRITE + 1];
		current = readBuffer[i * AX12_DATA_WRITE + 0] + 256 * readBuffer[i * AX12_DATA_WRITE + 1];
	
   
      vmax = VMAX;
      bmax = BMAX;
      se = (target - current);
      if (se < 0)
      {
         sgn = -1;
         se = -se;
      }
      else sgn = 1;
      
      if (vmax*vmax > bmax * se)
      {
         vmax =  sqrt(bmax * se);
         
         tb = ((long int)( vmax / (bmax * ipo))) * ipo;
         tv = (long int)(TIME - tb);
      } else
      {      
         tb = ((long int)( vmax / (bmax * ipo) + 0.5)) * ipo;
         tv = (long int)(TIME - tb);
      }
      
      if (tb == 0)
         tb = 1;
        if (tv == 0)
           tv = 1;

      te = tv + tb;
      vmax = se / (float)tv;
      bmax = vmax / (float)tb;
      
      tes[i] = te;
      sgns[i] = sgn;
      tbs[i] = tb;
      vmaxs[i] = vmax;
      bmaxs[i] = bmax;
   }
   
   switch (param)
   {
      case 1:
      // synchronous ptp
      
      // get max total time
      long int temax = 0;
      for (i=0; i<AX12_COUNT; i++)
           if (tes[i] > temax)
            temax = (long int)tes[i];
         
      // recalculate max velocity and acceleration times
      for (i=0; i<AX12_COUNT; i++)
      {
		target = writeBuffer[i * AX12_DATA_WRITE + 0] + 256 * writeBuffer[i * AX12_DATA_WRITE + 1];
		current = readBuffer[i * AX12_DATA_WRITE + 0] + 256 * readBuffer[i * AX12_DATA_WRITE + 1];

         se = (target - current);
         if (se < 0)
         {
            se = -se;
         }
         
         bmax = bmaxs[i];
         vmax = bmax * temax;
         vmax = 0.5 * vmax - sqrt(0.25*vmax*vmax - se * bmax);
         
         tv = ((long int)( se / (vmax * ipo) + 0.5)) * ipo;
         tb = temax - tv;
         vmax = se / (float)tv;
         bmax = vmax / (float)tb;
      
         tes[i] = temax;
         tbs[i] = tb;
         vmaxs[i] = vmax;
         bmaxs[i] = bmax;
      }
      break;
   }
}

// ptp movement (constant acceleration)
void writePositionDataPTP(DynamixelComm *dc, unsigned long  readPause, byte* readBuffer,byte* writeBuffer, float pos, float* tb, float* te, int* sgn, float* vmax, float* bmax)
{
	int i;
	long int current;
	float se, t;
   
	// sync write AX12_DATA_WRITE bytes starting at address P_GOAL_POSITION_L
	byte op = INST_SYNC_WRITE;
	gbpParameter[0] = P_GOAL_POSITION_L; //Address
	gbpParameter[1] = AX12_DATA_WRITE;
	
	printf("%s joints: ", __func__);
	// prepare write buffer -> unpack data from serial and store bytewise in buffer
	for (i=0;i<AX12_COUNT;i++)
	{   
	    gbpParameter[2 + i * (AX12_DATA_WRITE+1)] = i + 1; 
		
		current = readBuffer[i * AX12_DATA_WRITE + 0] + 256 * readBuffer[i * AX12_DATA_WRITE + 1];
		
		t = pos * te[i];
		if (t <= tb[i])
			se = 0.5 * bmax[i] * t * t;
		else if (t <= te[i] - tb[i])
			se = (vmax[i] * t - 0.5 * vmax[i] * vmax[i] / bmax[i]);
		else
			se = (vmax[i] * (te[i] - tb[i]) - 0.5 * bmax[i] * (te[i] - t) * (te[i] - t));
         
		current +=  sgn[i] * (long int) (se);
   
		if (current > 1023)
			current = 1023;
		else if (current < 0)
			current = 0;
         
		gbpParameter[3 + i * (AX12_DATA_WRITE+1) + 0] = current & 0xFF; 
		gbpParameter[3 + i * (AX12_DATA_WRITE+1) + 1] = (current & 0xFF00) >> 8;


		printf("%04x ", current);
		
    	// store target speed and torque in write buffer
		gbpParameter[3 + i * (AX12_DATA_WRITE+1) + 2] = writeBuffer[i * AX12_DATA_WRITE + 2]; 
		gbpParameter[3 + i * (AX12_DATA_WRITE+1) + 3] = writeBuffer[i * AX12_DATA_WRITE + 3];
		gbpParameter[3 + i * (AX12_DATA_WRITE+1) + 4] = writeBuffer[i * AX12_DATA_WRITE + 4]; 
		gbpParameter[3 + i * (AX12_DATA_WRITE+1) + 5] = writeBuffer[i * AX12_DATA_WRITE + 5];
		
    }
	printf("\n");
				
	// send all data to ax12s AT ONCE
	TxPacket(dc, BROADCASTING_ID,op,2+AX12_COUNT*(AX12_DATA_WRITE + 1));
}

volatile unsigned long gettickcount(void) {
	return mach_absolute_time() / 1000000ULL;
}

