/**************************************************************************
    
    Copyright 2007, 2008 Rainer Jäkel <rainer.jaekel@googlemail.com>

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    
**************************************************************************/


/*
 * Based On "Example of Dynamixel Evaluation with Atmega128 by BS KIM" 
 */

#include <inttypes.h>

#include "constants.h"
#include "types.h"
#include "communication.h"
#include "uart.h"
#include "interpolation.h"

#include "DynamixelComm.h"

// --- Global Variable Number ---
volatile byte gbpRxInterruptBuffer[256]; 
byte gbpParameter[128];
byte gbRxBufferReadPointer;
byte gbpRxBuffer[128]; 
byte gbpTxBuffer[128]; 
volatile byte gbRxBufferWritePointer;

void initMessageData(struct MessageData &msgData)
{
	msgData.msgIndex = 0;
	msgData.msgProcessed = false;
}

#if 0
void sendStatusInformation(byte* sendBuffer, struct RobotData &robotData, struct InterpolationData &ipoData, struct SCurveParameters &params)
{
	sendBuffer[0] = (robotData.writeTimeDiffAverage & 0xFF);
	sendBuffer[1] = (robotData.writeTimeDiffAverage & 0xFF00) >> 8;
			
	sendBuffer[2] = (robotData.readTimeDiff & 0xFF);
	sendBuffer[3] = (robotData.readTimeDiff & 0xFF00) >> 8;
			
	sendBuffer[4] = (ipoData.ipoPause & 0xFF);
	sendBuffer[5] = (ipoData.ipoPause & 0xFF00) >> 8;
			
	sendBuffer[6] = robotData.writeBufferLength & 0xFF;
	writeData((byte*)sendBuffer, 7);
}
#endif

/*
TxPacket() send data to RS485.
TxPacket() needs 3 parameter; ID of Dynamixel, Instruction byte, Length of parameters.
TxPacket() return length of Return packet from Dynamixel.
*/
byte TxPacket(DynamixelComm *dc, byte bID, byte bInstruction, byte bParameterLength) 
{
	//printf("TxPacket %d, %d, %d\n", bID, bInstruction, bParameterLength);

	byte bCount,bCheckSum,bPacketLength;

    gbpTxBuffer[0] = 0xff;
    gbpTxBuffer[1] = 0xff;
    gbpTxBuffer[2] = bID;
    gbpTxBuffer[3] = bParameterLength+2; //Length(Paramter,Instruction,Checksum)
    gbpTxBuffer[4] = bInstruction;
    
	for(bCount = 0; bCount < bParameterLength; bCount++)
    {
        gbpTxBuffer[bCount+5] = gbpParameter[bCount];
    }
    
	bCheckSum = 0;
    bPacketLength = bParameterLength+4+2;
    
	for(bCount = 2; bCount < bPacketLength-1; bCount++) //except 0xff,checksum
    {
        bCheckSum += gbpTxBuffer[bCount];
    }
    gbpTxBuffer[bCount] = ~bCheckSum; //Writing Checksum with Bit Inversion

	dc->Send(gbpTxBuffer);
    
	return(bPacketLength);
}

/*
RxPacket() read data from buffer.
RxPacket() need a Parameter; Total length of Return Packet.
RxPacket() return Length of Return Packet.
*/

byte RxPacket(DynamixelComm *dc, byte bRxPacketLength)
{
#define RX_TIMEOUT_COUNT2   3000L  
#define RX_TIMEOUT_COUNT1  (RX_TIMEOUT_COUNT2*10L)  
  unsigned long ulCounter;
  byte bCount, bLength, bChecksum;
  byte bTimeout;

  byte tmpBuffer[128];

  bTimeout = 0;
#if 0
  for(bCount = 0; bCount < bRxPacketLength; bCount++)
  {
    ulCounter = 0;
    
	int count;
	while(gbRxBufferReadPointer == gbRxBufferWritePointer)
    {
		//count = dc->ReceiveBytes((char *)tmpBuffer, 4);
		count = dc->Receive(tmpBuffer);
		//printf("RxPacket loop %d, %d, %d, %d, %d\n", bRxPacketLength, gbRxBufferReadPointer, gbRxBufferWritePointer, count, bCount);
		if(count == -1) {
			CLEAR_BUFFER;
			return 0;
		}

		int i;
		for(i=0;i<count;i++) {

			gbpRxInterruptBuffer[gbRxBufferWritePointer++] = tmpBuffer[i];
		}

		//printf("RxPacket xxxx %d, %d, %d, %d, %d\n", bRxPacketLength, gbRxBufferReadPointer, gbRxBufferWritePointer, count, bCount);
		if(count)
			break;
    }
    
	gbpRxBuffer[bCount] = gbpRxInterruptBuffer[gbRxBufferReadPointer++];
  }
#endif
  if((bCount = dc->Receive(gbpRxBuffer)) != bRxPacketLength) {
	  printf("packet read size fail: %d != %d\n", bCount, bRxPacketLength);
  } 

  bLength = bCount;
  bChecksum = 0;
  
  if(gbpTxBuffer[2] != BROADCASTING_ID)
  {
    if(bTimeout && bRxPacketLength != 255) 
    {
      //TxDString("\r\n [Error:RxD Timeout]");
      CLEAR_BUFFER;
    }
    
    if(bLength > 3) //checking is available.
    {
      if(gbpRxBuffer[0] != 0xff || gbpRxBuffer[1] != 0xff ) 
      {
        //TxDString("\r\n [Error:Wrong Header]");
        CLEAR_BUFFER;
        return 0;
      }
      
	  if(gbpRxBuffer[2] != gbpTxBuffer[2] )
      {
        //TxDString("\r\n [Error:TxID != RxID]");
        CLEAR_BUFFER;
        return 0;
      }  
      
	  if(gbpRxBuffer[3] != bLength-4) 
      {
        //TxDString("\r\n [Error:Wrong Length]");
        CLEAR_BUFFER;
        return 0;
      }  
      
	  for(bCount = 2; bCount < bLength; bCount++)
		bChecksum += gbpRxBuffer[bCount];
      
	  if(bChecksum != 0xff) 
      {
        //TxDString("\r\n [Error:Wrong CheckSum]");
        CLEAR_BUFFER;
        return 0;
      }
    }
  }
  return bLength;
}
