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
 
#ifndef __COMMUNICATION
#define __COMMUNICATION

#include ".\interpolation.h"

// max length of messages from pc
#define MSGBUFFERMAX 80U

// Communication-Protocol
enum COMM_TYPES
{
	COMM_NONE,  //INVALID
	COMM_RFC, 	//read from console
	COMM_WTC,	//write to console
	COMM_RFAX,	//read from ax
	COMM_WTAX,	//write to ax
	COMM_TIME,	//get time
	COMM_RBS,	//read button state
	COMM_RLS,	//read led state
	COMM_WLS,	//write led state
	COMM_RSAX,	//run scheduled ax
	COMM_RFAAX,	//read from ALL ax
	COMM_WTAAX,  // write to ALL ax
	COMM_PARAM, // set params
	COMM_RECHARGE, // recharge batteries
	COMM_TOTAL,
	COMM = 255
};

// min. number of bytes to read per command from serial
const int COMM_LENGTH[COMM_TOTAL] = 
{
	0,
	0, 	//read from console
	0,	//write to console
	3,	//read from ax
	4,	//write to ax
	0,	//get time
	1,	//read button state
	1,	//read led state
	2,	//write led state
	0,	//run scheduled ax
	0,	//read from ALL ax
	CRC_LEN + AX12_DATA_WRITE_TOTAL,  // write to ALL ax
	3, // set params
	2 // recharge
};

struct MessageData
{
	byte msgBuffer[MSGBUFFERMAX];
	int msgIndex; // index of last byte in buffer
	bool msgProcessed; // true if first msg should be deleted from the msgBuffer
	unsigned int data; // uart data
};

void sendStatusInformation(byte* buffer, struct RobotData &robotData, struct InterpolationData &ipoData, struct SCurveParameters &params);
void initMessageData(struct MessageData &msgData);
void readData(byte *bData, int len);
void writeData(byte *bData, int len);

void TxD81(byte bTxdData);
void TxD80(byte bTxdData);
byte RxD81(void);
byte RxD81Async(void);
void PortInitialize(void);
void SerialInitialize(byte bPort, byte bBaudrate, byte bInterrupt);
byte TxPacket(byte bID, byte bInstruction, byte bParameterLength);
byte RxPacket(byte bRxLength);

// --- Global Variable Number ---
extern volatile byte gbpRxInterruptBuffer[256]; 
extern byte gbpParameter[128];
extern byte gbRxBufferReadPointer;
extern byte gbpRxBuffer[128]; 
extern byte gbpTxBuffer[128]; 
extern volatile byte gbRxBufferWritePointer;

#endif