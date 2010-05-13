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
 
// system
#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/signal.h>
#include <util/delay.h>
#include <math.h>

// User
#include ".\include\constants.h"
#include ".\include\types.h"
#include ".\include\communication.h"
#include ".\include\timer.h"
#include ".\include\crc.h"
#include ".\include\uart.h"
#include ".\include\interpolation.h"
#include ".\include\recharge.h"

#define ENABLE_BIT_DEFINITIONS 

// cpu speed in hz
#define F_CPU 16000000UL 

/* default baud rate */
#define UART_BAUD_RATE_0 1000000UL  
#define UART_BAUD_RATE_1 115200UL 

/* 
	main function:
	main loop:
		read command from serial
		process command (read/write data from/to ax12s, 1xs1)
		send data to serial
*/
int main(void)
{  
  // init rs232 and rs485
  PortInitialize(); //Port In/Out Direction Definition
  RS485_RXD; //Set RS485 Direction to Input State.
  SerialInitialize(SERIAL_PORT0,1,RX_INTERRUPT);//RS485 Initializing(RxInterrupt)
  uart1_init( UART_BAUD_SELECT_DOUBLE_SPEED(UART_BAUD_RATE_1,F_CPU) );//RS232 Initializing

  gbRxBufferReadPointer = gbRxBufferWritePointer = 0;  //RS485 RxBuffer Clearing.
  
  // init timer
  init_timer();

  //Enable Interrupt -- Compiler Function	
  sei();  
  
  // signals a read failure (from ax12)
  byte failure;
  
  // misc
  byte cmd, id, addr, len, schedule, op;
  int i=0, j=0;
  
  struct RobotData robotData;
  initRobotData(robotData);
  
  // interpolation stuff
  struct InterpolationData ipoData;
  initInterpolationData(ipoData);
  robotData.writeTimeDiffAverage = ipoData.ipoTotalTime;
  
  struct SCurveParameters sCurveParams;
  initSCurveParameters(sCurveParams);
  
  // rs232 buffer
  byte sendBuffer[CRC_LEN + AX12_DATA_READ_TOTAL];
  
  // incoming bytes from rs232 will be stored here
  struct MessageData msgData;
  initMessageData(msgData);
  
  // stores checksum
  word crc;
  
  // stores tickcount
  longword time_tmp, time;
  
  // main loop
  while (true)
  {				
	//failure = AX12_NOERROR;
	
	// read data from servos and write new positions to servos
	failure = doInterpolation(robotData, ipoData, sCurveParams);
	
	// check if new data is available from rs232
	//while (true) 
	{
		msgData.data = uart1_getc();
		if ( !( msgData.data & UART_NO_DATA ) && msgData.msgIndex < MSGBUFFERMAX)
		{
			// store data in buffer
			msgData.msgBuffer[msgData.msgIndex] = msgData.data & 0xFF;
			
			if (msgData.msgBuffer[0] == COMM)
				msgData.msgIndex++;
			else
				msgData.msgIndex = 0;
		} //else break;
	}
	
	msgData.msgProcessed = false;
	
	// at least COMM and COMM_* have been received
	if (msgData.msgIndex >= 2)
	{
		cmd = msgData.msgBuffer[1]; // second byte is COMM_*
		
		// check for invalid cmds or too few data in msgData.msgBuffer
		if (cmd <= COMM_NONE || cmd >= COMM_TOTAL)
	    {
		    msgData.msgIndex = 0;
			cmd = COMM_NONE;
		} 
		else if (msgData.msgIndex < 2 + COMM_LENGTH[cmd])
			cmd = COMM_NONE;
		else 
			msgData.msgProcessed = true;
			
		// process command
		switch (cmd)
		{
			case COMM_RECHARGE:
			rechargeBatteries(msgData.msgBuffer[2] + 256 * msgData.msgBuffer[3]);
			break;
		
			case COMM_RFC:
			// TODO: read from console
			break;
			
			case COMM_WTC:
			// TODO: write to console
			break;
			
			case COMM_PARAM:
			setInterpolationData(ipoData,  robotData,msgData.msgBuffer[2], &msgData.msgBuffer[3], 2);
			break;
			
			case COMM_RFAAX:	
			// conversion variables
			unsigned int tmp, bit, start, value;
			byte low, high;
			
			// zero out sendBuffer - necessary, f.e. sendBuffer[i] |= tmp
			for (i=0;i<CRC_LEN + AX12_DATA_READ_TOTAL;i++)
				sendBuffer[i] = 0;
			
			// no failure? -> pack data (bitwise, one ax12 data item is only 10bit long)
			if (failure == 0)  
			for (i=0;i<AX12_COUNT;i++)
			{
				for (j=0; j<AX12_DATA_READ/2; j++)
				{
					// calculate bit number
					bit = i * 10 * AX12_DATA_READ/2 + j * 10;
					
					// calc start and end byte
					start = bit / 8;
					
					// read low and high byte
					low  = robotData.readBuffer[i*AX12_DATA_READ + j*2]; 
					high = robotData.readBuffer[i*AX12_DATA_READ + j*2 + 1];
					
					// 10 bits are in wrong order value = 11111111 11 = low high  
					tmp = low;
					tmp = tmp << 2;
					value = tmp + (high & 0x3);  
					
					/* 10 bits will always span exactly two bytes:
					   1|01001110|0 wont happen (startindex % 2 = 0)
					   
					   example:
						 byte      			    [i]      [i+1]
						 bits   		  		10101110 11011010				 
						 bit % 8 = 2    -> data=  101110 1101 
						 -> offset = 6
						 -> mask = 00111111   ->  101110
						 -> shift left 4	  ->  101110 0000
						 -> mask = 11110000   ->         11010000
						 -> shift right 4     ->         00001101
						 --------------------------------------
						 -> add			  ->  101110 1101	*/
					
					// invert here:
					byte mask;
					byte offset;
					offset = (bit % 8);
					mask = 0x00FF >> offset;
					offset = 8 - offset;
						
					tmp = (value >> (10 - offset)) & mask;
					sendBuffer[start] |= tmp;
						
					offset = (8 - (10 - offset));
					mask = 0x00FF << offset;
					tmp = (value << offset) & mask; 
						
					sendBuffer[start + 1] |= tmp;     
				}
			}
			
			// calc crc
			crc = calcCrc(sendBuffer, AX12_DATA_READ_TOTAL);
			
			//failure -> invalidate packet
			if (failure == AX12_ERROR)
				crc = crc + 1;
			
			// append crc to buffer
			sendBuffer[AX12_DATA_READ_TOTAL + 0] = (crc & 0x00FF);
			sendBuffer[AX12_DATA_READ_TOTAL + 1] = (crc & 0xFF00) >> 8;
			
			// write to serial
			writeData((byte*)sendBuffer, CRC_LEN + AX12_DATA_READ_TOTAL);
			break;
			
			case COMM_WTAAX:
			//write to ALL ax 
			failure = AX12_NOERROR;
			
			// calc checksum and compare with supplied one -> signal failure on mismatch
			crc = calcCrc(&msgData.msgBuffer[2], AX12_DATA_WRITE_TOTAL); 
			if (crc != (msgData.msgBuffer[2 + AX12_DATA_WRITE_TOTAL + 0] + 256U*msgData.msgBuffer[2 + AX12_DATA_WRITE_TOTAL + 1]))
				failure = AX12_ERROR;
			
			//if (robotData.writeBufferLength >= WRITEBUFFERMAX)
				//failure = AX12_ERROR;
			
			// confirmation byte -> PC knows when writing has finished
			sendBuffer[0] = (failure == AX12_NOERROR) ? COMM_WTAAX : COMM_NONE; 
			writeData((byte*)sendBuffer, 1);
			
			// status information
			sendStatusInformation(sendBuffer, robotData, ipoData, sCurveParams);
			
			if (failure == AX12_NOERROR)
			{
				// dont add if writebuffer full and dont override item before last item
				if (robotData.writeBufferLength >= WRITEBUFFERMAX-1)
					break;
				
				// conversion variables
				int bit, start, end, value,i,j,tmp;
				byte low, high, index;
				
				index = (robotData.writeBufferIndex + robotData.writeBufferLength) % WRITEBUFFERMAX;
				
				robotData.writeBufferLength++;
				
				// prepare write buffer -> unpack data from serial and store bytewise in buffer
				for (i=0;i<AX12_COUNT;i++)
				{
					for (j=0; j<AX12_DATA_WRITE/2; j++)
					{
						// calculate bit number
						bit = i * AX12_DATA_WRITE/2 * 10 + j * 10;
					
					
						// calc start and end byte
						start = bit / 8;
						end = (bit + 9) / 8;
						
						// example:
						// byte      			  [i]     [i+1]
						// bits   		  		10101110 11011010				 
						// bit % 8 = 2 -> data=   101110 1101 
						// -> offset = 6
						// -> mask = 00111111 ->  101110
						// -> shift left 4	  ->  101110 0000
						// -> mask = 11110000 ->         11010000
						// -> shift right 4   ->         00001101
						// --------------------------------------
						// -> add			  ->  101110 1101	
						
						byte mask;
						byte offset;
						offset = (bit % 8);
						mask = 0x00FF >> offset;
						offset = 8 - offset;
							
						tmp = (msgData.msgBuffer[2 + start]) & mask;
						value = (tmp & 0xFF) << (10 - offset);
				  
						offset = (8 - (10 - offset));
						mask = 0x00FF << offset;
						tmp = (msgData.msgBuffer[2 + start + 1]) & mask;
						value += (tmp & 0xFF) >> (offset);  
						
						// 10 bits are in wrong order 11111111 11 = low high    
						tmp = (value & 0x3);
						high = (byte)tmp;
						tmp = (value - tmp) >> 2;
						low = (byte) tmp;
						
						// store in write buffer
						robotData.writeBuffer[index][i * (AX12_DATA_WRITE) + 2*j] = low; 
						robotData.writeBuffer[index][i * (AX12_DATA_WRITE) + 2*j + 1] = high;
					}
				}
				
				switch (ipoData.ipoType)
				{
				    case 2:
					writePositionData(robotData.readBuffer, &robotData.writeBuffer[index][0], 1, 1);
					robotData.writeBufferLength = 0;
					break;
				}
			}
			break;
			
			case COMM_RFAX:
			// read from ax
			
			// forward command from pc to ax12
			// read ax12 id, addr and data length
			id   = msgData.msgBuffer[2]+AX12_STARTID-1;
			addr = msgData.msgBuffer[3];
			len  = msgData.msgBuffer[4];
			
			gbpParameter[0] = addr; //Address
			gbpParameter[1] = len;  //Read Length
			
			// send packet
			TxPacket(id,INST_READ,2);
			
			// read answer 
			if(RxPacket(DEFAULT_RETURN_PACKET_SIZE+gbpParameter[1]) == DEFAULT_RETURN_PACKET_SIZE+gbpParameter[1])
				failure = AX12_NOERROR;
			else
				failure = AX12_ERROR;
			
			// append crc
			// calc crc
			crc = calcCrc(&gbpRxBuffer[5], len);
			
			//failure -> invalidate packet
			if (failure == AX12_ERROR)
				crc = crc + 1;
			
			// insert crc into buffer
			gbpRxBuffer[ 5 + len ] = (crc & 0x00FF);
			gbpRxBuffer[ 5 + len + 1 ] = (crc & 0xFF00) >> 8;
			
			// send data
			writeData(&gbpRxBuffer[5], len + CRC_LEN); // 2 bytes crc
			break;
			
			case COMM_WTAX:
			// write to ax
			
			// forward command from pc to ax12
			// read ax12 id, addr and data length
			id   = msgData.msgBuffer[2]+AX12_STARTID-1;
			addr = msgData.msgBuffer[3];
			len  = msgData.msgBuffer[4];
			
			if (msgData.msgIndex < 6 + len + CRC_LEN)
			{
				msgData.msgProcessed = false;
				break;
			}	
			// schedule command?
			schedule = msgData.msgBuffer[5];
			
			// calc crc
			crc = calcCrc(&msgData.msgBuffer[2], 4 + len); 
			if (crc != (msgData.msgBuffer[6 + len] + 256U*msgData.msgBuffer[6 + len + 1]))
				failure = AX12_ERROR;
			else
				failure = AX12_NOERROR;
				
			if (failure == AX12_NOERROR)
			{
				for (i=0; i<len; i++)
					gbpParameter[1 + i] = msgData.msgBuffer[6 + i];
				
				if (schedule)
					op = INST_REG_WRITE;
				else
					op = INST_WRITE;
					
				gbpParameter[0] = addr; //address
				
				// send data to ax12
				TxPacket(id,op,1+len);
				RxPacket(DEFAULT_RETURN_PACKET_SIZE);
			}
			
			// confirmation byte -> PC knows when writing has finished
			sendBuffer[0] = (failure == AX12_NOERROR) ? COMM_WTAX : COMM_NONE; 
			writeData((byte*)sendBuffer, 1);
			break;
			
			case COMM_TIME:
			// send robot time to pc
			
			// get tickcount
			time = gettickcount();
			
			// store as 4 bytes
			for (id=0; id<4; id++)
			{
				time_tmp = time % 0xFF;
				sendBuffer[id] = (byte) time_tmp;
				time /= 0xFF;
			}
			
			// send to pc
			writeData((byte*)sendBuffer, 4);
			break;
			
			case COMM_RBS:
			//read button state
			id = msgData.msgBuffer[2];
			
			// todo
			uart1_putc(0);
			break;
			
			case COMM_RLS:
			//read led state
			id = msgData.msgBuffer[2];
			
			// todo
			uart1_putc(0);
			break;
			
			case COMM_WLS:
			//write led state
			id = msgData.msgBuffer[2];
			op = msgData.msgBuffer[3];
			
			//todo
			break;
			
			case COMM_RSAX:
			//run scheduled ax
			TxPacket(BROADCASTING_ID,INST_ACTION,0);
			break;
			
		}
	}
	
	if (msgData.msgProcessed)	
		msgData.msgIndex = 0;
  }
}


