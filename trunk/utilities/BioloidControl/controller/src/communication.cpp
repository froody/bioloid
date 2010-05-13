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
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/signal.h>

#include "..\include\constants.h"
#include "..\include\types.h"
#include "..\include\communication.h"
#include "..\include\uart.h"
#include ".\include\interpolation.h"

// --- Global Variable Number ---
volatile byte gbpRxInterruptBuffer[256]; 
byte gbpParameter[128];
byte gbRxBufferReadPointer;
byte gbpRxBuffer[128]; 
byte gbpTxBuffer[128]; 
volatile byte gbRxBufferWritePointer;

// read len bytes from serial
void readData(byte *bData, int len)
{
	while (len-- > 0)
	{
		*bData = uart1_getc();
		bData++;
	}
}

// write len bytes to serial
void writeData(byte *bData, int len)
{
	while (len-- > 0)
	{
		uart1_putc(*bData++);
	}
}

void initMessageData(struct MessageData &msgData)
{
	msgData.msgIndex = 0;
	msgData.msgProcessed = false;
}

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

// init serial comm
void PortInitialize(void)
{
  DDRA = DDRB = DDRC = DDRD = DDRE = DDRF = 0;  //Set all port to input direction first.
  PORTB = PORTC = PORTD = PORTE = PORTF = PORTG = 0x00; //PortData initialize to 0
  cbi(SFIOR,2); //All Port Pull Up ready
  DDRE |= (BIT_RS485_DIRECTION0|BIT_RS485_DIRECTION1); //set output the bit RS485direction

  DDRD |= (BIT_ZIGBEE_RESET|BIT_ENABLE_RXD_LINK_PC|BIT_ENABLE_RXD_LINK_ZIGBEE);
  
  PORTD &= ~_BV(BIT_LINK_PLUGIN); // no pull up
  PORTD |= _BV(BIT_ZIGBEE_RESET);
  PORTD |= _BV(BIT_ENABLE_RXD_LINK_PC);
  PORTD |= _BV(BIT_ENABLE_RXD_LINK_ZIGBEE);
}

/*
TxPacket() send data to RS485.
TxPacket() needs 3 parameter; ID of Dynamixel, Instruction byte, Length of parameters.
TxPacket() return length of Return packet from Dynamixel.
*/
byte TxPacket(byte bID, byte bInstruction, byte bParameterLength) 
{
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

    RS485_TXD;
    for(bCount = 0; bCount < bPacketLength; bCount++)
    {
        sbi(UCSR0A,6);//SET_TXD0_FINISH;
        TxD80(gbpTxBuffer[bCount]);
    }
    
	while(!CHECK_TXD0_FINISH) //Wait until TXD Shift register empty
		;// nothing
	
	RS485_RXD;
    
	return(bPacketLength);
}

/*
RxPacket() read data from buffer.
RxPacket() need a Parameter; Total length of Return Packet.
RxPacket() return Length of Return Packet.
*/

byte RxPacket(byte bRxPacketLength)
{
#define RX_TIMEOUT_COUNT2   3000L  
#define RX_TIMEOUT_COUNT1  (RX_TIMEOUT_COUNT2*10L)  
  unsigned long ulCounter;
  byte bCount, bLength, bChecksum;
  byte bTimeout;
  
  bTimeout = 0;
  for(bCount = 0; bCount < bRxPacketLength; bCount++)
  {
    ulCounter = 0;
    
	while(gbRxBufferReadPointer == gbRxBufferWritePointer)
    {
      if(ulCounter++ > RX_TIMEOUT_COUNT1)
      {
        bTimeout = 1;
        break;
      }
    }
    
	if(bTimeout) 
		break;
    
	gbpRxBuffer[bCount] = gbpRxInterruptBuffer[gbRxBufferReadPointer++];
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

/*Hardware Dependent Item*/
#define TXD1_READY			bit_is_set(UCSR1A,5) //(UCSR1A_Bit5)
#define TXD1_DATA			(UDR1)
#define RXD1_READY			bit_is_set(UCSR1A,7)
#define RXD1_DATA			(UDR1)

#define TXD0_READY			bit_is_set(UCSR0A,5)
#define TXD0_DATA			(UDR0)
#define RXD0_READY			bit_is_set(UCSR0A,7)
#define RXD0_DATA			(UDR0)

/*
SerialInitialize() set Serial Port to initial state.
Vide Mega128 Data sheet about Setting bit of register.
SerialInitialize() needs port, Baud rate, Interrupt value.
*/
void SerialInitialize(byte bPort, byte bBaudrate, byte bInterrupt)
{
  if(bPort == SERIAL_PORT0)
  {
    UBRR0H = 0; UBRR0L = bBaudrate; 
    UCSR0A = 0x02;  UCSR0B = 0x18;
	
    if(bInterrupt & RX_INTERRUPT) 
		sbi(UCSR0B,7); // RxD interrupt enable
		
    UCSR0C = 0x06; UDR0 = 0xFF;
    sbi(UCSR0A,6);//SET_TXD0_FINISH; // Note. set 1, then 0 is read
  }
  else if(bPort == SERIAL_PORT1)
  {
    UBRR1H = 0; UBRR1L = bBaudrate; 
    UCSR1A = 0x02;  UCSR1B = 0x18;
	
    if(bInterrupt & RX_INTERRUPT)
		sbi(UCSR1B,7); // RxD interrupt enable
		
	if(bInterrupt & TX_INTERRUPT)
		sbi(UCSR1B,6); // RxD interrupt enable
		
	UCSR1C = 0x06; UDR1 = 0xFF;
    sbi(UCSR1A,6);//SET_TXD1_FINISH; // Note. set 1, then 0 is read
  }
}

/*
TxD80() send data to USART 0.
*/
void TxD80(byte bTxdData)
{
  while(!TXD0_READY);
	TXD0_DATA = bTxdData;
}

/*
TXD81() send data to USART 1.
*/
void TxD81(byte bTxdData)
{
  while(!TXD1_READY);
	TXD1_DATA = bTxdData;
}

/*
RxD81() read data from UART1.
RxD81() return Read data.
*/
byte RxD81(void)
{
  while(!RXD1_READY);
	return(RXD1_DATA);
}

byte RxD81Async(void)
{
	return(RXD1_DATA);
}

/*
SIGNAL() UART0 Rx Interrupt - write data to buffer
*/
SIGNAL(SIG_UART0_RECV)
{
  gbpRxInterruptBuffer[(gbRxBufferWritePointer++)] = RXD0_DATA;
}


