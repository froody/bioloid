/* 
 * University of Plymouth Bioloid Robot Controller
 * for Atmel Mega 128
 * FIRA England Robot Football Team
 * - Based on Dynamixel Evaluation Example (Original Author : Byoung Soo Kim)
 * - Innitial support for executing Motion Editor Pages
 * Author : Joerg Wolf, joerg.wolf -xaxtx- plymouth.ac.uk
 * 
 * Version 0.11
 */
#include <avr/io.h>
#include <avr/interrupt.h>
#include <string.h>
#include <stdio.h>

#include "motion.h"
#include "robot.h"
#include "comms.h"
#include "BLC-Protocol.h"
#include "led.h"
#include "buggy.h"



extern volatile unsigned char TxBuff[MAXBUF_TX], RxBuff[MAXBUF_RX],Packet[MAXBUF_RX];       // Transmit and Receive buffers
extern volatile unsigned char *RxPtr, *TxPtr ;  					 // Pointers to buffer positions
extern volatile unsigned int TxBytes; 
extern volatile unsigned int ROBOTSTATUS;
extern volatile int ServoTrim[NUM_OF_SERVOS_ATTACHED];

extern uint16_t doPose(int page,char pose);
extern uint16_t doPage(int page);

char CheckProtocol(void);					// Check the message for correctness
void OnGoodPacket(void);					// Process message
void OnErr(int ErrNo);
void CommsSend(unsigned char *Data, int TxLen, unsigned char Block);

void CommsSend2Bytes(int l,int r);


void CommsSendInt(int Int );
int UnsignedCharToChar(unsigned char uchar);
uint16_t crc_compute(uint8_t *buf, uint16_t size);

void CommsSend2Bytes(int l,int r)
{

	unsigned char Data[4];
	Data[0]='[';
	Data[1]=(unsigned char)l;
	Data[2]=(unsigned char)r;
	Data[3]=']';
	CommsSend(Data,4,NO_BLOCK);
}


void CommsSendInt(int Int )
{
	uint16_t lowerbyte,higherbyte;
	unsigned char Data[4];
	lowerbyte = Int & 0x00FF;
	higherbyte = Int;
	higherbyte = higherbyte >> 8;
	Data[0]='[';
	Data[1]=(unsigned char)higherbyte;
	Data[2]=(unsigned char)lowerbyte;
	Data[3]=']';
	CommsSend(Data,4,NO_BLOCK);
}

/************************************************************************\
*   Routine to send debugging information back to the host
* We need to be able to specify whether this routine should or should not
* block
\************************************************************************/
void CommsSend(unsigned char *Data, int TxLen, unsigned char Block)
{
    
	if(TxLen>MAXBUF_TX){ return;} // reject huge packets
	int CurrentEndofMessage = (TxPtr-TxBuff + TxBytes);
	int SpaceLeft = MAXBUF_TX - CurrentEndofMessage;
	switch (Block)
    {
        case NO_BLOCK:
            if (TxBytes>0){return;}
            TxPtr=TxBuff;
            break;
        case MRG_BLOCK:
			//while(TxBytes+TxLen > MAXBUF_TX); //wait until there is enough space in the buffer
            //if (TxBytes+TxLen > MAXBUF_TX){ return; }// reject if packet doesn't fit into buffer
            if(SpaceLeft < TxLen){return;}
			break;
        case FUL_BLOCK:
        default:
            while (TxBytes > 0);                    // Wait for the transmit buffer to clear -- good idea?
            if (TxLen > MAXBUF_TX) TxLen = MAXBUF_TX;             // Try to avoid overflows
            TxPtr=TxBuff;
            break;
    }

    // Copy characters into Transmit buffer
	
    memcpy((void *)TxPtr+TxBytes, Data, TxLen);
    cli();
	TxBytes += TxLen;                           // Set length of buffer to transmit
	sei();
    if (TxBytes > 0)  // Make sure we have something to send
    {
        cli();
			Set_LED(LED_TXD, LED_ON);
			UDR1 = *TxPtr;	// Send character to UART
            TxBytes--;
            TxPtr++;	
        sei();
    }
}

char CheckProtocol(void){
	// Check the message for correctness
	if(Packet[0]!='['){return -1;}
	if(Packet[MAXBUF_RX-1]!=']'){return -1;}
	uint16_t lowerbyte = Packet[MAXBUF_RX-1-1];
	uint16_t higherbyte = Packet[MAXBUF_RX-1-2];
	uint16_t rec_crc = (higherbyte << 8) + lowerbyte;
	uint16_t crc = crc_compute((uint8_t*)&Packet[1], MAXBUF_RX-4); // packet is 40 bytes including brackets. without brackets and checksum the length is MAX_BUF_RX-4
	if(crc!=rec_crc){return -1;}
	
	return 0;
}



/************************************************************************\
*   Routine to process the received good data
\************************************************************************/

void OnGoodPacket()
{
	if(Packet[1]==DO_PAGE){
		ROBOTSTATUS = DO_PAGE;
		doPage(Packet[2]);
		//printf("[Doing]");
	}
	if(Packet[1]==DO_POSE){
		ROBOTSTATUS = DO_POSE;
		doPose(Packet[2],Packet[3]);
	}
	if(Packet[1]==STOP_POSE){
		ROBOTSTATUS = STOP_POSE;
		printf("[NotImpl]");
	}
	if(Packet[1]==STOP_PAGE){
		ROBOTSTATUS = STOP_PAGE;
		printf("[NotImpl]");
	}
	if(Packet[1]==BUGGY_MOVE){
		buggy_move(Packet[2]);
	}
	if(Packet[1]==TURN_SERVO){	
		uint16_t HBP = Packet[3];
		int16_t ThetaTarget = (HBP<<8) + Packet[4];
		uint16_t HBS = Packet[5];
		int16_t OmegaServo = (HBS<<8) + Packet[6];
		SendServoTargetPos(Packet[2],ThetaTarget,OmegaServo);
	}
	if(Packet[1]==SERVO_TRIM){
		if(Packet[2] < NUM_OF_SERVOS_ATTACHED){
			ServoTrim[Packet[2]]=UnsignedCharToChar(Packet[3]);
			printf("\r\n[trim(%i)=%i ]\r\n",Packet[2],ServoTrim[Packet[2]]);
		}
	}
	if(Packet[1]==BALANCE){
		if(Packet[2] == 0){
			Balance = BALANCE_OFF;
		}else{
			Balance = BALANCE_ON;
		}
	}
	if(Packet[1]==VERSION){
		printf("[BLV1.11]");
	}
	
}

/**************************************************************************\
*   Process the bad messages
*   (ignore but let user know by toggling the bad packet LED)
\**************************************************************************/

void OnErr(int ErrNo)
{
    //CommsSend("EH?",3,DBG_BLOCK);
    //Set_LED(LED_RED, LED_TOGGLE);                // Toggle the bad packet LED
}

/**************************************************************************\
*   Convert received characters into a speed value
\**************************************************************************/

int UnsignedCharToChar(unsigned char uchar)
{
    int RetVal = (int)(uchar & 0x7F);
    if (uchar & 0x80)
        RetVal *= -1;
    return RetVal;
}



uint16_t crc_compute(uint8_t *buf, uint16_t size)
{
   uint16_t index, crc;
   uint8_t v, xor_flag, byte, bit;
   
   crc = INITIAL_VALUE;

   for(index = 0; index < size; index++) {
      byte = buf[index];
      /*
	Align test bit with leftmost bit of the message byte.
      */
      v = 0x80;

      for(bit = 0; bit < 8; bit++) {
	 if(crc & 0x8000)
            xor_flag= 1;
	 else
            xor_flag= 0;

	 crc = crc << 1;
            
	 /*  Append next bit of message to end of CRC if it is not zero.
	     The zero bit placed there by the shift above need not be
	     changed if the next bit of the message is zero. */
	 if(byte & v)
	    crc= crc + 1;

	 if(xor_flag)
	    crc = crc ^ POLY;

	 /* Align test bit with next bit of the message byte. */
	 v = v >> 1;
      }
   }

   /* We have to augment the crc in order to comply with the ccitt spec. */

    for(bit = 0; bit < 16; bit++) {
        if(crc & 0x8000)
            xor_flag= 1;
        else
            xor_flag= 0;

	crc = crc << 1;

        if(xor_flag)
            crc = crc ^ POLY;
    }

    return crc;
}

