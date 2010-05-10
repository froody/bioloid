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
 
#define ENABLE_BIT_DEFINITIONS 
#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include "robot.h"
#include "bioloid.h"
#include "dynamixel-comms.h"
#include "dynamixel.h"
#include <stdio.h>

uint8_t PingDevice(uint8_t id){
	dxl_ping(id);

	if(dxl_get_result() == COMM_RXSUCCESS) {
		return 1;
	} else {
		printf("E:Device %d not connected\n\r",id);
		return 0;
	}
}


