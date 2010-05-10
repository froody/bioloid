
#define NUM_OF_SERVOS_ATTACHED		19

//RS232 baudrate:  ((16000000L/8L)/((long)UBRR1L+1L) ) 
//RS485 baudrate:  ((16000000L/8L)/((long)UBRR0L+1L) )

#define DEFAULT_BAUD_RATE 34   //57600bps at 16MHz  

#define DISABLE_DYNAMIXEL_WARNINGS
#define DISABLE_SERVO_MISSING_WARNINGS

/* SYSTEM CONSTANTS*/
//TCNT0 = 256 - (16MHz / (1024 * 600Hz)) = 256 - 26.041 ˜ 230
    // 178 for 200Hz at 16 MHz w/ prescale 1024
    // 194 for 250Hz at 16 MHz w/ prescale 1024
    // 204 for 300Hz
    // 217 for 400Hz at 16 MHz w/ prescale 1024
    // 230 for 600Hz at 16 MHz w/ prescale 1024
	 //134 for 128Hz (1 Unit Pause in Motion Editor) at 16 MHz w/ prescale 1024

#define TMRRESET    134


// Define RobotFlags (bit address) Here:
#define HBEAT       0
#define MSGRCV      1


#define TRUE		1
#define FALSE		0

// Define ROBOTSTATUS
#define RS_GO			0x0001
#define RS_STOP_NOW		0x0002
#define RS_STOP_POSE	0x0004
#define RS_STOP_PAGE	0x0008

// Bob Board
//BOB_BOARD_STATUS
#define DEV20_CONNECTED	0x01
#define DEV21_CONNECTED	0x02
#define DEV22_CONNECTED	0x04
#define I_HAVE_FALLEN_OVER 0x08
