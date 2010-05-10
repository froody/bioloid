/** 
 * University of Plymouth Bioloid Robot Controller
 * for Atmel Mega 128
 * FIRA England Robot Football Team
 * - Based on Dynamixel Evaluation Example (Original Author : Byoung Soo Kim)
 * - Initial support for executing Motion Editor Pages
 * Author : Joerg Wolf, joerg.wolf -xaxtx- plymouth.ac.uk
 * 
 * Version 0.12
 */

#define ENABLE_BIT_DEFINITIONS 
#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include "bioloid.h"
#include "robot.h"
#include "motion.h"
#include "comms.h"
#include "BLC-Protocol.h"
#include "led.h"
#include "robot-dash.h"
#include "dynamixel-comms.h"
#include "buggy.h"
#include <stdio.h>

#include <math.h>

#include "serial.h"
#include "dynamixel.h"
#include "zigbee.h"
#include <util/delay.h>

//#define DEBUG_MODE
//
#define DEFAULT_BAUDNUM 1


void OnGoodPacket(void);


// ---------- extern functions from motion.c --------------
extern void ContinueMotion_HeartBeat(void);
extern void MotionInitialise(void);
extern uint16_t doPose(int page,char pose);
extern uint16_t doPage(int page);

// ----------- extern functions from led.c ---------------
extern void LED_Init(void);
extern void Set_LED(unsigned char LEDno, unsigned char state);

// ------------ Comms Global Variables -------------------

// ------------ Function Prototypes ----------------------
void PortInitialize(void);
void Behaviour_HeartBeat(void);
void Test(void);
void Robot_Dash(void);
void EnableButtons(void);

// ------------ Global Variables -------------------------
volatile byte gbRxBufferWritePointer;
/* RobotFlags is used to indicate interrupts to the main program */
volatile unsigned char RobotFlags;

volatile unsigned char TxBuff[MAXBUF_TX], RxBuff[MAXBUF_RX],Packet[MAXBUF_RX];       // Transmit and Receive buffers
volatile unsigned char *RxPtr, *TxPtr ;  					 // Pointers to buffer positions
volatile unsigned int TxBytes; 
volatile unsigned int ROBOTSTATUS;
volatile unsigned char BOB_BOARD_STATUS;
volatile unsigned char BUTTON_STATUS;
#ifdef ROBOT_DASH_MODE
volatile int Forward_Dash_StepNo;
volatile int Backward_Dash_StepNo;
#endif

// ------------- Interrupts ------------------------------

//SIGNAL (SIG_UART0_RECV) in bioloid-comms.c


/**
	Interrupt for START button (INT0)
*/
SIGNAL(SIG_INTERRUPT0)
{
	if( BUTTON_STATUS & (1 << 2)){
		BUTTON_STATUS = 0;
	}else{
		BUTTON_STATUS |= (1 << 0);
	}
}

/**
	Interrupt for UP button   (INT4)
*/
SIGNAL(SIG_INTERRUPT4)
{
	BUTTON_STATUS |= (1 << 4); 
}
/**
	Interrupt for DOWN button (INT5)
*/
SIGNAL(SIG_INTERRUPT5)
{
	BUTTON_STATUS |= (1 << 5); 
}

/**
	Interrupt for LEFT button (INT6)
*/
SIGNAL(SIG_INTERRUPT6)
{
	BUTTON_STATUS |= (1 << 6); 
}

/**
	Interrupt for RIGHT button (INT7)
*/
SIGNAL(SIG_INTERRUPT7)
{
	BUTTON_STATUS |= (1 << 7); 
}

#if 0
/**
	This interrupt goes off when a byte from the TxBuffer was transmitted.
	If the end of the message has not been reached (TxBytes>0) then the next byte will be transmitted
	to the Pc or PDA
*/
SIGNAL (SIG_UART1_TRANS)
{
	if(TxBytes>0){
		cli();
		UDR1 = *TxPtr;
		TxPtr++;
		TxBytes--;
		if(TxBytes==0){
			TxPtr=TxBuff;
			Set_LED(LED_TXD, LED_OFF);
		}else{Set_LED(LED_TXD, LED_ON);}
		sei();
	}
}

/**
	This interrupt goes off when a byte from the serial port arrives
	the function controls the collection of a data-paket with "[" and "]"
*/
SIGNAL (SIG_UART1_RECV)
{
	unsigned char WRK_REG = UDR1;
	//Set_LED(LED_RXD, LED_ON);
	if(WRK_REG=='['){
		RxPtr = &RxBuff[0];
		*RxPtr='[';	
	}else if(WRK_REG ==']'){
		*RxPtr=WRK_REG;
		
		if(RxPtr >= &RxBuff[MAXBUF_RX-1]){		// check packet length
			memcpy((void *)Packet,RxBuff,MAXBUF_RX);
			RobotFlags |= _BV(MSGRCV);
		}else{
			// bracket closed inside the packet or error
		}
	}else{
		*RxPtr=WRK_REG ;
	}
	RxPtr++;
	if(RxPtr >= &RxBuff[MAXBUF_RX]){
		RxPtr = &RxBuff[0];		// prevent overflow
	}
	//Set_LED(LED_RXD, LED_OFF);
}
#endif


unsigned char get_device_data(int ServoNo, unsigned char adr){	
	return dxl_read_byte(ServoNo, adr);
}
volatile unsigned long timer0_overflow_count = 0;
volatile unsigned long timer0_clock_cycles = 0;
volatile unsigned long timer0_millis = 0;

#define clockCyclesPerMicrosecond() ( F_CPU / 1000000L )
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

SIGNAL(TIMER0_OVF_vect)
{
	timer0_overflow_count++;
	// timer 0 prescale factor is 64 and the timer overflows at 256
	timer0_clock_cycles += 64UL * 256UL;
	while (timer0_clock_cycles > clockCyclesPerMicrosecond() * 1000UL) {
		timer0_clock_cycles -= clockCyclesPerMicrosecond() * 1000UL;
		timer0_millis++;
	}

	RobotFlags |= _BV(HBEAT);
}

static unsigned long millis(void)
{
	unsigned long m;
	uint8_t oldSREG = SREG;
	
	// disable interrupts while we read timer0_millis or we might get an
	// inconsistent value (e.g. in the middle of the timer0_millis++)
	cli();
	m = timer0_millis;
	SREG = oldSREG;
	
	return m;
}

void delay(unsigned long ms);
void delay(unsigned long ms)
{
	unsigned long start = millis();
	
	while (millis() - start <= ms)
		;
}


void
timer_init(void)
{
	// on the ATmega168, timer 0 is also used for fast hardware pwm
	// (using phase-correct PWM would mean that timer 0 overflowed half as often
	// resulting in different millis() behavior on the ATmega8 and ATmega168)
	sbi(TCCR0A, WGM01);
	sbi(TCCR0A, WGM00);

	// set timer 0 prescale factor to 64
	sbi(TCCR0B, CS01);
	sbi(TCCR0B, CS00);

	// enable timer 0 overflow interrupt
	sbi(TIMSK0, TOIE0);
}

SIGNAL(TIMER1_OVF_vect)
{
}

int pwm_values[] = {0x40, 0x100, 0x180, 0x200, 0x280, 0x300, 0x380, 0x400 } ;
int cur_pwm = 0;

#define PULSE_WIDTH 0x40
void
start_pwm(void)
{
	cli();
#if 1
   TCCR1A = _BV(WGM10) | _BV(WGM11);   //Timer 1 is Phase-correct 10-bit PWM.
   TCCR1A |= _BV(COM1A1);            //Clear OC1A on compare match when up-counting, set OC1A on compare match when down-counting.

   TCCR1B = _BV(CS11);               // full XTAL, no prescalar

   // Set PWM value to 0. 
   //OCR1A = pwm_values[cur_pwm]/10;
   OCR1A = 0x33;
   cur_pwm++;
   cur_pwm %= sizeof(pwm_values)/sizeof(*pwm_values);

   // Enable OC1 (PB5 on m128) as output. 
   DDRB = _BV (PB5);   //0x20

   // Enable timer 1 overflow interrupt. 
   TIMSK1 = 0x1;

   //enable global interrupts
   sei ();
	
#else
	OCR1AL = PULSE_WIDTH;   //Load Pulse width
	OCR1AH = 0;
	DDRB |= (1<<5);         //PortD.5 as o/p
	TCCR1A = 0x81;          //8-bit, Non-Inverted PWM
	TCCR1B = 0x10;             //Start PWM
	sbi(TIMSK1, TOIE1);
#endif
	sei();

}

void
stop_pwm(void)
{
	TCCR1B = 0;
}

void
beep(void)
{
	start_pwm();
	delay(250);
	stop_pwm();
}

#define UNITS_PER_MS 0.6721
void
test_math(void)
{
	uint16_t longestDistance = 900;
	printf("longest time %d\n", (int) ((double)longestDistance/(double)UNITS_PER_MS));

	uint16_t dist;
	for(dist = 50;dist < 900;dist += 50) {

		double ratio = (double)dist/(double)longestDistance;

		ratio *= (double)505.67300322796609L;

		printf("dist = %d, speed = %d\n", dist, (int)ratio);
	}
}

int main(void)
{ 
	BOB_BOARD_STATUS = 0;
	ROBOTSTATUS = STOP_POSE;
	Balance = BALANCE_OFF;

	serial_initialize(57600);

	BUTTON_STATUS = (1 << 2);

	/* Timer0 is for the HeartBeat */
	RobotFlags = 0;

	timer_init();
	dxl_initialize( 0, DEFAULT_BAUDNUM ); // Not using device index

	LED_Init();
    Set_LED(LED_POWER,LED_ON);
	Set_LED(LED_MANAGE,LED_ON);
	
	sei();  //Enable Interrupt -- Compiler Function

	//InitGyro();

	//zgb_initialize(0);

	/**
		Wait 0.5 seconds and then switch off the MANAGE LED
		This indicates that a reset has occured
	*/
	int pause=0;
	do{
		if (RobotFlags & _BV(HBEAT)){
			pause++;
			RobotFlags=RobotFlags & ~_BV(HBEAT);
		}
	}while(pause<64);
	
	#ifdef DEBUG_MODE	
	printf("\r\n----------  DEBUG MODE -----------\r\n");
	#endif
	printf("\r\n[pg00]\r\n[University of Plymouth Bioloid Robot Controller]\r\n");
	
	if(PingDevice(20)){ BOB_BOARD_STATUS |= DEV20_CONNECTED; }
	if(PingDevice(21)){ BOB_BOARD_STATUS |= DEV21_CONNECTED; }
	if(PingDevice(22)){ BOB_BOARD_STATUS |= DEV22_CONNECTED; }

	MotionInitialise();
	EnableButtons();

	Set_LED(LED_MANAGE,LED_OFF);
	
	int page = 0;

	test_math();

#if 0 // Serial control interface
	while(1) {
		unsigned char c,d;
		int i;

		serial_read_blocking(&c, 1);
		printf("got %x\n", c);
		switch(c) {
			case 1:
				serial_read_blocking(&c, 1);
				doPage(c);
				break;
			case 2:
				serial_read_blocking(&c, 1);
				serial_read_blocking(&d, 1);
				doPose(c, d);
				break;
			case 3:
				for(i=1;i<NUM_OF_SERVOS_ATTACHED;i++) {
					printf("Servo %d: %d\n", i, getCurrentServoPos(i));
				}
				break;
			case 4:
				{
					serial_read_blocking(&c, 1);
					int pos, speed;
					serial_read_blocking((uint8_t *)&pos, 2);
					serial_read_blocking((uint8_t *)&speed, 2);

					printf("sending %d to %d @ %d\n", (int)c, pos, speed);

					SendServoTargetPos(c, pos, speed);
				}
				break;
			default:
				break;
		}

	}
#endif

	for(;;)
	{
		if (RobotFlags & _BV(MSGRCV)){
			cli();
				RobotFlags=RobotFlags & ~_BV(MSGRCV);
			sei();
			//if(CheckProtocol()==0){
				OnGoodPacket();	
			//}
		}
		if (RobotFlags & _BV(HBEAT)){
			cli();
				RobotFlags=RobotFlags & ~_BV(HBEAT);
			sei();
			delay(1000);

			uint32_t addr =  (uint32_t)POSE_BASE_ADR +(uint32_t)POSE_PAGE_SIZE*(uint32_t)page;
			if(addr >= 0x3E000)
				page = 0;
			else
				page++;

			doPage(page);

			//Behaviour_HeartBeat();
			//ContinueMotion_HeartBeat();
			//buggy_timer();
			
			#ifdef ROBOT_DASH_MODE
#error
			Robot_Dash();
			#endif
			
			#ifdef DEBUG_MODE
#error
			Test();
			#endif
			
			
		}	
	}
}


/** Low level behavior functions can go here 
*/

void Behaviour_HeartBeat(void)
{
	//MeasureGyro();
	static int tick=0;
	static int tick_tilt=0;
	tick++;
	tick_tilt++;
	
	if(tick==6){
		if((Balance)&&(MHD.CurrentPage==-1)){
			int CP[20];
			for(uint8_t ServoNo=13;ServoNo<17;ServoNo++){
				CP[ServoNo]=getCurrentServoPos(ServoNo);
			}
			for(uint8_t ServoNo=13;ServoNo<17;ServoNo++){
			//if((ServoNo==11)||(ServoNo==12)||(ServoNo==13)||(ServoNo==14)){
				int error = ServoPos[ServoNo] - CP[ServoNo];
				int ThetaTarget = ServoPos[ServoNo] + error;
					//if((abs(error)>0)||(abs(error)>0)){
					//	sprintf(txt,"he%i\r\n",error); TxDString(txt);
					//}
				SendServoTargetPos(ServoNo,ThetaTarget,400);	
			}
		}	
	}
	
	#ifndef ROBOT_DASH_MODE
	if(tick==6){
		if(BUTTON_STATUS!=0){
			for(int i=0;i<20000;i++){ i++;i--; i++;i--; i++;i--; i++;i--; i++;i--;}
			unsigned char pg=0;
			if(BUTTON_STATUS & (1 << 2)){} // system just switched on
			if(BUTTON_STATUS & (1 << 0)){ pg = 0x90; }
			if(BUTTON_STATUS & (1 << 4)){ pg = 0x91; }
			if(BUTTON_STATUS & (1 << 5)){ pg = 0x92; doPage(4); }
			if(BUTTON_STATUS & (1 << 6)){ pg = 0x93; }
			if(BUTTON_STATUS & (1 << 7)){ pg = 0x94; }
			
			if(pg!=0){
				printf("[pg%02X]",pg);
				BUTTON_STATUS = 0;
			}
		}
	}
	#endif
	
	if(tick==6)
	{
		tick=0;
	}
	
	/*
	if( BOB_BOARD_STATUS & DEV20_CONNECTED){
			tick_tilt++;
			if(BOB_BOARD_STATUS & I_HAVE_FALLEN_OVER){
				if(tick_tilt>128*14){
					tick_tilt=0;
					//PingDevice(20);
					if(get_device_data(20,P_TILT_SWITCH_FALL_FRONT) != 0){
						char txt[11]; txt[0]='\0';
						sprintf(txt,"[pg%02X]",128);	// page 128 mean fallen to front
						TxDString(txt);
						BOB_BOARD_STATUS |= I_HAVE_FALLEN_OVER;
					}else if(get_device_data(20,P_TILT_SWITCH_FALL_BACK) != 0 ){
						char txt[11]; txt[0]='\0';
						sprintf(txt,"[pg%02X]",129);	// page 129 mean fallen to back
						TxDString(txt);
						BOB_BOARD_STATUS |= I_HAVE_FALLEN_OVER;
					}else{
						char txt[11]; txt[0]='\0';
						sprintf(txt,"[pg%02X]",130);	// robot got up
						TxDString(txt);
						BOB_BOARD_STATUS &= ~I_HAVE_FALLEN_OVER;
					}
				}
			}else{
				if(tick_tilt>12){
					tick_tilt=0;
					//PingDevice(20);
					if(get_device_data(20,P_TILT_SWITCH_FALL_FRONT) != 0){
						char txt[11]; txt[0]='\0';
						sprintf(txt,"[pg%02X]",128);	// page 128 mean fallen to front
						TxDString(txt);
						BOB_BOARD_STATUS |= I_HAVE_FALLEN_OVER;;
					}else if(get_device_data(20,P_TILT_SWITCH_FALL_BACK) != 0 ){
						char txt[11]; txt[0]='\0';
						sprintf(txt,"[pg%02X]",129);	// page 129 mean fallen to back
						TxDString(txt);
						BOB_BOARD_STATUS |= I_HAVE_FALLEN_OVER;
					}
				}
			}
	}
*/

}

#ifdef ROBOT_DASH_MODE


void Robot_Dash(void){
	
	static int button_tick=0;
	static int DASH_STATE=-1;
	static int wait_tick=0;

	// Buttons:
	button_tick++;
	if(button_tick==12){
		if(BUTTON_STATUS!=0){
			for(int i=0;i<20000;i++){ i++;i--; i++;i--; i++;i--; i++;i--; i++;i--;}
			for(int i=0;i<20000;i++){ i++;i--; i++;i--; i++;i--; i++;i--; i++;i--;}
			for(int i=0;i<20000;i++){ i++;i--; i++;i--; i++;i--; i++;i--; i++;i--;}

			unsigned char pg=0;
			if(BUTTON_STATUS & (1 << 2)){} // system just switched on
			if(BUTTON_STATUS & (1 << 0)){ pg = 0x90; DASH_STATE=1; }		// start
			if(BUTTON_STATUS & (1 << 4)){ pg = 0x91; 
				ServoTrim[11]-=6;
				ServoTrim[12]+=6;
				doPage(1);
			}					// up
			if(BUTTON_STATUS & (1 << 5)){ pg = 0x92; 
				ServoTrim[11]+=6;
				ServoTrim[12]-=6;
				doPage(1);
			}					// down
			if(BUTTON_STATUS & (1 << 6)){ pg = 0x93; 
				ServoTrim[15]-=6;
				ServoTrim[16]+=6;
				doPage(1);
			}					// left
			if(BUTTON_STATUS & (1 << 7)){ pg = 0x94; 
				ServoTrim[15]+=6;
				ServoTrim[16]-=6;
				doPage(1);
			}					// right
			
			if(pg!=0){
				printf("[pg%02X]\n",pg);
				BUTTON_STATUS = 0;
			}
		}
		button_tick=0;
	}
	
	
	// state -1: boot up, robot stand straight
	if(DASH_STATE==-1){
		ROBOTSTATUS = DO_PAGE;
		doPage(1);
		printf("DS -1 ");
		DASH_STATE=0;
		
	}
	// state 1 :  start was pressed
	else if(DASH_STATE==1){
		printf("DS 1 ");
		Forward_Dash_StepNo=0;
		Backward_Dash_StepNo=0;
		
		doPage(FORWARD_STEP_PAGE_NO);
		DASH_STATE=2;
		printf("DS 2 ");
	}
	// state 2: robot is on its way forward
	else if(DASH_STATE==2){
	/*	if(Forward_Dash_StepNo < NO_OF_STEPS_FORWARD){
				// carry on walking forward
		}else{
			TxDString("DS 3 ");
			//doPage(FORWARD_STEP_EXIT_PAGE_NO);
			ROBOTSTATUS=STOP_PAGE;
			DASH_STATE=3;
		}*/
	}
	// state 3: robot is doing FORWARD_STEP_EXIT_PAGE_NO
	else if(DASH_STATE==3){
		if(MHD.CurrentPage!=-1){
			// robot busy
		}else{
			// robot finished , make it stand still
			ROBOTSTATUS = DO_PAGE;
			doPage(STAND_STRAIGHT_PAGE_NO);
			DASH_STATE=4;
			printf("DS 4 ");
		}
	}
	else if(DASH_STATE==4){
		wait_tick++;
		if(wait_tick>256){
			DASH_STATE=5;
			printf("DS 5 ");
			wait_tick=0;
		}
	}
	// state 4: robot is doing page 1,wait until finish then walk back
	else if(DASH_STATE==5){
		if(MHD.CurrentPage!=-1){
			// robot busy
		}else{
			// robot finished, make it go backwards
			ServoTrim[15]=+20;
			ServoTrim[16]=-20;
			ServoTrim[11]=ServoTrim[11]/2;
			ServoTrim[12]=ServoTrim[12]/2;
			doPage(BACKWARD_STEP_PAGE_NO);
			DASH_STATE=6;
			printf("DS 6 ");
		}
	}
	else if(DASH_STATE==6){
			// continue walk backward forever
	}
	
}
#endif


void EnableButtons(void)
{
  cli();
  PORTD |= ((1 << 0));
  PORTE |= ((1 << 4)|(1 << 5)|(1 << 6)|(1 << 7)); 
  EIMSK =  (1 << 0)|(1 << 4)|(1 << 5)|(1 << 6)|(1 << 7); 
  EICRA = (1<<ISC01);
  EICRB = (1<<ISC41)|(1<<ISC51)|(1<<ISC61)|(1<<ISC71);
  sei();
}

void PortInitialize(void)
{
  DDRA = DDRB = DDRC = DDRD = DDRE = DDRF = 0;  //Set all port to input direction first.
  PORTB = PORTC = PORTD = PORTE = PORTF = PORTG = 0x00; //PortData initialize to 0
//  cbi(SFIOR,2); //All Port Pull Up ready
  DDRE |= (BIT_RS485_DIRECTION0|BIT_RS485_DIRECTION1); //set output the bit RS485direction

  DDRD |= (BIT_ZIGBEE_RESET|BIT_ENABLE_RXD_LINK_PC|BIT_ENABLE_RXD_LINK_ZIGBEE);
  
  PORTD &= ~_BV(BIT_LINK_PLUGIN); // no pull up
  PORTD |= _BV(BIT_ZIGBEE_RESET);
  PORTD |= _BV(BIT_ENABLE_RXD_LINK_PC);
  PORTD |= _BV(BIT_ENABLE_RXD_LINK_ZIGBEE);
}


/** 
	Test procedures for debuging 
*/
void Test(void)
{
	static int tick=0;
	tick++;
	if(tick==128*1){
		ROBOTSTATUS = DO_PAGE; doPage(2);
	}
	if(tick==128*3){
		ROBOTSTATUS = DO_PAGE; doPage(62);
	}
	if(tick==128*7){
		ROBOTSTATUS=STOP_PAGE;
	}
	if(tick==128*10){
		tick=0;
	}
}

