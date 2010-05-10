/**
 -------------------------------------
 ---- SERIAL PROTOCOL DEFINITION -----
 ----  PDA <--> BioLoid Controller ---
 -------------------------------------

 University of Plymouth Bioloid Robot Controller
 for Atmel Mega 128
 FIRA England Robot Football Team
 Author : Joerg Wolf, joerg.wolf -xaxtx- plymouth.ac.uk
 Version 0.12

 Baudrate: 57600bps
 
 
 Byte	0	1	2	3	4	5	6	7	8	9
			C						C	C
		[	O	P1	P2	P3	P4	P5	R	R	]
			M						C	C

COM stands for "command"
Px stands for "Parameter"
 
 if COM == DO_PAGE then P1 contains the page number
 if COM == DO_POSE then P1 contains the page number and P2 the pose number
 if COM == STOP_NOW then the robot will freeze imidiately
 if COM == STOP_POSE then the robot will stop at the end of the next pose
 if COM == STOP_PAGE then the robot will stop at the end of the next page
 if COM == VERSION then the robot will reply with a text string [BLV1.11]
           (Useful command to see if the robot controller is connected)
 if COM == TURN_SERVO then P1 = servono , P2 = HSB Pos , P3 = LSB Pos, P4 = HSB Speed , P5 = LSB Speed
 the hash defines for the COM-byte:
*/

#define DO_PAGE		0x31
#define DO_POSE		0x32
#define STOP_NOW	0x00
#define STOP_POSE	0x33
#define STOP_PAGE	0x34
#define VERSION		0x35
#define SERVO_TRIM	0x36
#define BUGGY_MOVE	0x37

// new 
#define BALANCE		0x38
#define TURN_SERVO	0x39





