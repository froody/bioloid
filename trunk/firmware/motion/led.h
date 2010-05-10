#ifndef LED_H
#define LED_H

// Available LED's
#define LED_POWER		0
#define LED_TXD			1
#define LED_RXD			2
#define LED_AUX			3
#define LED_MANAGE		4
#define LED_PROGRAM		5
#define LED_PLAY		6

// LED states
#define LED_OFF 0
#define LED_ON  1
#define LED_TOGGLE 2

void LED_Init(void);
void Set_LED(unsigned char LEDno, unsigned char state);

#endif
