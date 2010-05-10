#include <avr/io.h>
#include "led.h"

/****************************************************************************\
*   LED_Init: Set port C bit 0 and 1, and bit 0 port B as LED output
*           : Set port C bit 2 and 3 as output pins for debugging ISR's
\****************************************************************************/

void LED_Init(void)
{
    DDRC |= 0x7F; // set bits 0 and 1 on port C as output; 
	PORTC = 0x7F; // switch off all LEDs
}


/****************************************************************************\
*   Set_LED: Control the LED state
*   Example Set_LED(LED_GRN,LED_ON);
\****************************************************************************/
void Set_LED(unsigned char LEDno, unsigned char state)
{
     switch (state)
    {
        case LED_OFF:
            PORTC |= 1 << LEDno;			// LEDs are inverted logic
            break;
        case LED_ON:
            PORTC &= ~(1<< LEDno);			// LEDs are inverted logic
            break;
        case LED_TOGGLE:
            PORTC ^= (1 << LEDno);
            break;
        default:
            break;
    }

}
