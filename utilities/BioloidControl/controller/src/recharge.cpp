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

// recharge batteries functions
// thanks to kess and pieddemamouth (robosavvy-forums)
unsigned int readADC(byte channel)
{
    ADMUX  = 0x40 | channel;
    ADCSRA = 0xC6;
    while(bit_is_set(ADCSRA, ADSC));
    return (ADCL | (ADCH<<8));
}

float getTrimmedAverage(unsigned int *voltage, int len, int trim)
{
			unsigned int min;
			int minId, i, j;
			for (i=0; i<len; i++)
			{
				min = voltage[i];
				minId = i;
				for (j=i+1; j<len; j++)
					if (voltage[j] < min)
					{
						minId = j;
						min = voltage[j];
					}
					
				min = voltage[i];
				voltage[i] = voltage[minId];
				voltage[minId] = min;
			}
			
			// calc average (without REMOVECYCLES largest and smallest values)
			unsigned long int tmp = voltage[trim];
			for (i=trim+1; i<len-trim; i++)
			{
					tmp += voltage[i];
			}
			
			return (float) tmp / ((float)(len-2*trim));
} 

void rechargeBatteries(unsigned int maxCycles)
{
	// configuration
	cli();
	
	PORTE |= _BV(PE4);    //enable pull-up resistor 4
    EIMSK |= _BV(INT4);   //enable interrupt 4
    DDRC = 0xFF;
    PORTC = 0xFF;
    DDRB |= _BV(DDB5);  //output for bit PB5

    //ADC configuration
    ADMUX = 0x10;
    ADCSRA = 0x80; 
	
    sei();
	
	unsigned int voltage[RECHARGECYCLES], voltageCounter, counter, average, oldAverage;
	byte buffer[8];
	
	// enable charging
	PORTB &= ~PB5; 
	
	counter = 0;
	voltageCounter = 0;
	average = 0;
	oldAverage = 10000;
	
	while (true)
	{
	    voltage[voltageCounter] = readADC(1);
		voltage[voltageCounter] *= 2;
		voltageCounter++;
		
		if (voltageCounter >= RECHARGECYCLES)
		{
			voltageCounter = 0;
			
			average = (unsigned int) (getTrimmedAverage(voltage, RECHARGECYCLES, RECHARGEREMOVECYCLES) + 0.5);
			
			if (average >= oldAverage) // unchanged
				counter++;
			else
			{
				counter = 0; // reset counter if changed
				oldAverage = average;
			}
		}
		
		if (counter >= maxCycles)
		{
			buffer[0] = 0xFF;
			buffer[1] = 0xFF;
			buffer[2] = 0xFF;
			buffer[3] = 0xFF;
			buffer[4] = 0xFF;
			buffer[5] = 0xFF;
			buffer[6] = 0xFF;
			buffer[7] = 0xFF;
			writeData(buffer, 8);
			break;
		}	
			
		buffer[0] = voltage[(voltageCounter - 1) % RECHARGECYCLES]  & 0xFF;
		buffer[1] = (voltage[(voltageCounter - 1) % RECHARGECYCLES] ) >> 8;
		buffer[2] = average & 0xFF;
		buffer[3] = (average) >> 8;
		buffer[4] = counter & 0xFF;
		buffer[5] = (counter & 0xFF00) >> 8;
		buffer[6] = maxCycles & 0xFF;
		buffer[7] = (maxCycles & 0xFF00) >> 8;
		writeData(buffer, 8);
		
		sleep(RECHARGEPAUSE); //100ms
	}
	
	// disable charging
	PORTB |= PB5; 
}
