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

#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/signal.h>
#include <util/delay.h>

volatile unsigned long tick_counter = 0;

void sleep(unsigned long us)
{
    unsigned long time = 0; 
	
	us = us * 16L;
	
	while (time < us)
		time++;
}

// returns tickcount (resolution 20ms atm)
volatile unsigned long gettickcount(void)
{
  unsigned long tmp;

  cli(); // disable interrupt
  tmp = tick_counter;

  // TODO: why no if (SREG & (1 << SREG_I))  like in atmega128 tutorial?
  sei();
  return tmp;
}

// init timer - see atmega128 description
void init_timer(void)
{
	TCCR1B = 0; // disable ticking
	TIMSK = (TIMSK & ~(0x3c)) | 0x04; // enable overflow interrupt
	TCNT1H = 0; // reset counter
	TCNT1L = 0;
	ICR1H = 200U >> 8; // set overflow value 0.1ms
	ICR1L = 200U & 0xff;
	TCCR1A = 0xfe; // 11 11 11 10, set channel config
	TCCR1B = 0x1a; // 00011010 start ticking 
}

// interrupt handler
SIGNAL(SIG_OVERFLOW1)
{
  ++tick_counter;
}
