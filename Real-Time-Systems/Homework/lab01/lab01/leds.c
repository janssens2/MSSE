/*
 * leds.c
 *
 * Created: 3/1/2015 2:32:56 PM
 *  Author: janssens
 */ 

#include <pololu/orangutan.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include "serial.h"
#include "leds.h"

volatile uint16_t gGreenToggleCycle = 500;
volatile uint16_t gRedToggleCycle = 500;
volatile uint16_t gYellowToggleCycle = 5;

uint32_t gYellowToggles = 0;
uint32_t gGreenToggles = 0;
uint32_t gRedToggles = 0;

uint8_t gGreenEnable = 0x1;
uint8_t gYellowEnable = 0x1;
uint8_t gRedEnable = 0x1;

extern const uint16_t gRedMSTimerCount;
extern const uint16_t gYellowMSTimerCount;

void toggle_green_iters( uint16_t ms_delay )
{
	serial_print( "setting GREEN to %d", ms_delay );
	
	if ( ms_delay == 0 )
	{
		// disable the timer
		TCCR1A &= ~((1 << COM1A1) | (1 << COM1A0));
		// turn off green
		turnoff_green();
	}
	else
	{
		// determine if our delay will exceed the 16 bit counter field for our top...
		if ( ms_delay > 1680 )
		{
			serial_print( "Attempt to modify Green to %d FAILED, its larger than uint16!", ms_delay );
			return;
		}
		uint16_t tmpVal = ((40000/1024) * ms_delay);
		serial_print( "Attempt to set GREEN to ICR1=0x%.4X delay(%d)", tmpVal, ms_delay );
		
		// disable the Timer
		TCCR1A &= ~((1 << COM1A1) | (1 << COM1A0));
		
		// change and reset the timer and counter
		ICR1 = (uint16_t) tmpVal;
		OCR1A = ICR1 / 2;
		TCNT1 = 0;
		
		// re-enable the timer
		TCCR1A |= (1 << COM1A1) | (0 << COM1A0);
		
		gGreenToggleCycle = ms_delay;
	}
}

void enable_green( )
{
	DDRD |= (1 << DDD5);
}

void toggle_green( )
{
	PORTD ^= (1 << PORTD5);
}

void turnoff_green( )
{
	PORTD &= ~(1 << PORTD5);
}

void enable_red( )
{
	DDRA |= (1 << DDD2);
}

void turnoff_red( )
{
	PORTA &= ~(1 << PORTA2);
}

void toggle_red( )
{
	PORTA ^= (1 << PORTA2);
	
	// only increment the counter when we complete a full toggle on - off to just before next on
	//if ( (PORTA & (1 << PORTA2)) == 0 )
	//{
	gRedToggles++;
	//}
}

uint16_t toggle_generic_iters( uint16_t ms_delay, volatile uint16_t *colorToggle, const uint16_t colorTimer )
{
	uint16_t rc = 0;
	
	if ( ms_delay > colorTimer )
	{
		*colorToggle = (ms_delay / colorTimer);
	}
	else
	{
		rc = 1;
	}
	return rc;
}

void toggle_red_iters( uint16_t ms_delay )
{
	uint16_t rc = 0;
	if ( ms_delay == 0 )
	{
		// turn off the red led, but keep the timer running
		turnoff_red();
		gRedEnable = 0x0;
	}
	else
	{
		rc = toggle_generic_iters( ms_delay, &gRedToggleCycle, gRedMSTimerCount );
		gRedEnable = 0x1;
	}
	
	serial_print( "setting RED to %d(%d)", ms_delay, gRedToggleCycle );
	
	if ( rc )
	{
		serial_print( "(%d) Attempt to modify Red to %d FAILED!", rc, ms_delay );
	}
}

void enable_yellow( )
{
	DDRA |= (1 << DDD0);
}

void turnoff_yellow( )
{
	PORTA &= ~(1 << PORTA0);
}

void toggle_yellow( )
{
	PORTA ^= (1 << PORTA0);
	
	// only increment the counter when we complete a full toggle on - off to just before next on
	//if ( (PORTA & (1 << PORTA0)) == 0 )
	//{
	gYellowToggles++;
	//}
}

void toggle_yellow_iters( uint16_t ms_delay )
{
	uint16_t rc = 0;
	
	if ( ms_delay == 0 )
	{
		// turn off the red led, but keep the timer running
		turnoff_yellow();
		gYellowEnable = 0x0;
	}
	else
	{
		rc = toggle_generic_iters( ms_delay, &gYellowToggleCycle, gYellowMSTimerCount );
		gYellowEnable = 0x1;
	}
	
	
	serial_print( "setting Yellow to %d(%d)", ms_delay, gYellowToggleCycle );
	
	if ( rc )
	{
		serial_print( "(%d) Attempt to modify Yellow to %d FAILED!", rc, ms_delay );
	}
}

void init_leds( )
{
	
	enable_green();
	enable_red();
	enable_yellow();
	
	toggle_green();
	toggle_red();
	toggle_yellow();
	
	clear();
	print( "Testing LEDs" );
	delay_ms( 1000 );
	
	toggle_green();
	toggle_red();
	toggle_yellow();
}