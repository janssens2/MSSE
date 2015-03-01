/* lab01 - an application for the Pololu Orangutan SVP
 *
 * This application uses the Pololu AVR C/C++ Library.  For help, see:
 * -User's guide: http://www.pololu.com/docs/0J20
 * -Command reference: http://www.pololu.com/docs/0J18
 *
 * Created: 2/7/2015 11:27:04 AM
 *  Author: janssens
 */

#include <pololu/orangutan.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include "main.h"
#include "serial.h"
#include "leds.h"
#include "timer.h"
//#include <avr/interrupt.h>

extern uint8_t gRedEnable;

volatile uint8_t gReleaseRed;

void findFOR_COUNT_10MS( )
{
	volatile uint32_t start, end, counter;
	counter = 0;
	start = get_ms();
	for( ; counter < 4000000; counter++ );
	end = get_ms();
	
	printf("F: %lu", (10*4000000)/(end-start));
	delay_ms(5000);
	clear();
}



int main()
{
	
	// setup the lcd and show the user the program version
	// helps guarantee a good load.
	lcd_init_printf();
	clear();
	printf("Loaded v0.6");
	delay_ms(1000);
	clear();
	
	serial_init();
	
	init_leds( );
		
	// validate the wait for 10ms by looking at ticks and microseconds....
#ifdef FINDFORCOUNT_10MS
	uint32_t sec;
	uint32_t first = get_ticks();
	WAIT_10MS;
	sec = get_ticks();
	printf("ms = %lu", ticks_to_microseconds(sec - first) );
	delay_ms(5000);
	clear();
		
	findFOR_COUNT_10MS();
#endif
		
	setupTimers( );

	initTimerCounters( 0, OCR1A, 0 ); 
	
	sei();
	clear();
	
	while(1)
	{
		serial_check();
		
		check_for_new_bytes_received();
		
		if ( gReleaseRed && gRedEnable )
		{
			toggle_red();
			gReleaseRed = 0x0;
		}
	}
}