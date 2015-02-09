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
#include <avr/interrupt.h>

uint32_t hitme = 0;
volatile uint8_t release_red = 0;

int main()
{
	// setup the lcd and show the user the program version
	// helps guarantee a good load.
	lcd_init_printf();
	clear();
	print("Loaded v0.3");
	delay_ms(1000);
	clear();
		
	// setup timer
	TCCR0A = (1 << COM0A1) | (0 << COM0A0) | (0 << COM0B1) | (0 << COM0B0) | (0x0 << 2) | (1 << WGM01) | (0 << WGM00);
	TCCR0B = (1 << CS02) | (0 << CS01) | (1 << CS00);
	OCR0A = 0xFF;
	
	// enable interrupts
	TIMSK0 = (1 << OCIE0A);

	sei();

	while(1)
	{
		if ( release_red )
		{
			red_led(TOGGLE);
			release_red = 0x0;
		}
	}
}

ISR(TIMER0_COMPA_vect)
{
	hitme++;
	//clear();
	lcd_goto_xy(0,0);
	printf("%.5lu", hitme % 100000);
	if ( (hitme % 500) == 0 )
	{
		release_red = 0x1;
	}
}
