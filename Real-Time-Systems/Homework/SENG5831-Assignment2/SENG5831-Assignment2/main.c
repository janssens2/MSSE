/* SENG5831-Assignment2 - an application for the Pololu Orangutan SVP
 *
 * This application uses the Pololu AVR C/C++ Library.  For help, see:
 * -User's guide: http://www.pololu.com/docs/0J20
 * -Command reference: http://www.pololu.com/docs/0J18
 *
 * Created: 1/30/2015 11:19:35 AM
 *  Author: janssens
 */

#include <pololu/orangutan.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include "SENG5831Assignment2.h"

const uint8_t led_config[2][2] = {{ DDD2, PORTD2 }, { DDD3, PORTD3 } };
uint32_t blinkInterval[2] = { 400, 700 };
uint8_t leds = 2;

int main()
{
	uint32_t cycles = 0;
	uint8_t index = 0;
	uint8_t blinkState[] = { 1, 1 };
	uint32_t blinkCycles[] = { 0, 0 };

	
	// Set the baud rate to 9600 bits per second.  Each byte takes ten bit
	// times, so you can get at most 960 bytes per second at this speed.
	serial_set_baud_rate(USB_COMM, 9600);

	// Start receiving bytes in the ring buffer.
	serial_receive_ring(USB_COMM, receive_buffer, sizeof(receive_buffer));
	
	// setup the lcd and show the user the program version
	// helps guarantee a good load.
	lcd_init_printf();
	clear();
	print("Loaded v0.1");
	delay_ms(1000);

	clear();
	// Setup the LED's we are working with...
	led_initialize( &DDRD, 2, DDD2, DDD3 );
	
	// Turn on the LEDs we are working with...
	for ( index = 0; index < leds; index++ )
	{
		led_on( &DDRD, &PORTD, led_config[index][PORTBIT] );
	}
	
	clear();
	printf("LED(s) ARE ON!");
	
	delay_ms(5000);
	
	// Turn off the LEDs we are working with...
	for ( index = 0; index < leds; index++ )
	{
		led_off( &DDRD, &PORTD, led_config[index][PORTBIT] );
	}
	
	clear();
	
	while(1)
	{
		cycles = get_ms();
		
		// call this method often to make sure serial mode is checked
		serial_check();
		
		check_for_new_bytes_received();
		
		// check for ANY_BUTTON to be pressed
		unsigned char button = get_single_debounced_button_press(ANY_BUTTON);
		// TOP_BUTTON increases blink rate
		if ( button & TOP_BUTTON )
		{
			clear();
			modify_interval( &blinkInterval[0], leds, 100 );
		}
		
		// BOTTOM_BUTTON decreases blink rate
		if ( button & BOTTOM_BUTTON )
		{
			modify_interval( &blinkInterval[0], leds, -100 );
		}
		
		// MIDDLE_BUTTON does something???
		if ( button & MIDDLE_BUTTON )
		{
			clear();
			print("MIDDLE button pressed what now?");
			process_send_bytes("Middle button???");
			process_send_bytes("Really, NYI");
		}
		
		for ( index=0; index < leds; index++ )
		{
			if ( blinkState[index] && ((blinkCycles[index] + blinkInterval[index]) < cycles) )
			{
				led_toggle( &PORTD, led_config[index][PORTBIT] );
				blinkCycles[index] = cycles;
			}
		}
		
	}
}

void modify_interval( uint32_t * led_intervals, const uint8_t leds, int8_t change )
{
	uint8_t index = 0;
	
	clear();
	for ( index = 0; index < leds; index++ )
	{
		led_intervals[index] = (led_intervals[index] > abs(change)) ? led_intervals[index] + change : 500;
		lcd_goto_xy( 0, index );
		printf("%s: %.5lu", (index == 0) ? "TOP" : "BOT", led_intervals[index]);
	}
	
}

void led_initialize( volatile uint8_t *ddr, uint8_t numArgs, ... )
{
	uint8_t index = 0;
	uint8_t bits = 0x0;
	
	va_list args;
	va_start( args, numArgs );
	
	for ( index = 0; index < numArgs; index++ )
	{
		bits |= (1 << va_arg(args, int));
	}
	
	(*ddr) |= bits;
}

void led_on( volatile uint8_t *ddr, volatile uint8_t *port, uint8_t bit )
{
	if ( ddr == &DDRD && port == &PORTD && bit == PORTD1 )
	{
		// special case to handle the red led onboard
		// the red onboard led is active low
		(*port) &= ~(1 << bit);
	}
	else
	{
		(*port) |= (1 << bit);
	}
}

void led_off( volatile uint8_t *ddr, volatile uint8_t *port, uint8_t bit )
{
	if ( ddr == &DDRD && port == &PORTD && bit == PORTD1)
	{
		// special case to handle the red led onboard
		// the red onboard led is active low
		(*port) |= (1 << bit);
	}
	else
	{
		(*port) &= ~(1 << bit);
	}
	
}

void led_toggle( volatile uint8_t *port, uint8_t bit )
{
	(*port) ^= (1 << bit);
}

void wait_for_sending_to_finish()
{
	while(!serial_send_buffer_empty(USB_COMM))
		serial_check();		// USB_COMM port is always in SERIAL_CHECK mode
}

void process_received_byte(char byte)
{
	clear();		// clear LCD
	print("RX: ");
	print_character(byte);
	lcd_goto_xy(0, 1);	// go to start of second LCD row

	switch(byte)
	{
		// 
		case '?':
			process_send_bytes( "Help is on its way!" );
			process_send_bytes( "+   Increase LED blink Interval" );
			process_send_bytes( "-   Decrease LED blink Interval" );
			process_send_bytes( "l   List LEDs" );
			break;

		// 
		case '+':
			modify_interval( &blinkInterval[0], leds, 100 );
			break;

		// 
		case '-':
			modify_interval( &blinkInterval[0], leds, -100 );
			break;

		// 
		case 'l':
			process_send_bytes( "Not yet implemented" );
			break;

		default:
			// do nothing
			break;
	}
}

void check_for_new_bytes_received()
{
	while(serial_get_received_bytes(USB_COMM) != receive_buffer_position)
	{
		// Process the new byte that has just been received.
		process_received_byte(receive_buffer[receive_buffer_position]);

		// Increment receive_buffer_position, but wrap around when it gets to
		// the end of the buffer.
		if (receive_buffer_position == sizeof(receive_buffer)-1)
		{
			receive_buffer_position = 0;
		}
		else
		{
			receive_buffer_position++;
		}
	}
}

void process_send_bytes( const char *myString )
{
	uint32_t parmSize = strlen(myString);
	uint32_t bufSize = sizeof(send_buffer);
	wait_for_sending_to_finish();
	
	memset( send_buffer, 0, bufSize );
	strncpy( send_buffer, myString, (bufSize > parmSize) ? parmSize : bufSize );
	serial_send( USB_COMM, send_buffer, (bufSize > parmSize) ? parmSize : bufSize );
	
	wait_for_sending_to_finish();
	memset( send_buffer, 0, bufSize );
	send_buffer[0] = '\r';
	send_buffer[1] = '\n';
	serial_send( USB_COMM, send_buffer, sizeof(send_buffer) );
}