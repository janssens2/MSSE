/* SENG5831-Assignment2 - an application for the Pololu Orangutan SVP
 *
 * This application uses the Pololu AVR C/C++ Library.  For help, see:
 * -User's guide: http://www.pololu.com/docs/0J20
 * -Command reference: http://www.pololu.com/docs/0J18
 *
 * Created: 1/30/2015 11:19:35 AM
 *  Author: janssens
 *
 *
 */

#include <pololu/orangutan.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include "SENG5831Assignment2.h"

int main()
{
	uint32_t cycles = 0;
	uint8_t index = 0;
	
	int32_t intervalModifier = 100;
	
	// DDR address, DDR bit, PORT address, PORT bit, cycle interval, cycles last updated, enable blink
	const uint8_t ledConfigSize = 2;
	led_info_t ledConfig[2] = {
								{ &DDRD, DDD3, &PORTD, PORTD3, 700, 0, 1 }, 
								{ &DDRD, DDD2, &PORTD, PORTD2, 500, 0, 1 }
							 };
	
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
	led_initializer( ledConfigSize, ledConfig );
	
	// Turn on the LEDs we are working with...
	led_onr( ledConfigSize, ledConfig );
	
	clear();
	printf("LED(s) ARE ON!");
	
	delay_ms(5000);
	
	// Turn off the LEDs we are working with...
	led_offr( ledConfigSize, ledConfig );
	
	clear();
	
	while(1)
	{
		cycles = get_ms();
		
		// call this method often to make sure serial mode is checked
		serial_check();
		
		check_for_new_bytes_received( ledConfigSize, ledConfig );
		
		// check for ANY_BUTTON to be pressed
		unsigned char button = get_single_debounced_button_press(ANY_BUTTON);
		// TOP_BUTTON increases blink rate
		if ( button & TOP_BUTTON )
		{
			clear();
			modify_interval( ledConfigSize, ledConfig, intervalModifier );
		}
		
		// BOTTOM_BUTTON decreases blink rate
		if ( button & BOTTOM_BUTTON )
		{
			modify_interval( ledConfigSize, ledConfig, (intervalModifier * -1) );
		}
		
		// MIDDLE_BUTTON does something???
		if ( button & MIDDLE_BUTTON )
		{
			clear();
			intervalModifier = rand() % 1000;
			printf("MIDDLE = %ld", intervalModifier );
			process_send_bytes( "modified interval modifier" );
		}
		
		for ( index=0; index < ledConfigSize; index++ )
		{
			if ( ledConfig[index].blinkState && ((ledConfig[index].blinkCycles + ledConfig[index].blinkInterval) < cycles) )
			{
				led_toggler( ledConfig[index] );
				ledConfig[index].blinkCycles = cycles;
			}
		}
		
	}
}

void modify_interval( uint8_t ledInfoSize, led_info_t *ledsInfo, int32_t change )
{
	uint8_t iterator = 0;
	char myString[32];
	
	clear();
	
	while ( iterator < ledInfoSize )
	{
		ledsInfo[iterator].blinkInterval = (ledsInfo[iterator].blinkInterval > abs(change)) ? ledsInfo[iterator].blinkInterval + change : ((rand() % 1000) + 100);
		lcd_goto_xy( 0, iterator % 2 );
		if ( iterator < 2 )
		{
			printf("%s: %.5lu", (iterator == 0) ? "TOP" : "BOT", ledsInfo[iterator].blinkInterval);
		}
		memset( myString, 0, sizeof(myString) );
		sprintf( myString, "%p/%p(%d) = %.5lu", &(ledsInfo[iterator].ddr_loc), &(ledsInfo[iterator].port_loc), ledsInfo[iterator].port_bit, ledsInfo[iterator].blinkInterval );
		process_send_bytes( myString );
		iterator++;
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

void led_initializer( uint8_t ledInfoSize, led_info_t *ledsInfo )
{
	uint8_t iterator = 0;

	while ( iterator < ledInfoSize )
	{
		led_initialize( ledsInfo[iterator].ddr_loc, 1, ledsInfo[iterator].dd_bit );
		iterator++;
	}
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

void led_onr( uint8_t ledInfoSize, led_info_t *ledsInfo )
{
	uint8_t iterator = 0;
	
	while ( iterator < ledInfoSize )
	{
		led_on( ledsInfo[iterator].ddr_loc, ledsInfo[iterator].port_loc, ledsInfo[iterator].port_bit );
		iterator++;
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

void led_offr( uint8_t ledInfoSize, led_info_t *ledsInfo )
{
	uint8_t iterator = 0;
	
	while ( iterator < ledInfoSize )
	{
		led_off( ledsInfo[iterator].ddr_loc, ledsInfo[iterator].port_loc, ledsInfo[iterator].port_bit );
		iterator++;
	}
}

void led_toggle( volatile uint8_t *port, uint8_t bit )
{
	(*port) ^= (1 << bit);
}

void led_toggler( led_info_t ledsInfo )
{
	(*ledsInfo.port_loc) ^= (1 << ledsInfo.port_bit);
}

void wait_for_sending_to_finish()
{
	while(!serial_send_buffer_empty(USB_COMM))
		serial_check();		// USB_COMM port is always in SERIAL_CHECK mode
}

void process_received_byte(char byte, uint8_t ledInfoSize, led_info_t *ledsInfo)
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
			modify_interval( ledInfoSize, ledsInfo, 100 );
			break;

		// 
		case '-':
			modify_interval( ledInfoSize, ledsInfo, -100 );
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

void check_for_new_bytes_received( uint8_t ledInfoSize, led_info_t *ledsInfo )
{
	while(serial_get_received_bytes(USB_COMM) != receive_buffer_position)
	{
		// Process the new byte that has just been received.
		process_received_byte(receive_buffer[receive_buffer_position], ledInfoSize, ledsInfo );

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