/*
 * serial.c
 *
 * Created: 2/28/2015 4:57:41 PM
 *  Author: janssens
 */ 

#include <pololu/orangutan.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "serial.h"
#include "leds.h"

extern uint32_t gYellowToggles;
extern uint32_t gGreenToggles;
extern uint32_t gRedToggles;

char my_received_buffer[BUFFER_SIZE];
uint8_t bytes_received = 0;

// receive_buffer: A ring buffer that we will use to receive bytes on USB_COMM.
// The OrangutanSerial library will put received bytes in to
// the buffer starting at the beginning (receiveBuffer[0]).
// After the buffer has been filled, the library will automatically
// start over at the beginning.
char receive_buffer[BUFFER_SIZE];

// receive_buffer_position: This variable will keep track of which bytes in the receive buffer
// we have already processed.  It is the offset (0-31) of the next byte
// in the buffer to process.
unsigned char receive_buffer_position = 0;

// send_buffer: A buffer for sending bytes on USB_COMM.
char send_buffer[BUFFER_SIZE];

void wait_for_sending_to_finish()
{
	int32_t timeout = SERIAL_SEND_TIMEOUT;
	// need to have a timer here that can timeout
	// this helps in the case the serial port is not connected and
	// we could hang here until reset.
	while(!serial_send_buffer_empty(USB_COMM) && ( 0 < timeout-- ) )
	{
		serial_check();		// USB_COMM port is always in SERIAL_CHECK mode
	}
}

void print_menu( )
{
	serial_print( "Menu: <operation> <color> [value]" );
	serial_print( "   Operation" );
	serial_print( "     z: Zero the counter for LED color specified" );
	serial_print( "     p: Print the counter for LED color specified" );
	serial_print( "     t: Toggle the LED color selected every value specified" );
	serial_print( "   Color" );
	serial_print( "     r: Red LED" );
	serial_print( "     g: Green LED" );
	serial_print( "     y: Yellow LED" );
	serial_print( "     a: All LEDs" );
	serial_print( "   Value" );
	serial_print( "     Any integer value to set to" );
	serial_print( "   ? : To view this help again" );
	serial_print( "   Examples" );
	serial_print( "     t r 250   - toggle red led at 250ms" );
	serial_print( "     t a 2000  - all LEDs toggle at 2000ms" );
	serial_print( "     t y 0     - Toggle yellow led off" );
	serial_print( "     z r       - zero the toggle counter for the red LED" );
}

void process_received_bytes( char bytes[] )
{
	char operation, color;
	int32_t value = 0;
	
	sscanf( bytes, "%c %c %ld", &operation, &color, &value);
	
	serial_print( "OP:%c CO:%c VA:%ld", operation, color, value );
	//snprintf( tmpBuffer, BUFFER_SIZE, "OP:%c CO:%c VA:%ld", operation, color, value );
	//serial_print_string( &tmpBuffer );
	
	// code to view the buffer in all its glory
	//uint8_t counter = 0;
	//for( ; bytes[counter] != NULL; counter++ )
	//{
	//memset( tmpBuffer, 0, BUFFER_SIZE );
	//snprintf( tmpBuffer, BUFFER_SIZE, "%d: Chr(%c) Int(%d)", counter, bytes[counter], bytes[counter] );
	//serial_print_string( &tmpBuffer );
	//}

	switch(operation)
	{
		//
		case '?':
		// print the menu again
		print_menu();
		break;

		//
		case 'Z':
		case 'z':
			switch( color )
			{
				case 'Y':
				case 'y':
					gYellowToggles = 0;
					break;
				
				case 'G':
				case 'g':
					gGreenToggles = 0;
					break;
				
				case 'R':
				case 'r':
					gRedToggles = 0;
					break;
				
				case 'A':
				case 'a':
					gYellowToggles = gGreenToggles = gRedToggles = 0;
					break;
				
				default:
					serial_print( "Invalid color identifier, try again" );
					print_menu();
					break;
			}
			break;

		//
		case 'P':
		case 'p':
			switch( color )
			{
				case 'Y':
				case 'y':
					serial_print( "Yellow toggles: %lu", gYellowToggles );
					break;
				
				case 'G':
				case 'g':
					serial_print( "Green toggles:  %lu", gGreenToggles );
					break;
				
				case 'R':
				case 'r':
					serial_print( "Red toggles:    %lu", gRedToggles );
					break;
				
				case 'A':
				case 'a':
					serial_print( "Red toggles:    %lu", gRedToggles );
					serial_print( "Green toggles:  %lu", gGreenToggles );
					serial_print( "Yellow toggles: %lu", gYellowToggles );
					break;
				
				default:
					serial_print( "Invalid color identifier, try again" );
					print_menu();
					break;
			}
			break;

		//
		case 'T':
		case 't':
			switch( color )
			{
				case 'Y':
				case 'y':
					toggle_yellow_iters( (uint16_t)value );
					break;
				
				case 'G':
				case 'g':
					toggle_green_iters( (uint16_t)value );
					break;
				
				case 'R':
				case 'r':
					toggle_red_iters( (uint16_t)value );
					break;
				
				case 'A':
				case 'a':
					toggle_red_iters( (uint16_t)value );
					toggle_green_iters( (uint16_t)value );
					toggle_yellow_iters( (uint16_t)value );
					break;
				
				default:
					serial_print( "Invalid color identifier, try again" );
					print_menu();
					break;
			}
			break;

		default:
			// something wrong ????
			serial_print( "Unknown Operation, try again!" );
			break;
	}
}

void strip( char bytes[], size_t mySize )
{
	uint32_t counter = 0;
	
	for( ; counter < mySize && bytes[counter] != '\0'; counter++ )
	{
		if ( bytes[counter] != 32 &&
		bytes[counter] != 63 &&
		(bytes[counter] < 48 || bytes[counter] > 57) &&
		(bytes[counter] < 65 || bytes[counter] > 122) )
		{
			bytes[counter] = 0x0;
		}
	}
}

void check_for_new_bytes_received( )
{
	while(serial_get_received_bytes(USB_COMM) != receive_buffer_position)
	{
		// Remember the byte that was received, ignore if user pushed too many bytes
		if ( bytes_received > sizeof(my_received_buffer) )
		{
			bytes_received = 0;
			memset( my_received_buffer, 0, sizeof(my_received_buffer) );
			serial_print( "Buffer Exceeded, try again!" );
		}
		else
		{
			if ( receive_buffer[receive_buffer_position] == 0x8 || receive_buffer[receive_buffer_position] == 0x7F )
			{
				// treat this as a backspace
				bytes_received = (bytes_received > 0) ? bytes_received - 1 : bytes_received;
				my_received_buffer[bytes_received] = '\0';
			}
			else
			{
				my_received_buffer[bytes_received] = receive_buffer[receive_buffer_position];
				bytes_received++;
			}
			
			// echo what was typed on the serial console back to the serial console
			serial_print_char( receive_buffer[receive_buffer_position] );
		}
		
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
	
	if ( bytes_received && (my_received_buffer[bytes_received-1] == '\r' || my_received_buffer[bytes_received-1] == '\n') )
	{
		// echo string to serial console
		serial_print( "%s", my_received_buffer );
		
		// check makes sure we have atleast 3 bytes of data from user interface
		if ( bytes_received > 1)
		{
			strip( my_received_buffer, sizeof(my_received_buffer) );
			process_received_bytes( my_received_buffer );
		}
		
		clear();
		lcd_goto_xy(0,0);
		printf("t: %s", my_received_buffer );
		
		memset( my_received_buffer, 0, sizeof(my_received_buffer) );
		bytes_received = 0;
	}
}

void serial_print_string( const char myString[] )
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
	wait_for_sending_to_finish();
}

void serial_print( char *format, ... )
{
	va_list args;
	va_start( args, format );
	
	char myBuffer[BUFFER_SIZE];
	memset( myBuffer, 0, BUFFER_SIZE );
	vsprintf( myBuffer, format, args );
	
	serial_print_string( myBuffer );
}

void serial_print_char( const char myChar )
{
	wait_for_sending_to_finish();
	memset( send_buffer, 0, sizeof(send_buffer) );
	send_buffer[0] = myChar;
	serial_send( USB_COMM, send_buffer, 1 );
}

void serial_init( )
{
	// Set the baud rate to 9600 bits per second.  Each byte takes ten bit
	// times, so you can get at most 960 bytes per second at this speed.
	serial_set_baud_rate(USB_COMM, 9600);

	// Start receiving bytes in the ring buffer.
	serial_receive_ring(USB_COMM, receive_buffer, sizeof(receive_buffer));
	
	memset( my_received_buffer, 0, sizeof(my_received_buffer));
	
	serial_print_string( "USB Serial Initialized" );
	serial_print_string( "" );
	print_menu();
}