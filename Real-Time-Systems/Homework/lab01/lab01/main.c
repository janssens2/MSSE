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
//#include <avr/interrupt.h>

// following line is valid when __ii is not volatile
//#define FOR_COUNT_10MS 18065
// (23424 / 4) - 292
#define FOR_COUNT_10MS 5565
volatile uint32_t __ii;
#define WAIT_10MS {for(__ii=0; __ii<FOR_COUNT_10MS; __ii++);}

uint32_t gYellowToggles = 0;
uint32_t gGreenToggles = 0;
uint32_t gRedToggles = 0;

const uint16_t gRedMSTimerCount = 1;
const uint16_t gYellowMSTimerCount = 100;

volatile uint16_t gGreenToggleCycle = 500;
volatile uint16_t gRedToggleCycle = 500;
volatile uint16_t gYellowToggleCycle = 5;

uint8_t gGreenEnable = 0x1;
uint8_t gYellowEnable = 0x1;
uint8_t gRedEnable = 0x1;

volatile uint32_t gMS1Count = 0;
volatile uint32_t gMS100Count = 0;

volatile uint8_t gReleaseRed = 0;

#define BUFFER_SIZE 79
#define SERIAL_SEND_TIMEOUT 100

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

void toggle_green_iters( uint16_t ms_delay )
{
	if ( ms_delay == 0 )
	{
		// disable the timer
		TCCR1A &= ~((1 << COM1A1) | (1 << COM1A0));
	}
	else
	{
		// set ICR1 and OCR1A and OCR1B
		// ICR1 set to 2 * (20000000/1000) * (1/1024) * ms_delay
		//                   Hz        ms    prescaler
		// OCR1A is set to half of ICR2 for a 50% duty cycle
		
		uint16_t tmpVal = ((40000/1024) * ms_delay);
		serial_print( "Attempt to set GREEN to ICR1=0x%X delay(%d)", tmpVal, ms_delay );
		
		// disable the Timer
		TCCR1A &= ~((1 << COM1A1) | (1 << COM1A0));
		
		// change and reset the timer and counter
		ICR1 = tmpVal;
		OCR1A = ICR1 / 2;
		TCNT1 = 0;
		
		// re-enable the timer
		TCCR1A |= (1 << COM1A1) | (0 << COM1A0);
		
		gGreenToggleCycle = ms_delay;
	}
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
		PORTA &= ~(1 << PORTA2);
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
		PORTA &= ~(1 << PORTA0);
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
	
	DDRD |= (1 << DDD5);
	DDRA |= (1 << DDD0);
	DDRA |= (1 << DDD2);
	
	// validate the wait for 10ms by looking at ticks and microseconds....
	//uint32_t sec;
	//uint32_t first = get_ticks();
	//WAIT_10MS;
	//sec = get_ticks();
	//printf("ms = %lu", ticks_to_microseconds(sec - first) );
	//delay_ms(5000);
	//clear();
		
	//findFOR_COUNT_10MS();
		
	// setup of 8 bit timer to a 1ms resolution
	// COM0A1/COM0A0 set to 10=Clear OC0A on compare match (this is probably not necessary) in non-PWM mode table 15-2
	// COM0B1/COM0B0 not necessary to set table 15-5
	// WGM01/WGM00 set to 10= since we are using mode 2 CTC mode table 15-8
	TCCR0A = (1 << COM0A1) | (0 << COM0A0) | (0 << COM0B1) | (0 << COM0B0) | (0x0 << 2) | (1 << WGM01) | (0 << WGM00);
	// setup clock select table 15-9
	// WGM02 set to 0 since we are using mode 2 CTC mode table 15-8 (other WGM bits set in TCCR0A)
	// CS02/CS01/CS00 bits set to 100=clk io / 256 prescalar table 15-9
	TCCR0B = (0 << WGM02) | (1 << CS02) | (0 << CS01) | (0 << CS00);
	// top for output compare register A
	// 20000(ms) / 256(prescalar)(TCCR0B CS fields) = 78 (0x4E)
	// (20000ms / 256 * (0x4c + 1)) = 1.014ms
	// or (20000ms / 256) - 1 = 0x4D
	// 8-bit value when to generate an output compare interrupt
	OCR0A = 0x4D;
	// enable interrupts for Timer0 output compare match A
	TIMSK0 |= (1 << OCIE0A);

	// setup of 16 bit timer for 100ms resolution
	TCCR3A = (1 << COM3A1) | (0 << COM3A0) | (0 << COM3B1) | (0 << COM3B0) | (0x0 << 2) | (0 << WGM31) | (0 << WGM30);
	// setup clock select table 15-9
	TCCR3B = (0 << WGM33) | (1 << WGM32) | (0 << CS32) | (1 << CS31) | (1 << CS30);
	// top for output compare register A
	OCR3A = 0x7A11;
	// enable interrupts
	TIMSK3 |= (1 << OCIE3A);

	// setup timer for PWM
	// clock select bits to control frequency table 16.3 COM1A, clear on compare match
	TCCR1A = (1 << COM1A1) | (0 << COM1A0) | (0 << COM1B1) | (0 << COM1B0) | (0x0 << 2) | (1 << WGM11) | (0 << WGM10);
	// table 16-5 mode 14 for WGM bits (0x1D)
	TCCR1B = (0 << ICNC1) | (0 << ICES1) | (0x0 << 5) | (1 << WGM13) | (1 << WGM12) | (1 << CS12) | (0 << CS11) | (1 << CS10);
	// set ICR for (whatever) top  (ICR1 for timer1)
	// = 2x
	//ICR1 = 0x2625;
	//ICR1 = 0x1312;
	ICR1 = 0x4C4A;
	// set OCR1A for match, should equal top/2 for 50% duty cycle
	OCR1A = ICR1/2;
	OCR1B = OCR1A;
	
	TIMSK1 |= (1 << OCIE1A) | (1 << TOIE1);

	TCNT0 = TCNT3 = 0;
	TCNT1 = OCR1A;

	sei();

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

ISR(TIMER0_COMPA_vect)
{
	gMS1Count++;
	//lcd_goto_xy(0,0);
	//printf("%.5lu", hitme1 % 100000);
	if ( (gMS1Count % gRedToggleCycle) == 0 )
	{
		gReleaseRed = 0x1;
	}
}

ISR(TIMER3_COMPA_vect)
{
#ifdef YELLOW_510MS_BUSYWAIT_SEI
	sei();
#endif
	gMS100Count++;
	//lcd_goto_xy(0,1);
	//printf("%.5lu", hitme2 % 100000);
	if ( (gMS100Count % gYellowToggleCycle) == 0 && gYellowEnable )
	{
		toggle_yellow();
	}
#ifdef YELLOW_90MS_BUSYWAIT
	for( int i=0; i < 9; i++)
	{
		WAIT_10MS;
	}
#endif
#ifdef YELLOW_110MS_BUSYWAIT
	for( int i=0; i < 11; i++)
	{
		WAIT_10MS;
	}
#endif
#ifdef YELLOW_510MS_BUSYWAIT
	for( int i=0; i < 51; i++)
	{
		WAIT_10MS;
	}
#endif
}

ISR(TIMER1_COMPA_vect)
{
	// Using the COMPA of timer 1 to register a toggle when the ICR is met
	gGreenToggles++;
	
}

ISR(TIMER1_OVF_vect)
{
#ifdef GREEN_510ms_BUSYWAIT_SEI
	sei();
#endif
	// Using the overflow timer to register a toggle when OCR is met
	gGreenToggles++;
#ifdef GREEN_90MS_BUSYWAIT
	for( int i=0; i < 9; i++)
	{
		WAIT_10MS;
	}
#endif
#ifdef GREEN_110MS_BUSYWAIT
	for( int i=0; i < 11; i++)
	{
		WAIT_10MS;
	}
#endif
#ifdef GREEN_510MS_BUSYWAIT
	for( int i=0; i < 51; i++)
	{
		WAIT_10MS;
	}
#endif
}