/*
 * timer.c
 *
 * Created: 2/28/2015 4:59:23 PM
 *  Author: janssens
 */ 

#include <pololu/orangutan.h>
#include <stdlib.h>
#include <stdio.h>
#include "leds.h"

const uint16_t gRedMSTimerCount = 1;
const uint16_t gYellowMSTimerCount = 100;

volatile uint32_t gMS1Count = 0;
volatile uint32_t gMS100Count = 0;

extern volatile uint32_t gMS1Count;
extern volatile uint32_t gMS100Count;
extern uint32_t gGreenToggles;
extern uint8_t gYellowEnable;
extern volatile uint8_t gReleaseRed;
extern volatile uint16_t gRedToggleCycle;
extern volatile uint16_t gYellowToggleCycle;

void initTimer0( )
{
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
}

void initTimer1( )
{
	// setup timer for PWM
	// clock select bits to control frequency table 16.3 COM1A, clear on compare match
	TCCR1A = (1 << COM1A1) | (0 << COM1A0) | (0 << COM1B1) | (0 << COM1B0) | (0x0 << 2) | (1 << WGM11) | (0 << WGM10);
	// table 16-5 mode 14 for WGM bits (0x1D)
	TCCR1B = (0 << ICNC1) | (0 << ICES1) | (0x0 << 5) | (1 << WGM13) | (1 << WGM12) | (1 << CS12) | (0 << CS11) | (1 << CS10);
	// set ICR for (whatever) top  (ICR1 for timer1)
	// = 2x
	ICR1 = 0x4C4A;
	// set OCR1A for match, should equal top/2 for 50% duty cycle
	// set ICR1 and OCR1A 
	// ICR1 set to 2 * (20000000/1000) * (1/1024) * ms_delay
	//                   Hz        ms    prescaler
	// OCR1A is set to half of ICR2 for a 50% duty cycle
	OCR1A = ICR1/2;
	OCR1B = OCR1A;
	
	TIMSK1 |= (1 << OCIE1A) | (1 << TOIE1);
}

void initTimer3( )
{
	// setup of 16 bit timer for 100ms resolution
	TCCR3A = (1 << COM3A1) | (0 << COM3A0) | (0 << COM3B1) | (0 << COM3B0) | (0x0 << 2) | (0 << WGM31) | (0 << WGM30);
	// setup clock select table 15-9
	TCCR3B = (0 << WGM33) | (1 << WGM32) | (0 << CS32) | (1 << CS31) | (1 << CS30);
	// top for output compare register A
	OCR3A = 0x7A11;
	// enable interrupts
	TIMSK3 |= (1 << OCIE3A);
}

void initTimerCounters( uint8_t timer0, uint16_t timer1, uint16_t timer3 )
{
	TCNT0 = timer0;
	TCNT3 = timer3;
	TCNT1 = timer1;
}

void setupTimers( )
{
	initTimer0();
	initTimer1();
	initTimer3();
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