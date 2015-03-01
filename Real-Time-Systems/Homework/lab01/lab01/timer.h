/*
 * timer.h
 *
 * Created: 2/28/2015 4:58:59 PM
 *  Author: janssens
 */ 


#ifndef TIMER_H_
#define TIMER_H_

// following line is valid when __ii is not volatile
//#define FOR_COUNT_10MS 18065
// (23424 / 4) - 292
#define FOR_COUNT_10MS 5565
volatile uint32_t __ii;
#define WAIT_10MS {for(__ii=0; __ii<FOR_COUNT_10MS; __ii++);}

void initTimer0( );
void initTimer1( );
void initTimer3( );
void initTimerCounters( uint8_t timer0, uint16_t timer1, uint16_t timer3 );
void setupTimers( );

#endif /* TIMER_H_ */