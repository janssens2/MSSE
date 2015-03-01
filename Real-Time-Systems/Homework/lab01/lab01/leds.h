/*
 * leds.h
 *
 * Created: 3/1/2015 2:33:23 PM
 *  Author: janssens
 */ 


#ifndef LEDS_H_
#define LEDS_H_

void toggle_green_iters( uint16_t ms_delay );
void turnoff_green( );
void enable_green( );
void toggle_green( );
void enable_red( );
void turnoff_red( );
void toggle_red( );
uint16_t toggle_generic_iters( uint16_t ms_delay, volatile uint16_t *colorToggle, const uint16_t colorTimer );
void toggle_red_iters( uint16_t ms_delay );
void enable_yellow( );
void turnoff_yellow( );
void toggle_yellow( );
void toggle_yellow_iters( uint16_t ms_delay );
void init_leds( );

#endif /* LEDS_H_ */