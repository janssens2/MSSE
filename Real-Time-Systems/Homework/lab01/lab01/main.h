/*
 * main.h
 *
 * Created: 2/28/2015 11:13:52 AM
 *  Author: janssens
 */ 


#ifndef MAIN_H_
#define MAIN_H_

void findFOR_COUNT_10MS( );
void toggle_green_iters( uint16_t ms_delay );
void toggle_red( );
uint16_t toggle_generic_iters( uint16_t ms_delay, volatile uint16_t *colorToggle, const uint16_t colorTimer );
void toggle_red_iters( uint16_t ms_delay );
void toggle_yellow( );
void toggle_yellow_iters( uint16_t ms_delay );
void wait_for_sending_to_finish();
void print_menu( );
void process_received_bytes( char bytes[] );
void strip( char bytes[], size_t mySize );
void check_for_new_bytes_received( );
void serial_print_string( const char myString[] );
void serial_print( char *format, ... );
void serial_print_char( const char myChar );
void serial_init( );


#endif /* MAIN_H_ */