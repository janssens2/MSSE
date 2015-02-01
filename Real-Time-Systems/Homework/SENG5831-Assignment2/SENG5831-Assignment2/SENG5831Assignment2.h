/*
 * SENG5831ASSIGNMENT2.h
 *
 * Created: 1/30/2015 12:22:57 PM
 *  Author: janssens
 */ 


#ifndef SENG5831ASSIGNMENT2_H_
#define SENG5831ASSIGNMENT2_H_

//#define DDRLOC 0
#define DDLOC 0
//#define PORTLOC 0
#define PORTBIT 1

void led_on( volatile uint8_t *ddr, volatile uint8_t *port, uint8_t bit );
void led_off( volatile uint8_t *ddr, volatile uint8_t *port, uint8_t bit );
void led_toggle( volatile uint8_t *port, uint8_t bit );
void led_initialize( volatile uint8_t *ddr, uint8_t numArgs, ... );
void modify_interval( uint32_t * led_intervals, const uint8_t leds, int8_t change );

// Note the shell of buffers, position, and functions wait_for_sending_to_finish and process_received_byte 
// modified from example programs from libpolulu-avr

// receive_buffer: A ring buffer that we will use to receive bytes on USB_COMM.
// The OrangutanSerial library will put received bytes in to
// the buffer starting at the beginning (receiveBuffer[0]).
// After the buffer has been filled, the library will automatically
// start over at the beginning.
char receive_buffer[32];

// receive_buffer_position: This variable will keep track of which bytes in the receive buffer
// we have already processed.  It is the offset (0-31) of the next byte
// in the buffer to process.
unsigned char receive_buffer_position = 0;

// send_buffer: A buffer for sending bytes on USB_COMM.
char send_buffer[32];

// wait_for_sending_to_finish:  Waits for the bytes in the send buffer to
// finish transmitting on USB_COMM.  We must call this before modifying
// send_buffer or trying to send more bytes, because otherwise we could
// corrupt an existing transmission.
void wait_for_sending_to_finish();

// process_received_byte: Responds to a byte that has been received on
// USB_COMM.  If you are writing your own serial program, you can
// replace all the code in this function with your own custom behaviors.
void process_received_byte(char byte);

void check_for_new_bytes_received();

void process_send_bytes( const char *myString );

#endif /* SENG5831Assignment2_H_ */