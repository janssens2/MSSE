/* SENG5831-Assignment1 - an application for the Pololu Orangutan SVP-1284
 *
 * This application uses the Pololu AVR C/C++ Library.  For help, see:
 * -User's guide: http://www.pololu.com/docs/0J20
 * -Command reference: http://www.pololu.com/docs/0J18
 *
 * Created: 1/26/2015 10:20:49 PM
 *  Author: janssens
 *
 * Description: This program will start out with a character scrolling
 *              across the screen.  This character indicates that the
 *              program is ready to be interacted with.  A press of the 
 *              top or bottom button will toggle the state of the LED.
 *              While a button is pressed text will be displayed saying
 *              what color is blinking.  Both LEDs will blink at their 
 *              own rates.  When no button is pressed it will continue 
 *              to scroll the character on the LCD display.
 *	
 */

#include <pololu/orangutan.h>
#include <stdio.h>

// index references for the led colors
#define RED 0
#define GREEN 1

int main()
{
	// setup the lcd and show the user the program version
	// helps guarantee a good load.
	lcd_init_printf();
	clear();
	print("Loaded v0.1");
	delay_ms(1000);

	// set the blink rate to a constant
	const uint32_t blinkrate = 500;

	// state variables using RED and GREEN
	// as the indexes
	// Note could not use red/green in an array
	// because data was getting stale.  volatile keyword
	// did not appear to resolve issue.
	uint8_t red = 0;
	uint8_t green = 0;
	uint8_t blinkState[2] = { 0, 0 };
	uint32_t blinkMSLast[2] = { 0, 0 };
	
	// coordinate timer and vars to keep track of where
	// lazy timer is printing things to the display
	uint32_t coordMSChange = 250;
	uint32_t coordMSTimer = 0;
	uint8_t x_coord = 0;
	uint8_t y_coord = 0;

	while(1)
	{		
		uint32_t ms = get_ms();	// get elapsed milliseconds
		
		// look for the button TOP or BOTTOM being pressed
		// only looking for an active press, code below will stop the 
		// blinking if the button is not pressed.
		unsigned char button = button_is_pressed(TOP_BUTTON|BOTTOM_BUTTON);

		// set the state of the LED blink to what button was pressed
		if ( button & BOTTOM_BUTTON )
		{
			blinkState[RED] = 1;
		}
		else if ( blinkState[RED] )
		{
			blinkState[RED] = 0;
		}

		if ( button & TOP_BUTTON )
		{
			blinkState[GREEN] = 1;
		}
		else if ( blinkState[GREEN] )
		{
			blinkState[GREEN] = 0;
		}

		// for red, we want to blink if we are within our window
		if ( blinkState[RED] && ((blinkMSLast[RED]+blinkrate) < ms) )
		{
			// toggle the LED since we don't care the previous state
			red_led(TOGGLE);
			blinkMSLast[RED] = ms;
			
			// print something to the screen and toggle it too
			// when the led is blinking
			lcd_goto_xy(0,1);
			char text[9];

			if ( red )
			{
				snprintf(text, sizeof(text)+1, "BLINK RED");
				red = 0;
			}
			else
			{
				snprintf(text, sizeof(text)+1, "BLINK    ");
				red = 1;
			}
			printf("%.9s",text);
		}

		// for green, we want to blink if we are withing our window
		if ( blinkState[GREEN] && ((blinkMSLast[GREEN]+blinkrate) < ms) )
		{
			// toggle the LED since we don't care about the previous state
			green_led(TOGGLE);
			blinkMSLast[GREEN] = ms;
			
			// print something to the screen and toggle it too
			// when the led is blinking
			lcd_goto_xy(0,0);
			char text[11];
			if ( green )
			{
				snprintf(text, sizeof(text)+1, "BLINK GREEN");
				green = 0;
			}
			else
			{
				snprintf(text, sizeof(text)+1, "BLINK      ");
				green = 1;
			}
			printf("%.11s", text);
		}
		
		// cleanup the printout to not print a blink text when one of the 
		// buttons is not pressed...
		if ( blinkState[GREEN] && !blinkState[RED] )
		{
			lcd_goto_xy(0,1);
			print("           ");
		}
		else if ( blinkState[RED] && !blinkState[GREEN] )
		{
			lcd_goto_xy(0,0);
			print("           ");
		}
		
		// create the appearance of something running, waiting for the user to 
		// do something
		if ( (coordMSTimer + coordMSChange) < ms && !blinkState[RED] && !blinkState[GREEN] )
		{
			// clear everything from the screen
			// should only be here if we are not attempting
			// to blink anything
			clear();
			lcd_goto_xy(x_coord,y_coord);
			printf(".");
			if ( x_coord == 15 )
			{
				y_coord = (y_coord < 1) ? y_coord + 1 : 0;	
			}
			
			x_coord = (x_coord < 15) ? x_coord + 1 : 0;
			coordMSTimer = ms;
		}
	} 
}