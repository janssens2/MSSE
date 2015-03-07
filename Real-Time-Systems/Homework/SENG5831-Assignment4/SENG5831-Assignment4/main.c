/* SENG5831-Assignment4 - an application for the Pololu Orangutan SVP
 *
 * This application uses the Pololu AVR C/C++ Library.  For help, see:
 * -User's guide: http://www.pololu.com/docs/0J20
 * -Command reference: http://www.pololu.com/docs/0J18
 *
 * Created: 3/1/2015 4:55:40 PM
 *  Author: janssens
 */

#include <pololu/orangutan.h>
#include <stdio.h>

int main()
{
	// setup the lcd and show the user the program version
	// helps guarantee a good load.
	lcd_init_printf();
	clear();
	printf("Loaded v0.7");
	delay_ms(1000);
	clear();

	int16_t pot_max = 512;
	int16_t pot_mid = pot_max / 2;
	int16_t pot_min = 0;
	int16_t pot = 256;
	int16_t pot_target = pot;
	int16_t pot_change_interval = 8;
	
	uint32_t last_motor_change = 0;
	uint32_t current_ms = get_ms();
	int16_t motorSpeed = 0;

	//pulse_in_start( (unsigned char[]) {IO_D3, IO_D2}, 2 );
	encoders_init( IO_D2, IO_D3, IO_C4, IO_C5 );
	
	while(1)
	{
		current_ms = get_ms();
		
		// check for ANY_BUTTON to be pressed
		unsigned char button = get_single_debounced_button_press(ANY_BUTTON);
		if ( button & TOP_BUTTON )
		{
			pot_target = (pot_target + 32 <= pot_max) ? pot_target + 32 : pot_max;
		}
		if ( button & MIDDLE_BUTTON )
		{
			pot_target = (pot_target >= pot_mid) ? (pot_mid - (pot_target - pot_mid)) : ((pot_mid - pot_target) + pot_mid);
		}
		if ( button & BOTTOM_BUTTON )
		{
			pot_target = (pot_target - 32 >= pot_min) ? pot_target - 32 : pot_min;
		}
		
		if ( ((current_ms - last_motor_change) % 100) == 0 )
		{
			if ( pot_target < (pot_min + pot_change_interval) && pot < (pot_min + pot_change_interval) )
			{
				pot = pot_min;
			}
			else if ( pot_target > (pot_max - pot_change_interval) && pot > (pot_max - pot_change_interval) )
			{
				pot = pot_max;
			}
			else if ( pot_target > pot && (pot_target - pot) >= pot_change_interval )
			{
				pot += pot_change_interval;
			}
			else if ( pot_target < pot && (pot_target + pot) >= pot_change_interval )
			{
				pot -= pot_change_interval;
			}
			else
			{
				// will get here when pot_target = pot
			}
			motorSpeed = (pot_mid - pot);
			// avoid clearing the LCD to reduce flicker
			lcd_goto_xy(0, 0);
			print("encoder=");
			print_long(encoders_get_counts_m1());               // print the trim pot position (0 - 1023)
			print("  ");              // overwrite any left over digits
			lcd_goto_xy(0, 1);
			print("spd=");
			print_long(motorSpeed);        // print the resulting motor speed (-255 - 255)
			print("   ");
			set_m1_speed(motorSpeed);
			
			// all LEDs off
			red_led(0);
			green_led(0);
			// turn green LED on when motors are spinning forward
			if (motorSpeed > 0)
			green_led(1);
			// turn red LED on when motors are spinning in reverse
			if (motorSpeed < 0)
			red_led(1);
		}
	}
}
