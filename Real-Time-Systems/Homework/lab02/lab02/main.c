/* lab02 - an application for the Pololu Orangutan SVP
 *
 * This application uses the Pololu AVR C/C++ Library.  For help, see:
 * -User's guide: http://www.pololu.com/docs/0J20
 * -Command reference: http://www.pololu.com/docs/0J18
 *
 * Created: 3/29/2015 5:00:19 PM
 *  Author: janssens
 */

#include <pololu/orangutan.h>
#include <stdio.h>
#include <string.h>

volatile unsigned char gLastM1A_val = 0;
volatile unsigned char gLastM1B_val = 0;
volatile unsigned char gErrorM1     = 0;
volatile int32_t gEncoderCounts = 0;
volatile int32_t gPrvEncoderCounts = 0;

volatile uint32_t gT0Count = 0;
volatile uint32_t gT3Count = 0;

#define COUNT_SIZE 10

#define ASSIGNMENTQ2Q5
// run PID controller every n TIMER0 hits i.e. TIMER0 is 10ms, so this runs every 20ms if a 2
#ifdef ASSIGNMENTQ2Q5
	#define PID_RUN 5	
#else
	#define PID_RUN 1
#endif

volatile int32_t gT0AverageSpeed      = 0;
volatile int32_t gT0Speed[COUNT_SIZE] = {0};
volatile uint8_t gT0SpeedIdx          = 0;

volatile uint8_t gReleasePID = 0;
volatile uint8_t gReleaseLCD = 0;
volatile uint8_t gReleaseTrajectory = 0;

uint8_t gTrajectoryTime = 0;

int32_t gSpeedTarget = 0;
uint8_t toLog = 0x0;
uint8_t toDebug = 0x0;

void serial_print( char *format, ... );

ISR(PCINT3_vect)
{
	// ISR method to calculate the number of encoder counts based
	// on direction
	
	// a 1 or 0 allows us to get to 48 encoder checks for a single
	// revolution of the encoder magnet.
	unsigned char m1a_val = (PIND >> IO_D3) & 0x1;
	
	// a 1 or 0 allows us to get to 48 encoder checks for a single
	// revolution of the encoder magnet.
	unsigned char m1b_val = (PIND >> IO_D2) & 0x1;
	
	unsigned char plus_m1 = m1a_val ^ gLastM1B_val;
	unsigned char minus_m1 = m1b_val ^ gLastM1A_val;
	
	if ( plus_m1 )
	{
		gEncoderCounts += 1;
	}
	if ( minus_m1 )
	{
		gEncoderCounts -= 1;
	}
	
	if ( m1a_val != gLastM1A_val && m1b_val != gLastM1B_val )
	{
		gErrorM1 = 1;
	}
	
	gLastM1A_val = m1a_val;
	gLastM1B_val = m1b_val;
}

ISR(TIMER0_COMPA_vect)
{
	gT0Count++;
	
	//if ( gEncoderCounts != gPrvEncoderCounts )
	//{
		if ( gT0SpeedIdx >= COUNT_SIZE )
		{
			gT0SpeedIdx = 0;
		}
		gT0Speed[gT0SpeedIdx] = (gEncoderCounts - gPrvEncoderCounts);
		
		gT0SpeedIdx++;
		//gT0Speed = (gEncoderCounts - gPrvEncoderCounts) * 1000; // convert to seconds
	//}
	//else if ( gEncoderCounts <= gPrvEncoderCounts )
	//{
		//gT0Speed = (gPrvEncoderCounts - gEncoderCounts) * 100; // convert to seconds
	//}
	//else
	//{
		//gT0Speed = 0;
	//}
	gPrvEncoderCounts = gEncoderCounts;
	
	double tmpAverageSpeed = 0.0;
	for ( uint8_t idx=0; idx < COUNT_SIZE; idx++ )
	{
		tmpAverageSpeed += gT0Speed[idx];
	}
	tmpAverageSpeed = (tmpAverageSpeed / ((COUNT_SIZE > gT0Count) ? gT0Count : COUNT_SIZE)) * 100;
	gT0AverageSpeed = (int32_t) tmpAverageSpeed;
	
	if ( gT0Count % PID_RUN == 0 )
	{
		gReleasePID = 1;
	}
	
	gReleaseTrajectory = 1;
}

ISR(TIMER3_COMPA_vect)
{
	gT3Count++;
	
	gReleaseLCD = 1;
}

// motor stuff
int16_t gCurrentTorque = 0;

enum motor_mode_t {
	MOTOR_MODE_SPEED,
	MOTOR_MODE_POSITION
};

typedef struct
{	
	double dState;
	double iState;
	double iMax;
	double iMin;
	
	double iGain; // integral gain
	double pGain; // proportional gain
	double dGain; // derivative gain
	
	enum motor_mode_t myMode;
	int32_t targetRef;
	int32_t currentRef;
} SPid;

SPid *myPid;

enum interpolator_action_t
{
	INTERPOLATOR_SLEEP = 0,
	INTERPOLATOR_PIDUPDATE = 1,
	INTERPOLATOR_QUANTITY = 2
};

typedef struct  
{
	enum interpolator_action_t myAction;
	int32_t actionValue;
	int32_t actionStatus;
	uint8_t actionSet;
} Interpolator;

#define MOTOR_NUM_COUNT_360 2249
#define INTERPOLATOR_ACTIONS 5
Interpolator *myInterpolator[INTERPOLATOR_ACTIONS];

void setupMotor( );
void setMyMotorSpeed( int16_t speed );
int16_t getMyMotorTorque( );
void setupInterpolatorStructure( );

void setupInterpolatorStructure( )
{
	Interpolator a;
	a.myAction = INTERPOLATOR_PIDUPDATE;
	a.actionValue = (MOTOR_NUM_COUNT_360 * ( 0.25 ));
	a.actionStatus = 0;
	a.actionSet = 0;
	myInterpolator[0] = calloc( 1, sizeof(Interpolator));
	memcpy( myInterpolator[0], &a, sizeof(Interpolator));
	if ( toDebug > 1) serial_print( "(0) act=%d val=%ld sta=%ld", myInterpolator[0]->myAction, myInterpolator[0]->actionValue, myInterpolator[0]->actionStatus );

	a.myAction = INTERPOLATOR_SLEEP;
	a.actionValue = 500;
	a.actionStatus = 0;
	a.actionSet = 0;
	myInterpolator[1] = calloc( 1, sizeof(Interpolator));
	memcpy( myInterpolator[1], &a, sizeof(Interpolator));
	if ( toDebug > 1) serial_print( "(1) act=%d val=%ld sta=%ld", myInterpolator[1]->myAction, myInterpolator[1]->actionValue, myInterpolator[1]->actionStatus );
	
	a.myAction = INTERPOLATOR_PIDUPDATE;
	a.actionValue = (MOTOR_NUM_COUNT_360 * ( -1 ));
	a.actionStatus = 0;
	a.actionSet = 0;
	myInterpolator[2] = calloc( 1, sizeof(Interpolator));
	memcpy(myInterpolator[2], &a, sizeof(Interpolator));
	if ( toDebug > 1) serial_print( "(2) act=%d val=%ld sta=%ld", myInterpolator[2]->myAction, myInterpolator[2]->actionValue, myInterpolator[2]->actionStatus );
	
	a.myAction = INTERPOLATOR_SLEEP;
	a.actionValue = 500;
	a.actionStatus = 0;
	a.actionSet = 0;
	myInterpolator[3] = calloc( 1, sizeof(Interpolator));
	memcpy(myInterpolator[3], &a, sizeof(Interpolator));
	if ( toDebug > 1) serial_print( "(3) act=%d val=%ld sta=%ld", myInterpolator[3]->myAction, myInterpolator[3]->actionValue, myInterpolator[3]->actionStatus );
	
	a.myAction = INTERPOLATOR_PIDUPDATE;
	a.actionValue = (MOTOR_NUM_COUNT_360 * ( 0.014 ));
	a.actionStatus = 0;
	a.actionSet = 0;
	myInterpolator[4] = calloc( 1, sizeof(Interpolator));
	memcpy(myInterpolator[4], &a, sizeof(Interpolator));
	if ( toDebug > 1) serial_print( "(4) act=%d val=%ld sta=%ld", myInterpolator[4]->myAction, myInterpolator[4]->actionValue, myInterpolator[4]->actionStatus );
}

void dumpInterpolatorStructure( )
{
	char tmpBuff[79];
	serial_print( "Interpolator Structure Dump" );
	for ( uint8_t cnt=0; cnt < INTERPOLATOR_ACTIONS; cnt++ )
	{
		memset( tmpBuff, 0, 79 );
		if ( myInterpolator[cnt]->myAction == INTERPOLATOR_PIDUPDATE )
		{
			sprintf( tmpBuff, "(%d) - action=%s", cnt, "PIDUPDATE" );
		}
		else
		{
			sprintf( tmpBuff, "(%d) - action=%s", cnt, "SLEEP" );
		}
		sprintf( tmpBuff, "%s actionVal=%ld actionSta=%ld", tmpBuff, myInterpolator[cnt]->actionValue, myInterpolator[cnt]->actionStatus );
		serial_print_string( tmpBuff );			
	}
}

void setupMotor( )
{
	// set port to 0 for both PWM
	PORTD &= ~(1 << (PORTD6));
	//PORTD &= ~(1 << (PORTD7));
	
	PORTC &= ~(1 << (PORTC6));
	//PORTC &= ~(1 << (PORTC7));
	
	// set data direction to 0 for both PWM
	DDRD |= (1 << DDD6);
	//DDRD |= (1 << DDD7);
	
	DDRC |= (1 << DDC6);
	//DDRC |= (1 << DDC7);
	
	setMyMotorSpeed( 0 );
}

void setupMotorPinChange( )
{
	// IO_D2 and IO_D3 are where the motor is connected for pin change interrupts
	// This equates to PCINT26 and PCINT27 which are labeled D2/D3 on the board
	PCMSK3 |= (1 << IO_D2) | (1 << IO_D3);  //0x0c
	// PCIE3 shifted into the pin change interrupt control register
	// Enables pcint pins 31:24 which are set via the PCMSK3 register
	PCICR  |= (1 << PCIE3);					//0x08
}

int16_t getMyMotorTorque( )
{
	return gCurrentTorque;
}

void setMyMotorSpeed( int16_t speed )
{
	// determine if we are in reverse mode or not
	uint8_t reverse = ( speed >= 0 ) ? 0 : 1;
	
	// get a positive value of speed
	speed = abs( speed );
	
	// limit the max of speed to 0xFF
	speed = ( speed > 0xFF ) ? 0xFF : speed;

	gCurrentTorque = (reverse) ? speed * -1 : speed;
	
	OCR2B = speed;
	
	if ( speed == 0 )
	{
		// 0% duty cycle
		TCCR2A &= ~( 1 << COM2B1 );
	}
	else
	{
		TCCR2A |= ( 1 << COM2B1 );
		
		if ( reverse )
		{
			PORTC |= (1 << (PORTC6));
		}
		else
		{
			PORTC &= ~(1 << (PORTC6));
		}
	}
}

int16_t updatePID( SPid *pid, int32_t error, int32_t position )
{
	double pTerm = 0.0;
	double dTerm = 0.0;
	double iTerm = 0.0;
	
	if ( toDebug > 2 )
	{
		serial_print( "args err=%.1f position=%.1f iState=%.1f dState=%.1f", error, position, pid->iState, pid->dState );
	}
	
	pTerm = pid->pGain * error; // calculate proportional term

	pid->iState += error; // calculate integral state
	if ( pid->iState > pid->iMax )
	{
		pid->iState = pid->iMax;
	}
	else if ( pid->iState < pid->iMin )
	{
		pid->iState = pid->iMin;
	}
	
	iTerm = pid->iGain * pid->iState; // calculate the integral term
	
	//if ( pid->dState != 0.0 )
	//{
		dTerm = pid->dGain * (pid->dState - position);
	//}
	pid->dState = position;
	
	if ( toDebug > 2 )
	{
		serial_print( "p=%.1f d=%.1f i=%.1f", pTerm, dTerm, iTerm);
	}
	
	return (int16_t) (pTerm + dTerm + iTerm);
}

// timer stuff
void initTimer0( );
void initTimer1( );
void initTimer2( );
void initTimer3( );
void initTimerCounters( uint8_t timer0, uint16_t timer1, uint16_t timer3 );
void setupTimers( );

void initTimer0( )
{
	// setup of 8 bit timer to a 10ms resolution
	// COM0A1/COM0A0 set to 10=Clear OC0A on compare match (this is probably not necessary) in non-PWM mode table 15-2
	// COM0B1/COM0B0 not necessary to set table 15-5
	// WGM01/WGM00 set to 10= since we are using mode 2 CTC mode table 15-8
	TCCR0A = (1 << COM0A1) | (0 << COM0A0) | (0 << COM0B1) | (0 << COM0B0) | (0x0 << 2) | (1 << WGM01) | (0 << WGM00);
	// setup clock select table 15-9
	// WGM02 set to 0 since we are using mode 2 CTC mode table 15-8 (other WGM bits set in TCCR0A)
	// CS02/CS01/CS00 bits set to 101=clk io / 1024 prescalar table 15-9
	TCCR0B = (0 << WGM02) | (1 << CS02) | (0 << CS01) | (1 << CS00);
	// top for output compare register A
	// 20000(ms) / 1024(prescalar)(TCCR0B CS fields) = 78 (0x4E)
	// (20000ms / 1024 * (0xc2 + 1)) = 1.014ms
	// 8-bit value when to generate an output compare interrupt
	OCR0A = 0xc2; // 10ms
	//OCR0A = 0x13; // 1ms  for testing purposes
	// enable interrupts for Timer0 output compare match A
	TIMSK0 |= (1 << OCIE0A);
}

//void initTimer1( )
//{
//// setup timer for PWM
//// clock select bits to control frequency table 16.3 COM1A, clear on compare match
//TCCR1A = (1 << COM1A1) | (0 << COM1A0) | (0 << COM1B1) | (0 << COM1B0) | (0x0 << 2) | (1 << WGM11) | (0 << WGM10);
//// table 16-5 mode 14 for WGM bits (0x1D)
//TCCR1B = (0 << ICNC1) | (0 << ICES1) | (0x0 << 5) | (1 << WGM13) | (1 << WGM12) | (1 << CS12) | (0 << CS11) | (1 << CS10);
//// set ICR for (whatever) top  (ICR1 for timer1)
//// = 2x
//ICR1 = 0x4C4A;
//// set OCR1A for match, should equal top/2 for 50% duty cycle
//// set ICR1 and OCR1A
//// ICR1 set to 2 * (20000000/1000) * (1/1024) * ms_delay
////                   Hz        ms    prescaler
//// OCR1A is set to half of ICR2 for a 50% duty cycle
//OCR1A = ICR1/2;
//OCR1B = OCR1A;
//
//TIMSK1 |= (1 << OCIE1A) | (1 << TOIE1);
//}

void initTimer2( )
{
	// setup of 8 bit timer
	// COM2A = 0b00 Set OC2A on Compare Match, clear OC2A at BOTTOM (inverting mode) (table 17-3)
	// COM2B = 0b00 Set OC2B on Compare Match, clear OC2B at BOTTOM (inverting mode) (table 17-6)
	// WGM   = 0b011 Set Fast PWM mode (table 17-8)
	// CS    = 0b010 Prescaler of 8 (table 17-9)
	TCCR2A = (0 << COM2A1) | (0 << COM2A0) | (0 << COM2B1) | (0 << COM2B0) | (0x0 << 2) | (1 << WGM21) | (1 << WGM20);
	TCCR2B = (0 << WGM22) | (0 << CS22) | (1 << CS21) | (0 << CS20);
	
	// Initialize all PWMs to 0% duty cycle (braking)
	OCR2A = OCR2B = 0x00;
	
	// enable interrupts, not necessary with PWM... unless we want to see them too
	//TIMSK2 |= (1 << OCIE2B);
}

void initTimer3( )
{
	// setup of 16 bit timer for 100ms resolution
	TCCR3A = (1 << COM3A1) | (0 << COM3A0) | (0 << COM3B1) | (0 << COM3B0) | (0x0 << 2) | (0 << WGM31) | (0 << WGM30);
	// setup clock select table 15-9, prescaler of 1024
	TCCR3B = (0 << WGM33) | (1 << WGM32) | (1 << CS32) | (0 << CS31) | (1 << CS30);
	// top for output compare register A
	OCR3A = 0x07A0;
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
	initTimer2();
	initTimer3();
}

// serial stuff
#define BUFFER_SIZE 81
#define SERIAL_SEND_TIMEOUT 100

void wait_for_sending_to_finish();
void print_menu( );
void process_received_bytes( char bytes[] );
void strip( char bytes[], size_t mySize );
void check_for_new_bytes_received( );
void serial_print_string( const char myString[] );
//void serial_print( char *format, ... );
void serial_print_char( const char myChar );
void serial_init( );

char *myLogger[100];
uint8_t myLoggerCnt = 0;

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
	serial_print( "Menu: <operation> [value]" );
	serial_print( "   Operation" );
	serial_print( "     l: Start/Stop Logging" );
	serial_print( "     v: View current values" );
	serial_print( "     r: Set the reference position (counts)" );
	serial_print( "     s: Set the reference speed (counts/sec)" );
	serial_print( "     P/p: Increase/Decrease Kp by value/1000" );
	serial_print( "     D/d: Increase/Decrease Kd by value/1000" );
	serial_print( "     I/i: Increase/Decrease Ki by value/100000" );
	serial_print( "     t: Execute trajectory" );
	serial_print( "   Value" );
	serial_print( "     Any integer value to set to, Kp/Kd will be converted to double with / 1000" );
	serial_print( "   ? : To view this help again" );
	serial_print( "   Examples" );
	serial_print( "     P 2   - Increase Kp by .002" );
	serial_print( "     p 10  - Decrease Kp by .010" );
	serial_print( "     L     - Toggle whether to log or not" );
}

void process_received_bytes( char bytes[] )
{
	char operation;
	int32_t value = 0;
	
	// may need this -Wl,-u,vfscanf -lscanf_flt for float reads...
	int tmp = sscanf( bytes, "%c %ld", &operation, &value);

	serial_print( "(%d) OP:%c VA:%ld", tmp, operation, value );
	
	if ( gTrajectoryTime )
	{
		serial_print( "input ignored until Trajectory is done running" );
		return;
	}

	switch(operation)
	{
		//
		case '?':
		// print the menu again
		print_menu();
		break;

		//
		case 'L':
		case 'l':
		// toggle the logging flag
		if ( toLog ) 
		{
			toLog = 0;
			if ( toDebug ) serial_print( "Logging disabled" );
		}
		else
		{
			toLog = 1;
			if ( toDebug ) serial_print( "Logging enabled" );
		}
		break;
		
		case '0':
		// turn off debug
		toDebug = 0;
		serial_print( "Debug disabled" );
		break;
		
		case '1':
		// toggle the debug flag
		toDebug = 1;
		serial_print( "Debug enabled lvl1" );
		break;
		
		case '2':
		toDebug = 2;
		serial_print( "Debug enabled lvl2" );
		break;
		
		case '3':
		toDebug = 3;
		serial_print( "Debug enabled lvl3" );
		break;
		
		case 'V':
		case 'v':
		// view the current values of Kd, Kp, Vm, Pr, Pm, and T
		serial_print( "Kd=%.3f Kp=%.3f Ki=%.5f Pr=%.4ld Pm=%.4ld T=%.3d", myPid->dGain,
		                                                                  myPid->pGain,
																		  myPid->iGain,
																		  myPid->targetRef,
																		  gT0AverageSpeed,
																		  getMyMotorTorque());
		break;
		
		case 't':
		// Execute the trajectory
		gTrajectoryTime = 1;
		// trajectory also needs reference position settings so set those too
		
		case 'R':
		case 'r':
		// set the reference position by value
		if ( myPid->myMode != MOTOR_MODE_POSITION )
		{
			myPid->iMax = 500.0;
			myPid->iMin = -500.0;
			myPid->pGain = 0.100;
			myPid->iGain = 0.01300;
			myPid->dGain = 1.000;
		}
		myPid->myMode = MOTOR_MODE_POSITION;
		myPid->targetRef = value;
		if ( toDebug > 1 ) serial_print("Target updated to %d from %ld", myPid->targetRef, value);
		break;
		
		case 'S':
		case 's':
		// set the reference speed by value per sec
		if ( myPid->myMode != MOTOR_MODE_SPEED )
		{
			myPid->iMax = 100000.0;
			myPid->iMin = -100000.0;
			myPid->pGain = 0.001;
			myPid->iGain = 0.00001;
			myPid->dGain = 0.001;
		}
		myPid->myMode = MOTOR_MODE_SPEED;
		myPid->targetRef = value;
		if ( toDebug > 1 ) serial_print("Target updated to %d from %ld", myPid->targetRef, value);
		break;
		
		case 'P':
		// Increase Kp by value
		myPid->pGain += ((double) value / 1000);
		if ( toDebug > 1 ) serial_print( "pGain set to %.3f", myPid->pGain );
		break;
		
		case 'p':
		// Decrease Kp by value
		myPid->pGain -= ((double) value / 1000);
		if ( myPid->pGain < 0.0 )
		{
			myPid->pGain = 0.0;
		}
		if ( toDebug > 1 ) serial_print( "pGain set to %.3f", myPid->pGain );
		break;
		
		case 'D':
		// Increase Kd by value
		myPid->dGain += ((double) value / 1000);
		if ( toDebug > 1 ) serial_print( "dGain set to %.3f", myPid->dGain );
		break;
		
		case 'd':
		// Decrease Kd by value
		myPid->dGain -= ((double) value / 1000);
		if ( myPid->dGain < 0.0 )
		{
			myPid->dGain = 0.0;
		}
		if ( toDebug > 1 ) serial_print( "dGain set to %.3f", myPid->dGain );
		break;
		
		case 'I':
		// Increase Ki by value
		myPid->iGain += ((double) value / 100000);
		if ( toDebug > 1 ) serial_print( "iGain set to %.5f", myPid->iGain );
		break;
		
		case 'i':
		// Decrease Ki by value
		myPid->iGain -= ((double) value / 100000);
		if ( myPid->iGain < 0.0 )
		{
			myPid->iGain = 0.0;
		}
		if ( toDebug > 1 ) serial_print( "iGain set to %.5f", myPid->iGain );
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
		if ( bytes[counter] != 32 && bytes[counter] != 45 &&
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
		
		//clear();
		//lcd_goto_xy(0,0);
		//printf("t: %s", my_received_buffer );
		
		memset( my_received_buffer, 0, sizeof(my_received_buffer) );
		bytes_received = 0;
	}
}

void serial_print_string( const char myString[] )
{
	uint32_t parmSize = strlen(myString);
	parmSize = (BUFFER_SIZE - 2 > parmSize) ? parmSize+2 : BUFFER_SIZE;
	
	wait_for_sending_to_finish();
	memset( send_buffer, 0, BUFFER_SIZE );
	strncpy( send_buffer, myString, parmSize );
	send_buffer[parmSize-2] = '\r';
	send_buffer[parmSize-1] = '\n';
	serial_send( USB_COMM, send_buffer, BUFFER_SIZE );
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
	lcd_init_printf();
	clear();
	printf("Loaded v0.7");
	delay_ms(1000);
	clear();
	
	serial_init();
	
	setupTimers();
	
	setupMotor();
	
	setupMotorPinChange();
	
	setupInterpolatorStructure();
	dumpInterpolatorStructure();
	uint8_t trajectoryidx = 0;
	
	myPid = calloc(1, sizeof( SPid ));
	
	myPid->iMax = 100000.0;
	myPid->iMin = -100000.0;
	// .05 causes motor to jerk and move to negative speed/positive speed too fast.
	myPid->pGain = 0.001;
	myPid->iGain = 0.00001;
	myPid->dGain = 0.001;
	myPid->targetRef = 0;
	myPid->myMode = MOTOR_MODE_SPEED;
	
	uint32_t current_ms = get_ms();

	sei();
	
	int32_t myError = 0;
	
	while(1)
	{
		current_ms = get_ms();
		
		serial_check();
		
		check_for_new_bytes_received();
		
		if ( gReleasePID )
		{
			if ( myPid->myMode == MOTOR_MODE_SPEED )
			{
				myPid->currentRef = gT0AverageSpeed;
				if ( toDebug >1 )
					serial_print( "set currentRef (spd) to %ld (%ld)", myPid->currentRef, myPid->targetRef );
			}
			else if ( myPid->myMode == MOTOR_MODE_POSITION )
			{
				myPid->currentRef = gEncoderCounts;
				if ( toDebug >1 )
					serial_print( "set currentRef (pos) to %ld (%ld)", myPid->currentRef, myPid->targetRef );
			}
			else
			{
				serial_print_string( "should never get here!" );
			}
			myError = myPid->targetRef - myPid->currentRef;
			int16_t myNewSpeed = updatePID( myPid, myError, myPid->currentRef );
			myNewSpeed = ( myPid->myMode == MOTOR_MODE_SPEED ) ? myNewSpeed + getMyMotorTorque() : myNewSpeed;
			setMyMotorSpeed( myNewSpeed );
			if ( toLog )
				serial_print( "%.5ld %.5ld %.3d %.5ld %.3f %.5f %.3f", myPid->currentRef % 100000, 
																	   gT0AverageSpeed % 100000,
																	   getMyMotorTorque() % 1000, 
																	   myPid->targetRef % 100000, 
																	   myPid->pGain,
																	   myPid->iGain,
																	   myPid->dGain);
			if ( toDebug > 2 )
			{
				serial_print( "Pm=%.5ld T=%.3d Pr=%.5ld Kp=%.3f Ki=%.5f Kd=%.3f", myPid->currentRef % 100000,
																				  getMyMotorTorque() % 1000,
																				  myPid->targetRef % 100000,
																				  myPid->pGain,
																				  myPid->iGain,
																				  myPid->dGain);
				serial_print( "New=%.3d encoder=%.9ld err=%.5ld spd=%.5ld", myNewSpeed % 1000, 
																		   gEncoderCounts % 1000000000, 
																		   myError,
																		   gT0AverageSpeed % 100000 );
			}
			gReleasePID = 0;
		}
		
		if ( gReleaseLCD )
		{
			lcd_goto_xy(0,0);
			printf( "enc=%.9ld ", gEncoderCounts % 1000000000);
			lcd_goto_xy(0,1);
			printf( "t=%.3d e=%.4ld ", getMyMotorTorque() % 1000, myError % 10000 );
			gReleaseLCD = 0;
		}
		
		// released every 10 ms per TIMER0 settings
		if ( gReleaseTrajectory )
		{
			if ( gTrajectoryTime && (trajectoryidx < INTERPOLATOR_ACTIONS) )
			{
				// trajectoryidx lets us know where we currently are
				if ( myInterpolator[trajectoryidx]->myAction == INTERPOLATOR_SLEEP )
				{
					// keeping track of ms slept via timer... add it to our status
					if ( myInterpolator[trajectoryidx]->actionSet == 0 )
					{
						myInterpolator[trajectoryidx]->actionSet = 1;
					}
					myInterpolator[trajectoryidx]->actionStatus += 10;
					if ( toDebug > 1 )
						serial_print( "sleep (%ld) (%ld)", myInterpolator[trajectoryidx]->actionStatus, myInterpolator[trajectoryidx]->actionValue);
					if ( myInterpolator[trajectoryidx]->actionStatus >= myInterpolator[trajectoryidx]->actionValue )
					{
						// go to the next trajectory index
						// we met our object for this item
						trajectoryidx+=1;
					}
				}
				else if ( myInterpolator[trajectoryidx]->myAction == INTERPOLATOR_PIDUPDATE )
				{
					// need to wait in this method until we get close to our target
					// leaving a stddev of +/- 10 here
					if ( toDebug > 1 )
						serial_print( "enc=%ld act=%ld", gEncoderCounts, myInterpolator[trajectoryidx]->actionValue );
					if ( abs(gEncoderCounts - myInterpolator[trajectoryidx]->actionValue) <= 10 &&
						 gT0AverageSpeed == 0 )
					{
						myInterpolator[trajectoryidx]->actionStatus++;
					}
					
					if ( myInterpolator[trajectoryidx]->actionSet == 0 )
					{
						// here we need to be setting out target for the PID to pick it up.
						myPid->targetRef = gEncoderCounts + myInterpolator[trajectoryidx]->actionValue;
						// copy this value into the actionValue so we know what we're going to
						myInterpolator[trajectoryidx]->actionValue = myPid->targetRef;
						myInterpolator[trajectoryidx]->actionSet = 1;
						if ( toDebug > 0 ) serial_print( "PIDUPDATE action=%d", myInterpolator[trajectoryidx]->actionSet);
					}
					
					if ( myInterpolator[trajectoryidx]->actionStatus >= 10 )
					{
						// increment our trajectory index to move on to the next thing
						// after hitting our goal for 100ms... can we make this faster?
						trajectoryidx++;
					}
				}
				else
				{
					// whoops!!!!
				}
			}
			else if ( gTrajectoryTime )
			{
				// only enter here if we are doing trajectory and our idx is more than what we should do.
				serial_print( "Trajectory is done!" );
				trajectoryidx = 0;
				gTrajectoryTime = 0;
			}
			gReleaseTrajectory = 0;
		}
	}
}