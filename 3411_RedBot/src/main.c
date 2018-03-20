/*
RedBot Automated Line Guidance Program
Authors:
	Jon Serio	jonathan.serio@uconn.edu
	Bill Brown	william.brown@uconn.edu
	
Functionality:
	Controls a robot.
*/

#define F_CPU			16000000UL

//Sensor pins
#define LEFT_SENSOR		0
#define CENTER_SENSOR	1
#define RIGHT_SENSOR	2

//Motor pins
#define LEFT_MOTOR		0
#define RIGHT_MOTOR		1

// Motor Operation Modes
#define	MODE_FORWARD	0
#define MODE_CW			1
#define MODE_BACKWARD	2
#define MODE_CCW		3

// Debug
#define ENABLED			0
#define DISABLED		1

#include <asf.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "uart.h"

#define START_SPEED		0.05

//UART stream
FILE uart_str = FDEV_SETUP_STREAM(uart_putchar, uart_getchar ,_FDEV_SETUP_RW);

//Global variables
//Sensors
volatile unsigned int left_sensor = 0;
volatile unsigned int center_sensor = 0;
volatile unsigned int right_sensor = 0;

//Motors
void init_all(void);
void ADC_init(void);
void timer0_init(void);
void timer2_init(void);

void ADC_clear(void);
int ADC_conv(void);
void sensor_select(int sens_mot_sel);
void motor_mode_select(int mode_select);

void debug_motor(int enabled);

void left_motor_en(int en);
void right_motor_en(int en);

/* 
Interrupt Service Routines
*/
ISR(TIMER0_COMPA_vect)
{
	left_motor_en(ENABLED);
}

ISR(TIMER0_COMPB_vect)
{
	left_motor_en(DISABLED);
}

ISR(TIMER2_COMPA_vect)
{
	right_motor_en(ENABLED);
}

ISR(TIMER2_COMPB_vect)
{
	right_motor_en(DISABLED);
}
void ADC_init(void)
{
	ADMUX |= (1<<REFS0);
	ADCSRA |= (1<<ADEN) | (1<<ADATE) | (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0); //Enable the ADC in Free-Running mode with a fadc = 125 kHz
}

void ADC_clear(void)
{
	ADMUX &= ~(1<<MUX3) & ~(1<<MUX2) & ~(1<<MUX1) & ~(1<<MUX0); //Clear ADC MUX
}

// Initialize TIMER0 in Fast PWM Mode
// PWM frequency set to match ADC sample rate
//		Frequency = Prescaler * (OCR0A+1) / Clock Speed
//		125kHz = 1 * OCR0A+1 / 16 MHz
//		OCR0A = 127
// Yields 125kHz frequency, 8us period
void timer0_init(void)
{
	TCCR0A |= (1<<WGM01) | (1<<WGM00); // Fast PWM Mode
	TCCR0B |= (1<<WGM02) | (1<<CS01);  // Fast PWM Mode, Prescaler off
	OCR0A = 127;  // Derived from equation for 8us period
	
	TIMSK0 |= (1<<OCIE0A)|(1<<OCIE0B); // Enable interrupts for both rising and falling edge
	
	// Define Initial PWM
	OCR0B = START_SPEED * OCR0A;
}

void timer2_init(void)
{
	TCCR2A |= (1<<WGM21) | (1<<WGM20);	// Fast PWM Mode
	TCCR2B |= (1<<WGM22) | (1<<CS21);	// Fast PWM Mode, Prescaler off
	OCR2A = 127;
	
	TIMSK2 |= (1<<OCIE2A)|(1<<OCIE2B);	// Enable interrupts for both rising and falling edge
	
	// Define Initial PWM
	OCR2B = OCR2A;
}
void init_all(void)
{
	board_init();
	ADC_clear();
	ADC_init();
	uart_init();
	
	// Timers
	timer0_init();
	timer2_init();

	sei();
	stdout = stdin = stderr = &uart_str;
	
	DDRD = 0b11111111;
	DDRB = 0b11111111;
	
	printf("Hello! \n\n");
}

int ADC_conv(void)
{
	ADCSRA |= (1<<ADSC);									//start conversion
	while(!(ADCSRA &(1<<ADSC)));							//wait for it to complete
	uint16_t sample_val_int = ADC;							//store the sample in a variable
	

	return sample_val_int;								//return as an int
}

/*
Sensor Motor Control

MUX ENCODING
	LEFT SENSOR		:	MUX1
	CENTER SENSOR	:	MUX2, MUX1
	RIGHT SENSOR	:	MUX1, MUX0
*/
void sensor_select(int sens_mot_sel)
{
	switch(sens_mot_sel)
	{
		case LEFT_SENSOR:
			ADC_clear();
			ADMUX |= (1<<MUX1);
			left_sensor = ADC_conv();
			printf("Left sensor: %d \n", left_sensor);
			break;
		
		case CENTER_SENSOR:
			ADC_clear();
			ADMUX |= (1<<MUX2) | (1<<MUX1);
			center_sensor = ADC_conv();
			printf("Center sensor: %d \n", center_sensor);
			break;
		
		case RIGHT_SENSOR:
			ADC_clear();
			ADMUX |= (1<<MUX1) | (1<<MUX0);
			right_sensor = ADC_conv();
			printf("Right sensor: %d \n", right_sensor);
			break;
		
		default:
			printf("ERROR: State ID %d not recognized \n", sens_mot_sel);
			break;
	}
}

/*
Motor Operating Mode Selector
-----------------------------
Determines the operating mode for the motor.
*/
void motor_mode_select(int mode_select)
{
	switch(mode_select)
	{
		case MODE_BACKWARD:
			PORTD |= (1<<PORTD4);		// Left Motor	: CCW	: IN1 L IN2 H
			PORTD |= (1<<PORTD7);		// Right Motor  : CW	: IN1 H IN2 L
			break;
		case MODE_CW:
			PORTD |= (1<<PORTD4);		// Left Motor	: CCW	: IN1 L IN2 H
			PORTB |= (1<<PORTB0);		// Right Motor	: CCW	: IN1 L	IN2 H
			break;
		case MODE_FORWARD:
			PORTD |= (1<<PORTD2);		// Left Motor	: CW	: IN1 H IN2 L
			PORTB |= (1<<PORTB0);		// Right Motor	: CCW	: IN1 L	IN2 H			
			break;
		case MODE_CCW:
			PORTD |= (1<<PORTD2);		// Left Motor	: CW	: IN1 H	IN2 L
			PORTD |= (1<<PORTD7);		// Right Motor	: CW	: IN1 H IN2 L
			break;
		default:
			printf("ERROR: Invalid Motor Mode %d. How did you do this.\n", mode_select);
			break;
	}
}

void left_motor_en(int en) 
{ 
	PORTD = (PORTD & ~(1<<PORTD5)) | ((1 & en)<<PORTD5);
}
void right_motor_en(int en)
{
	PORTD = (PORTD & ~(1<<PORTD6)) | ((1 & en)<<PORTD6);
}

// Enabled both motors at 100% duty cycle
void debug_motor(int enabled)
{
	switch(enabled)
	{
		case ENABLED:
			PORTD |= (1<<PORTD5);
			PORTD |= (1<<PORTD6);
			break;
		case DISABLED:
			PORTD &= ~(1<<PORTD5);
			PORTD &= ~(1<<PORTD6);
			break;
		default:
			PORTD |= (1<<PORTD5);
			PORTD |= (1<<PORTD6);
			break;
	}
}
int main (void)
{	
	init_all();
	
	int sens_mot_sel = 0;
	motor_mode_select(MODE_FORWARD);
	while(1)
	{
		sensor_select(sens_mot_sel);
		sens_mot_sel++;
		
		if(sens_mot_sel > 2)
		{
			printf("\n");
			sens_mot_sel = 0;
		}
		
		_delay_ms(300);
	}
}
