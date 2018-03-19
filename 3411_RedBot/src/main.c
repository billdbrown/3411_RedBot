/**
 * \file
 *
 * \brief Empty user application template
 *
 */

/**
 * \mainpage User Application template doxygen documentation
 *
 * \par Empty user application template
 *
 * Bare minimum empty user application template
 *
 * \par Content
 *
 * -# Include the ASF header files (through asf.h)
 * -# "Insert system clock initialization code here" comment
 * -# Minimal main function that starts with a call to board_init()
 * -# "Insert application code here" comment
 *
 */

/*
 * Include header files for all drivers that have been imported from
 * Atmel Software Framework (ASF).
 */
/*
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */
#define F_CPU 16000000UL

//Sensor pins
#define LEFT_SENSOR 0
#define CENTER_SENSOR 1
#define RIGHT_SENSOR 2

//Motor pins
#define LEFT_MOTOR 3
#define RIGHT_MOTOR 4

#include <asf.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "uart.h"

//UART stream
FILE uart_str = FDEV_SETUP_STREAM(uart_putchar, uart_getchar ,_FDEV_SETUP_RW);

//Global variables
//Sensors
volatile unsigned int left_sensor = 0;
volatile unsigned int center_sensor = 0;
volatile unsigned int right_sensor = 0;

//Motors

void ADC_init(void);
void ADC_clear(void);
void init_all(void);
int ADC_conv(void);
void sensor_and_motor_select(int sens_mot_sel);

void ADC_init(void)
{
	ADMUX |= (1<<REFS0);
	ADCSRA |= (1<<ADEN) | (1<<ADATE) | (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0); //Enable the ADC in Free-Running mode with a fadc = 125 kHz
}

void ADC_clear(void)
{
	ADMUX &= ~(1<<MUX3) & ~(1<<MUX2) & ~(1<<MUX1) & ~(1<<MUX0); //Clear ADC MUX
}

void init_all(void)
{
	board_init();
	ADC_clear();
	ADC_init();
	uart_init();
	stdout = stdin = stderr = &uart_str;
	printf("Hello! \n\n");
}

int ADC_conv(void)
{
	ADCSRA |= (1<<ADSC);									//start conversion
	while(!(ADCSRA &(1<<ADSC)));							//wait for it to complete
	uint16_t sample_val_int = ADC;							//store the sample in a variable
	
	//float sample_val_f = (float) sample_val_int;			//convert to float
	//sample_val_f = (float)4095.0(sample_val_f/1023.0);		//calculate voltage in mV
	return sample_val_int;								//return as an int
}

void sensor_and_motor_select(int sens_mot_sel)
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
		ADMUX |= (1<<MUX1) | (1<<MUX0);
		center_sensor = ADC_conv();
		printf("Center sensor: %d \n", center_sensor);
		break;
		
		case RIGHT_SENSOR:
		ADC_clear();
		ADMUX |= (1<<MUX2) | (1<<MUX1);
		right_sensor = ADC_conv();
		printf("Right sensor: %d \n", right_sensor);
		break;
		
		default:
		printf("ERROR: State ID %d not recognized \n", sens_mot_sel);
		break;
	}
}

int main (void)
{
	/* Insert system clock initialization code here (sysclk_init()). */
	/* Insert application code here, after the board has been initialized. */
	
	init_all();
	
	int sens_mot_sel = 0;
	
	while(1)
	{
		sensor_and_motor_select(sens_mot_sel);
		sens_mot_sel++;
		
		if(sens_mot_sel > 2)
		{
			printf("\n");
			sens_mot_sel = 0;
		}
		
		_delay_ms(300);
	}
}
