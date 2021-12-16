/***********************************************************************
 * 
 * The meteostation DE2 project
 * utlizing pressure, temperature and humidity sensors, an RTC module 
 * two servos for solar panel manipulation, an LCD and UART communication.
 * ATmega328P (Arduino Uno), 16 MHz, AVR 8-bit Toolchain 3.6.2
 *
 * Matej Ledvina, Tomas Marcak, Pavol Rohal, Matyas Sedlacek
 * This work is licensed under the terms of the MIT license.
 * 
 **********************************************************************/

/* Defines -----------------------------------------------------------*/
#ifndef F_CPU
# define F_CPU 16000000  // CPU frequency in H
#endif
#define buttonOk    PC3     // external pushbuttons
#define buttonUp	PB4
#define buttonDown	PB5
#define ok 1
#define up 2
#define down 3

/* Includes ----------------------------------------------------------*/
#include <avr/io.h>         // AVR device-specific IO definitions
#include <avr/interrupt.h>  // Interrupts standard C library for AVR-GCC
#include <stdlib.h>         // C library. Needed for conversion function
#include <stdio.h>          // C library. Needed for conversion function
#include <avr/sfr_defs.h>   // C library. Needed for conversion function
#include "timer.h"          // Timer library for AVR-GCC
#include "uart.h"           // Peter Fleury's UART library
#include "twi.h"            // TWI library for AVR-GCC
#include "lcd.h"            // Peter Fleury's LCD library
#include "meteoread.h"      // Library for reading from DHT12 and HP206C sensors
#include "RTC.h"            // Library for reading and writing to the RTC DS3231

/* Variables ---------------------------------------------------------*/
typedef enum {              // FSM declaration
	STATE_PANELS,
	STATE_READ,
	STATE_TIME,
	STATE_HUMID,
	STATE_PRES,
	STATE_TEMP,
} state_t;

volatile uint8_t humid_rel = 0;      // Humidity in percent
volatile uint32_t press_hpa = 0;     // Pressure in hPa
volatile int32_t temp_c = 0;         // Temperature in Celsius
volatile int *TimePoint;             //Time data pointer
char timeStr[9] = "11:22:33"; //string variable with time data
char dateStr[11];//string variable with date data

void ADC_Init()										// ADC Initialization function
{

	ADCSRA |= 1<<ADEN | 1<<ADPS2 | 1<<ADPS1 ;		// AVCC with external capacitor at AREF pin,  64 prescaler
	ADMUX = (1 << REFS0);
}

int ADC_Read(char channel)							// ADC Read function
{
	ADMUX = (1 << REFS0) | (channel & 0x07);		// set input channel to read
	ADCSRA |= (1<<ADSC);							// Start ADC conversion
	while (ADCSRA & (1<<ADSC));				        // Wait until conversion is completed

	return ADCW;									// Return ADC word
}

//LCD customChar sun
uint8_t customChar[8] = {
	0b00000,
	0b10101,
	0b01110,
	0b11111,
	0b01110,
	0b10101,
	0b00000,
	0b00000
};

int main(void)
{
    // Initialize I2C (TWI)
    twi_init();

    // Initialize UART to asynchronous, 8N1, 9600
    uart_init(UART_BAUD_SELECT(9600, F_CPU));
	
	// Initialize LCD
	lcd_init(LCD_DISP_ON);
	
	//load customCharacter
	lcd_command(1 << LCD_CGRAM);
	for (uint8_t i = 0; i < 8; i++)
	{
		// Store all new chars to memory line by line
		lcd_data(customChar[i]);
	}
	lcd_command(1 << LCD_DDRAM);
	
	
	RTCsetup();
	
	
	ADC_Init();
	
	DDRB |= (1<<PB1);											//Set port B1 & B2 to output
	DDRB |= (1<<PB2);
	TCNT1 = 0;													// Set timer1 count zero
	ICR1 = 2499;												// Set TOP count for timer1 in ICR1 register
	TCCR1A = (1<<WGM11)|(1<<COM1A1)|(1<<COM1B1);				// Set OC1A/OC1B on compare match, clear OC1A/OC1B at BOTTOM (inverting mode)
	TCCR1B = (1<<WGM12)|(1<<WGM13)|(1<<CS10)|(1<<CS11);			// set clk/64 (from prescaler)
    
	// RTC variables
	DDRC = DDRC & ~(1<<buttonOk);
	PORTC = PORTC | (1<<buttonOk);
	
	DDRB = DDRB & ~(1<<buttonUp);
	PORTB = PORTB | (1<<buttonUp);
	
	DDRB = DDRB & ~(1<<buttonDown);
	PORTB = PORTB | (1<<buttonDown);
	
	uint8_t button = 0;
	uint8_t lastButton = 0;
	uint8_t cycle = 0;
	
	
	volatile struct RTC_data	//structure of all RTC variables
	{
		uint8_t sec1;
		uint8_t sec10;
		uint8_t min1;
		uint8_t min10;
		uint8_t hour1;
		uint8_t hour10;
		uint8_t day1;
		uint8_t day10;
		uint8_t month1;
		uint8_t month10;
		uint8_t year1;
		uint8_t year10;
	} data;
	
    // Configure external interrupt INT0 on pin PD2
	EIMSK = (1<<INT0); //setting mask register
	EIFR = (1<<INTF0); //setting flag register
	EICRA = (1<<ISC01)|(1<<ISC00); // //setting request register

    // Enables interrupts by setting the global interrupt mask
    sei();
   

    // Infinite loop
    while (1)
    {
		if (bit_is_clear(PINC, buttonOk))
		{
			button=1;
		}
		else if (bit_is_clear(PINB, buttonUp))
		{
			button=2;
		}
		else if (bit_is_clear(PINB, buttonDown))
		{
			button=3;
		}
		else button=0;
		
		if(button!=lastButton && button!=0)//if button was pressed
		{
			switch(cycle) //setup sequence state machine
			{
				case 0://start
				lcd_clrscr(); //clear LCD
				lcd_puts("Setup:");
				cli(); //disable all interrupts
				*TimePoint=getTimeDate();
				
				if(button==ok) cycle = 1;
				break;
				
				case 1://set seconds 1
				if(button==up)
				{
					if(data.sec1==9) data.sec1=0;
					else data.sec1++;
				}
				else if(button==down)
				{
					if(data.sec1==0) data.sec1=9;
					else data.sec1--;
				}
				else if(button==ok) cycle = 2;
				break;
				case 2://set seconds 10
				if(button==up)
				{
					if(data.sec10==5) data.sec10=0;
					else data.sec10++;
				}
				else if(button==down)
				{
					if(data.sec10==0) data.sec10=5;
					else data.sec10--;
				}
				else if(button==ok) cycle = 3;
				break;
				
				case 3://set minutes 1
				if(button==up)
				{
					if(data.min1==9) data.min1=0;
					else data.min1++;
				}
				else if(button==down)
				{
					if(data.min1==0) data.min1=9;
					else data.min1--;
				}
				else if(button==ok) cycle = 4;
				break;
				
				case 4://set minutes 10
				if(button==up)
				{
					if(data.min10==5) data.min10=0;
					else data.min10++;
				}
				else if(button==down)
				{
					if(data.min10==0) data.min10=5;
					else data.min10--;
				}
				else if(button==ok) cycle = 5;
				break;
				case 5://set hours
				if(button==up)
				{
					if(data.hour1==9 && data.hour10<2)
					{
						data.hour1=0;
						data.hour10++;
					}
					else if(data.hour1>2 && data.hour10==2)
					{
						data.hour1=0;
						data.hour10=0;
					}
					else data.hour1++;
				}
				else if(button==down)
				{
					if(data.hour1==0 && data.hour10>0)
					{
						data.hour1=9;
						data.hour10--;
					}
					else if(data.hour1==0 && data.hour10==0)
					{
						data.hour1=3;
						data.hour10=2;
					}
					else data.hour1--;
				}
				else if(button==ok) cycle = 6;
				break;
				
				case 6://set days
				if(button==up)
				{
					if(data.day1==9 && data.day10<3)
					{
						data.day1=0;
						data.day10++;
					}
					else if(data.day1>3 && data.day10==1)
					{
						data.day1=0;
						data.day10=0;
					}
					else data.day1++;
				}
				else if(button==down)
				{
					if(data.day1==0 && data.day10>0)
					{
						data.day1=9;
						data.day10--;
					}
					else if(data.day1==0 && data.day10==0)
					{
						data.day1=1;
						data.day10=3;
					}
					else data.day1--;
				}
				else if(button==ok) cycle = 7;
				break;
				
				case 7://set months
				if(button==up)
				{
					if(data.month1==9 && data.month10<1)
					{
						data.month1=0;
						data.month10++;
					}
					else if(data.month1>1 && data.month10==1)
					{
						data.month1=0;
						data.month10=0;
					}
					else data.month1++;
				}
				else if(button==down)
				{
					if(data.month1==0 && data.month10>0)
					{
						data.month1=9;
						data.month10--;
					}
					else if(data.month1==0 && data.month10==0)
					{
						data.month1=2;
						data.month10=1;
					}
					else data.month1--;
				}
				else if(button==ok) cycle = 8;
				break;
				
				case 8://set year 1
				if(button==up)
				{
					if(data.year1==9) data.year1=0;
					else data.year1++;
				}
				else if(button==down)
				{
					if(data.year1==0) data.year1=9;
					else data.year1--;
				}
				else if(button==ok) cycle = 9;
				break;
				
				case 9://set year 10
				if(button==up)
				{
					if(data.year10==9) data.year10=0;
					else data.year10++;
				}
				else if(button==down)
				{
					if(data.year10==0) data.year10=9;
					else data.year10--;
				}
				else if(button==ok)
				{
					
					setTimeDate(*TimePoint); //send all the data to RTC
					sei(); //enable interrupts
					cycle = 0; //reset the state machine

				}
				break;
				
			}
			
			//store the time from RTC in a string variable as hh:mm:ss
			sprintf(timeStr, "%i%i:%i%i:%i%i",data.hour10, data.hour1, data.min10
			,data.min1, data.sec10, data.sec1);
			//store the date from RTC in a string as dd.mm.yyyy
			sprintf(dateStr, "%i%i.%i%i.20%i%i", data.day10, data.day1, data.month10
			, data.month1, data.year10, data.year1);
			
			lcd_gotoxy(7, 0);
			lcd_puts(timeStr); //display time  on LCD
			lcd_gotoxy(5, 1);
			lcd_puts(dateStr); //display date on LCD
			if(cycle==0) lcd_clrscr(); //clear LCD after setup
			
		}
		lastButton=button;
		
		
	}
    // Will never reach this
    return 0;
	}
/* Interrupt service routines ----------------------------------------*/
/**********************************************************************
 * Function: Timer/Counter1 overflow interrupt
 * Purpose:  Update Finite State Machine and test I2C slave addresses 
 *           between 8 and 119.
 **********************************************************************/
ISR(INT0_vect)
{
	char hum_string[9];
	char pres_string[33];
	char temp_string[33];
	char time_string[9] = "11:22:33"; //string variable with time data
	uint16_t val = 0;
	
    static state_t state = STATE_PANELS;  // Current state of the FSM

    // FSM
    switch (state)
    {
	case STATE_PANELS:
		if (ADC_Read(2)>310)					//do when value on light sensor is grater then 30%
		{
			val = ((ADC_Read(0) - 340)/3.4);		//write ADC value to val & conversion to percentage
			OCR1A = 251 + (val*2.49);				//move with servo by percentage value
			val = ((ADC_Read(1) - 340)/3.4);
			OCR1B = 251 + (val*2.49);
			lcd_gotoxy(15,0);                       //display a sun symbol on the LCD
			lcd_putc(' ');
			lcd_gotoxy(15,0);
			lcd_putc(0);
		}
		else
		{
			lcd_gotoxy(15,0);                       //clear the sun symbol slot
			lcd_putc(' ');
		}		
		state=STATE_READ;
		break;
	
	case STATE_READ:
		humid_rel = get_humidity();                 // read all data
		press_hpa = get_pressure();
		temp_c = get_temperature();
		*TimePoint = getTimeDate();
		state = STATE_TIME;
		break;
		
	case STATE_TIME:
		//Convert time values
		sprintf(time_string,"%i%i:%i%i:%i%i",*(TimePoint+5),*(TimePoint+4),*(TimePoint+3),*(TimePoint+2),*(TimePoint+1),*(TimePoint));
		
		//Send to UART
		uart_puts("\r\nTime: ");
		uart_puts(time_string);
		uart_puts(";");
		//Display on LCD
		lcd_gotoxy(0,0);
		lcd_puts("         ");
		lcd_gotoxy(0,0);
		lcd_puts(time_string);
		lcd_gotoxy(5,0);
		lcd_puts("   ");
		//Next state
		state=STATE_HUMID;
		break;
		 
	case STATE_HUMID:
		//Convert humidity values
		utoa(humid_rel, hum_string, 10);
		//Send to UART
		uart_puts(" Hum: ");
		uart_puts(hum_string);
		uart_puts(" %;");
		//Display on LCD
		lcd_gotoxy(1,1);
		lcd_puts("   ");
		lcd_gotoxy(1,1);
		lcd_puts(hum_string);
		lcd_putc('%');
		//Next state
		state = STATE_PRES;
		break;
	
    case STATE_PRES:      	
		//Convert pressure to string
		ultoa(press_hpa,pres_string,10);
		//Send to UART
        uart_puts(" Pres: ");
        uart_puts(pres_string);
        uart_puts(" hPa;");
		//Display on LCD
		lcd_gotoxy(7,1);
		lcd_puts("        ");
		lcd_gotoxy(7,1);
		lcd_puts(pres_string);
		lcd_puts(" hPa");
		//Next state
        state=STATE_TEMP;
        break;
    
    case STATE_TEMP:
		//Convert to string
		ltoa(temp_c,temp_string,10);
		//data communicated over UART
		uart_puts(" Temp: ");
		uart_puts(temp_string);
		uart_puts(" C;");
		//display on LCD
		lcd_gotoxy(10,0);
		lcd_puts("    ");
		lcd_gotoxy(10,0);
		lcd_puts(temp_string);
		lcd_puts("C");
		
              
       state=STATE_PANELS;

        break;
  
    default:
        state = STATE_PANELS;
        break;
    }
}