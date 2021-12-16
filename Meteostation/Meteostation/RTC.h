/*X<O>X<O>X<O>X<O>X<O>X<O>X<O>X<O>X<O>X<O>X<O>X<O>X<O>X<O>X<O>X<O>X<O>X<O>X<O>X<O>X<|
|																	    			|
|       		       DS3231 RTC Library for I2C communication						|
|																					|
|>X<O>X<O>X<O>X<O>X<O>X<O>X<O>X<O>X<O>X<O>X<O>X<O>X<O>X<O>X<O>X<O>X<O>X<O>X<O>X<O>X*/

//#define RTC 0x68 //I2C address of the RTC module
#ifndef RTC_H_
#define RTC_H_
#include <avr/io.h>
#include <stdlib.h>
#include <stdio.h>
#include <avr/sfr_defs.h>
#include "twi.h"
#include "lcd.h"

struct RTC_data	//structure of all RTC variables
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
}data;
int myArray[12];




//#define RTC 0x68 //I2C address of the RTC module


int getTimeDate();

void setTimeDate(int myArray[12]);

void RTCsetup();

#endif /* RTC_H_ */