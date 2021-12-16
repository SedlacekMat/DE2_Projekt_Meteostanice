/*X<O>X<O>X<O>X<O>X<O>X<O>X<O>X<O>X<O>X<O>X<O>X<O>X<O>X<O>X<O>X<O>X<O>X<O>X<O>X<O>X<|
|																	    			|
|       		       DS3231 RTC Library for I2C communication						|
|																					|
|>X<O>X<O>X<O>X<O>X<O>X<O>X<O>X<O>X<O>X<O>X<O>X<O>X<O>X<O>X<O>X<O>X<O>X<O>X<O>X<O>X*/

#include "RTC.h"
#define RTC 0x68 //I2C address for the RTC

int getTimeDate() //this function pulls the time and date data from the RTC
{
	uint8_t temp;
	static int myArray[12];
	twi_start((RTC<<1)+TWI_WRITE);
	twi_write(0x00);
	twi_start((RTC<<1)+TWI_READ);
	//get seconds
	temp=twi_read_ack();
	data.sec1=(temp & 0b00001111);
	data.sec10=((temp & 0b01110000)>>4);
	//get minutes
	temp=twi_read_ack();
	data.min1=(temp & 0b00001111);
	data.min10=(((temp) & 0b01110000)>>4);
	//get hours
	temp=twi_read_nack();
	data.hour1=(temp & 0b00001111);
	data.hour10=((temp & 0b00110000)>>4);
	
	twi_start((RTC<<1)+TWI_WRITE);
	twi_write(0x04);
	twi_start((RTC<<1)+TWI_READ);
	//get day
	temp=twi_read_ack();
	data.day1=(temp & 0b00001111);
	data.day10=((temp & 0b00110000)>>4);
	//get month
	temp=twi_read_ack();
	data.month1=(temp & 0b00001111);
	data.month10=((temp & 0b00010000)>>4);
	//get year
	temp=twi_read_nack();
	data.year1=(temp & 0b00001111);
	data.year10=((temp & 0b11110000)>>4);
	twi_stop();
	//convert struct data to an array and return to the program
	myArray[0] = data.sec1;
	myArray[1] = data.sec10;
	myArray[2] = data.min1;
	myArray[3] = data.min10;
	myArray[4] = data.hour1;
	myArray[5] = data.hour10;
	myArray[6] = data.day1;
	myArray[7] = data.day10;
	myArray[8] = data.month1;
	myArray[9] = data.month10;
	myArray[10] = data.year1;
	myArray[11] = data.year10;
	return *myArray;
}

void setTimeDate(int *TimePoint) //this function reads the set time from the program and sends it to the RTC
{
twi_start((RTC<<1)+TWI_WRITE);
twi_write(0x00);
//set seconds
twi_write(((*(TimePoint))  & 0b00001111)
+((*(TimePoint+1)) & 0b00001111)<<4);
//set minutes
twi_write(((*(TimePoint+2)) & 0b00001111)
+((*(TimePoint+3)) & 0b00001111)<<4);
//set hour
twi_write(((*(TimePoint+4)) & 0b00001111)
+((*(TimePoint+5)) & 0b00001111)<<4);

twi_start((RTC<<1)+TWI_WRITE);
twi_write(0x04);
//set day
twi_write(((*(TimePoint+6)) & 0b00001111)
+((*(TimePoint+7)) & 0b00001111)<<4);
//set month
twi_write(((*(TimePoint+8)) & 0b00001111)
+((*(TimePoint+9)) & 0b00001111)<<4);
//set year
twi_write(((*(TimePoint+10)) & 0b00001111)
+((*(TimePoint+11)) & 0b00001111)<<4);
twi_stop();
	
}

void RTCsetup() //this function provides the initial setup for the RTC module
{
	twi_start((RTC<<1)+TWI_WRITE);
	twi_write(0x0E);
	//set control register
	twi_write(0b01000000);
	
	twi_start((RTC<<1)+TWI_WRITE);
	twi_write(0x00);
	//set seconds
	twi_write(0b00000000);
	//set minutes
	twi_write(0b00000000);
	//set hours
	twi_write(0b00000000);
	
	twi_start((RTC<<1)+TWI_WRITE);
	twi_write(0x04);
	//set day 	
	twi_write(0b00000001);
	//set month
	twi_write(0b00000001);
	//set year
	twi_write(0b00100001);

	twi_stop();

}