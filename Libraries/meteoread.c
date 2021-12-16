/***********************************************************************
 * 
 * meteoread library for AVR-GCC.
 * ATmega328P (Arduino Uno), 16 MHz, AVR 8-bit Toolchain 3.6.2
 *
 * Copyright (c) 2021 Matyas Sedlacek
 * Dept. of Radio Electronics, Brno University of Technology, Czechia
 * This work is licensed under the terms of the MIT license.
 *
 **********************************************************************/

/* Includes ----------------------------------------------------------*/
#include "meteoread.h"
#include "twi.h"

/* Functions ---------------------------------------------------------*/
/**********************************************************************
 * Function: get_humidity()
 * Purpose:  Get humidity data from DHT12 sensor.
 * Returns:  Relative humidity as a percentage with +-1% precision
 **********************************************************************/
uint8_t get_humidity(void)
{
	uint8_t data = 0;
	twi_start((DHT12<<1) + TWI_WRITE);
	twi_write(HUM_INT_REG);
	twi_stop();
	twi_start((DHT12<<1) + TWI_READ);
	data = twi_read_nack();
	twi_stop();
	return(data);	
}
/**********************************************************************
 * Function: get_pressure()
 * Purpose:  Get pressure data from HP206C sensor.
 * Returns:  Absolute pressure in units hPA as a 20-bit unsigned integer stored as 32-bits
 **********************************************************************/
uint32_t get_pressure(void)
{
	uint32_t data = 0;
	twi_start((HP206C<<1) + TWI_WRITE);
	twi_write(PRES_MSB_REG);
	twi_stop();
	twi_start((HP206C<<1) + TWI_READ);
	uint8_t get4 = twi_read_ack()<<4;
	data = get4<<12;
	data = data|(twi_read_ack()<<8);
	data = data|twi_read_nack();
	data = data/100;
	twi_stop();
	return(data);
}
/**********************************************************************
 * Function: get_temperature()
 * Purpose:  Get temperature data from HP206C sensor.
 * Returns:  Temperature in degrees Celsius as a 20-bit signed integer stored as 32-bits
 **********************************************************************/
int32_t get_temperature()
{
	int32_t data = 0;
	twi_start((HP206C<<1) + TWI_WRITE);
	twi_write(TEMP_MSB_REG);
	twi_stop();
	twi_start((HP206C<<1) + TWI_READ);
	int8_t get4 = twi_read_ack()<<4;
	data = get4<<12;
	data = data|(twi_read_ack()<<8);
	data = data|twi_read_nack();
	data = data/100;
	twi_stop();
	return(data);
}
}
