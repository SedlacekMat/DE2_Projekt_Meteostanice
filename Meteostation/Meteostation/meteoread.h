#ifndef METEOREAD_H
# define METEOREAD_H
/***********************************************************************
 * 
 * TWI library for AVR-GCC.
 * ATmega328P (Arduino Uno), 16 MHz, AVR 8-bit Toolchain 3.6.2
 *
 * Copyright (c) 2021 Matyas Sedlacek
 * Dept. of Radio Electronics, Brno University of Technology, Czechia
 * This work is licensed under the terms of the MIT license.
 *
 **********************************************************************/

/* Includes ----------------------------------------------------------*/
#include "twi.h"

/* Definitions -------------------------------------------------------*/
#define DHT12 0x5C
#define HP206C 0x76 //manufacturer states 0xEC(8bit), which is incorrect, since its higher then max i2c address value, in reality its 0xEC>>1

#define HUM_INT_REG 0x00
#define PRES_MSB_REG 0x30
#define TEMP_MSB_REG 0x32

/* Functions----------------------------------------------------------*/
/**
 * @brief  Get relative humidity from DHT12 sensor as a percentage .
 * @par    Implementation notes:
 *           - i2c communication is established, desired register address
 *             is written into the slave device
 *           - data is read from the register
 * @return Received 8-bit value
 */
uint8_t get_humidity(void);

/**
 * @brief  Get pressure data in Pa from the HP206C sensor .
 * @par    Implementation notes:
 *           - i2c communication is established, desired register address
 *             is written into the slave device
 *           - data is read from the register
 * @return Received 8-bit value
 */
uint32_t get_pressure(void);

/**
 * @brief  Get temperature in degrees C from the HP206C sensor .
 * @par    Implementation notes:
 *           - i2c communication is established, desired register address
 *             is written into the slave device
 *           - data is read from the register
 * @return Received 8-bit value
 */
int32_t get_temperature(void);
#endif