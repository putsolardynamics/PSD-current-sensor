/*
 * LTC2485.h
 *
 *  Created on: Aug 19, 2021
 *      Author: jan
 */

#ifndef INC_LTC2485_H_
#define INC_LTC2485_H_

#include "main.h"
#include "adc.h"
#include "can.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

#define LTC_ADDERS  0b00101110
#define LTC248XADDR 0b01001000

// Select ADC source - differential input or PTAT circuit
#define VIN    0b00000000
#define PTAT   0b00001000
// Select rejection frequency - 50, 55, or 60Hz
#define R50    0b00000010
#define R55    0b00000000
#define R60    0b00000100
// Select speed mode
#define SLOW   0b00000000 // slow output rate with autozero
#define FAST   0b00000001 // fast output rate with no autozero

extern I2C_HandleTypeDef hi2c1;

int32_t LTC_get_Current(){

	uint8_t buf[8];

	HAL_I2C_Master_Receive(&hi2c1, LTC248XADDR | 1, &buf,4, HAL_MAX_DELAY);

	int32_t x = (buf[0])<<24 | buf[1]<<16 | buf[2]<<8 | (buf[3] & 0b11000000);
	x ^= 0x80000000;

	const float Vref = 4090;
	float voltage = (float) x;
	voltage = voltage * Vref / 2147483648.0; //  voltage * Vref_in_mV / 2^31;

	float current = voltage / (20 * 0.001); //  voltage / (GAIN * Rshunt) ;
	current *=1.006; //gain calibration
	current += 25; //offest calibration
	//if(current<50 && current>-50)current = 0;// elminate small drift
	if(fabsf(current)<50)current = 0;
	current += 0.5; //to round number

	return (int32_t)current;
}



void LTC_init(){
	int8_t config = VIN | R50 | SLOW;
	HAL_I2C_Master_Transmit(&hi2c1, LTC248XADDR, &config, 1, HAL_MAX_DELAY);
	HAL_I2C_Master_Transmit(&hi2c1, LTC248XADDR, &config, 1, HAL_MAX_DELAY);\
}






#endif /* INC_LTC2485_H_ */
