/**
  ******************************************************************************

  BMP180 LIBRARY for STM32 using I2C
  Author:   Mucahit Demirci
  Updated:  13/06/2022

  ******************************************************************************
  Copyright (C) 2017 Mucahit Demirci

  This is a free software under the GNU license, you can redistribute it and/or modify it under the terms
  of the GNU General Public License version 3 as published by the Free Software Foundation.
  This software library is shared with public for educational purposes, without WARRANTY and Author is not liable for any damages caused directly
  or indirectly by this software, read more about this on the GNU General Public License.

  ******************************************************************************
*/

/*
*BMP180 DIGITAL PRESSURE SENSOR
*The BMP180 is the function compatible successor of the BMP085, a new generation of high
*precision digital pressure sensors for consumer applications.
*
*
*
*/

#include "stm32l0xx_hal.h"

#include "math.h"

extern I2C_HandleTypeDef hi2c1;

#define BMP180_I2C &hi2c1

#define BMP180_ADDRESS 0xEE


/*Defines according to the datasheet*/

short AC1 = 0;
short AC2 = 0;
short AC3 = 0;
unsigned short AC4 = 0;
unsigned short AC5 = 0;
unsigned short AC6 = 0;
short B1 = 0;
short B2 = 0;
short MB = 0;
short MC = 0;
short MD = 0;

/*Calibration coefficients */
long UT = 0;
short oss = 0;
long UP = 0;
long X1 = 0;
long X2 = 0;
long X3 = 0;
long B3 = 0;
long B5 = 0;
unsigned long B4 = 0;
long B6 = 0;
unsigned long B7 = 0;

/*******************/
long Pressure = 0;
long Temperature = 0;

#define ATM_Pressure 101325 //Pa



void read_calibration_data (void)
{
	uint8_t calibration_data[22] = {0};
	uint16_t calibration_start = 0xAA;
	
	HAL_I2C_Mem_Read(BMP180_I2C, BMP180_ADDRESS, calibration_start, 1, calibration_data,22, HAL_MAX_DELAY);

	AC1 = ((calibration_data[0] << 8) | calibration_data[1]);
	AC2 = ((calibration_data[2] << 8) | calibration_data[3]);
	AC3 = ((calibration_data[4] << 8) | calibration_data[5]);
	AC4 = ((calibration_data[6] << 8) | calibration_data[7]);
	AC5 = ((calibration_data[8] << 8) | calibration_data[9]);
	AC6 = ((calibration_data[10]<< 8) | calibration_data[11]);
	B1 = ((calibration_data[12] << 8) | calibration_data[13]);
	B2 = ((calibration_data[14] << 8) | calibration_data[15]);
	MB = ((calibration_data[16] << 8) | calibration_data[17]);
	MC = ((calibration_data[18] << 8) | calibration_data[19]);
	MD = ((calibration_data[20] << 8) | calibration_data[21]);

}


/* Observe uncompensated temperature data*/
uint16_t uncompensated_temperature (void)
{
	uint8_t datatowrite = 0x2E;
	uint8_t Temp_RAW[2] = {0};
	HAL_I2C_Mem_Write(BMP180_I2C, BMP180_ADDRESS, 0xF4, 1, &datatowrite, 1, 1000);
	HAL_Delay (5);  // wait 4.5 ms
	HAL_I2C_Mem_Read(BMP180_I2C, BMP180_ADDRESS, 0xF6, 1, Temp_RAW, 2, 1000);
	return ((Temp_RAW[0]<<8) + Temp_RAW[1]);
}

float BMP180_Temperature (void)
{
	UT = uncompensated_temperature();
	X1 = ((UT-AC6) * (AC5/(pow(2,15))));
	X2 = ((MC*(pow(2,11))) / (X1+MD));
	B5 = X1+X2;
	Temperature = (B5+8)/(pow(2,4));
	return Temperature/10.0;
}

/* Observe uncompensated pressure data*/
uint32_t uncompensated_pressure (int oss)   // over-sampling setting 0,1,2,3
{
	uint8_t datatowrite = 0x34+(oss<<6);
	uint8_t Press_RAW[3] = {0};
	HAL_I2C_Mem_Write(BMP180_I2C, BMP180_ADDRESS, 0xF4, 1, &datatowrite, 1, 1000);
	switch (oss)
	{
		case (0):
			HAL_Delay (5);
			break;
		case (1):
			HAL_Delay (8);
			break;
		case (2):
			HAL_Delay (14);
			break;
		case (3):
			HAL_Delay (26);
			break;
	}
	HAL_I2C_Mem_Read(BMP180_I2C, BMP180_ADDRESS, 0xF6, 1, Press_RAW, 3, 1000);
	return (((Press_RAW[0]<<16)+(Press_RAW[1]<<8)+Press_RAW[2]) >> (8-oss));
}


float BMP180_Pressure (int oss)
{
	UP = uncompensated_pressure(oss);
	X1 = ((UT-AC6) * (AC5/(pow(2,15))));
	X2 = ((MC*(pow(2,11))) / (X1+MD));
	B5 = X1+X2;
	B6 = B5-4000;
	X1 = (B2 * (B6*B6/(pow(2,12))))/(pow(2,11));
	X2 = AC2*B6/(pow(2,11));
	X3 = X1+X2;
	B3 = (((AC1*4+X3)<<oss)+2)/4;
	X1 = AC3*B6/pow(2,13);
	X2 = (B1 * (B6*B6/(pow(2,12))))/(pow(2,16));
	X3 = ((X1+X2)+2)/pow(2,2);
	B4 = AC4*(unsigned long)(X3+32768)/(pow(2,15));
	B7 = ((unsigned long)UP-B3)*(50000>>oss);
	if (B7<0x80000000) Pressure = (B7*2)/B4;
	else Pressure = (B7/B4)*2;
	X1 = (Pressure/(pow(2,8)))*(Pressure/(pow(2,8)));
	X1 = (X1*3038)/(pow(2,16));
	X2 = (-7357*Pressure)/(pow(2,16));
	Pressure = Pressure + (X1+X2+3791)/(pow(2,4));

	return Pressure;
}


float BMP180_Altitude (int oss)
{
	BMP180_Pressure (oss);
	return 44330*(1-(pow((Pressure/(float)ATM_Pressure), 0.19029495718)));
}

void BMP180_Start (void)
{
	read_calibration_data();
}

