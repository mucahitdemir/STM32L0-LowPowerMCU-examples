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

#ifndef _BMP180_H_
#define _BMP180_H_


#include "stm32l0xx_hal.h"

void BMP180_Start (void);

float BMP180_Temperature (void);

float BMP180_Pressure (int oss);

float BMP180_Altitude (int oss);

#endif /* INC_BMP180_H_ */
