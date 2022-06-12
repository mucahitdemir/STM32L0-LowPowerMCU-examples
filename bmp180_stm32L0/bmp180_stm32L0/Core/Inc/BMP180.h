/**
  ******************************************************************************

  BMP180 LIBRARY for STM32 using I2C
  Author:   ControllersTech
  Updated:  26/07/2020

  ******************************************************************************
  Copyright (C) 2017 ControllersTech.com

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

float BMP180_GetTemp (void);

float BMP180_GetPress (int oss);

float BMP180_GetAlt (int oss);

#endif /* INC_BMP180_H_ */
