/**
 * @file Waechter_Main.c
 * @author Jens Heilig
 * @date 30.12.2016
 * @brief Implements Waechter functions
 *
 * This implements the software functions of Waechter, a device
 * for monitoring a digital input and sending a message via RFM12
 * on each change of the input.
 * 
 * Copyright (C) 2016 Jens Heilig
 *
 * This software may be modified and distributed under the terms
 * of the MIT license.  See the LICENSE file for details.
*/

#include "stm32f1xx_hal.h"
#include "Driver_RFM12.h"

extern SPI_HandleTypeDef hspi1;

/**
 * RFM12_Sendword sends one data word (16 bit) to RFM12 module
 *
 * @param data - data word (16 bits) to send to RFM12
 */
void RFM12_Sendword(uint16_t data)
{
  HAL_SPI_Transmit(&hspi1, (uint8_t *) &data, 2, 100); /// @TODO Big-Endian/Little-Endian Conversion necessary?
}

 
/**
 * Initialize the Waechter routines
 *
 */
void  Waechter_Init(void)
{
  __HAL_SPI_ENABLE(&hspi1);
  RFMInit();
}

void  Waechter_Main(void)
{
  ;
}
