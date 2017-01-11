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

void DoApp1(void);

//- Global variables of DoApp1 --------------------------------------------------
enum A1states		// states of app1 test application
{
  A1waitinit,
  A1prepmsg,
  A1waitfill,
  A1putstart,
  A1putwait
} A1stat;

uint32_t A1Tim = 0;

/** Im Systick wird ein Timer inkrementiert ==> Mit dieser Konstanten vergleichen für Zeitmessungen */
#define WAIT_10ms 10
#define WAIT_200ms 200

/** Sendpuffer für Transfer von Daten an RFM12 */
extern char Ftxbu[BUMAX];

/*******************************************************************************
 * FillTxBu: Fills TxBuffer
 ******************************************************************************/
void FillTxBu(void)
{
	Ftxbu[0]  = 0x11;
	Ftxbu[1]  = 0x13;
	Ftxbu[2]  = 0x09;
	Ftxbu[3]  = 0x01;
	Ftxbu[4]  = 0x51;
	Ftxbu[5]  = 0x52;
	Ftxbu[6]  = 0x53;
	Ftxbu[7]  = 0x54;
	Ftxbu[8]  = 0x00;
	Ftxbu[9]  = 0x00;
}

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
void Waechter_Init(void)
{
  __HAL_SPI_ENABLE(&hspi1);
  RFMInit();
}

void Waechter_Main(void)
{
  DoApp1();
}

void HAL_SYSTICK_Callback(void)
{
  const unsigned int period_ms = 100;
	static unsigned int cnt = period_ms;
  if (cnt == 0)
  {
    cnt = period_ms;
    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
  }
	cnt--;
	
	A1Tim++;
}


/*******************************************************************************
* DoApp1:	Test for RFM12 put: Sends Ftxbu every 200ms
 ******************************************************************************/
void DoApp1(void)
{

	switch(A1stat)				/* App1 state machine for put*/
	{
		case A1waitinit:
		{
			if(A1Tim > WAIT_10ms)
				A1stat = A1prepmsg;
			break;
		}
		case A1prepmsg:
		{
			FillTxBu();
			A1Tim = 0;
			A1stat = A1waitfill;
			break;
		}
		case A1waitfill:
		{
			if(A1Tim > WAIT_200ms)
				A1stat = A1putstart;
			break;
		}
		case A1putstart:
		{
			RFMPutStart();
			A1stat = A1putwait;
			break;
		}
		case A1putwait:
		{
			if(RFMPutReady() == 3)
			{
				A1stat = A1prepmsg;
			}
			break;
		}
	}
}

