/******************************************************************************
 * title   	RFM12Lib.h
 * author  	Walter Trojan
 * date  	05.02.2014
 * MCU		STM32F103C8 with 72.0 Mhz clock
 * target  	Header for RFM12Lib.c
 * release  3.0
 *
 *Contents:
 *	main.c:    This is a test program for the Lib. It is organized a multiple
 *             finite state machine, in order to demonstrate the non-blocking
 *             mode. It uses an external Interrupt on GPIOA.8. The ISR for this
 *             has to be included in your app as well.
 *
 *             Perform a message transmission:
 *               Init the RFM12 by calling InitRFM12 (only once)
 *               Fill the the buffer Txbu according to the protocol
 *               Start transmission by RFMPutStart
 *               Now you can do something else or
 *               Ask the status of transmission by RFMPutReady
 *               If this function replies with 3 then put is done and OK
 *
 *              Perform a message receive:
 *               Init the RFM12 by calling InitRFM12 (only once)
 *               Start receive by RFMGetStart
 *               Now you can do something else or
 *               Ask the status of receive by RFMGetReady
 *               If this function replies with 3 then get is done and OK
 *               Take the message out of buffer Rxbu
 *
 *  RFMLib.h:  Header file for the Lib.
 *
 *	RFMLib.c:  This is the RFM12-Library Release 3.0
 *             In your application, you only have to use:
 *               InitRFM12
 *               RFMPutStart
 *               RFMPutReady
 *               RFMgetStart
 *               RFMgetReady
 *
 *
 *
*******************************************************************************/

#define BUMAX 32					   // size of Txbu and RXbu
#include "stdint.h"


uint16_t RFMInit(void);
uint16_t RFMGetStart(void);
uint16_t RFMGetStop(void);
uint16_t RFMGetReady(void);
uint16_t RFMPutStart(void);
uint16_t RFMPutStop(void);
uint16_t RFMPutReady(void);
char	 RFMIntRX(void);
void	 RFMIntTX(void);

void delay_us(uint32_t dele);
void delay_ms(uint32_t delms);

extern char Txbu[BUMAX];					   // Put buffer
extern char Rxbu[BUMAX];					   // Get buffer
