/******************************************************************************
 * title   	RFM12Lib.c
 * author  	Walter Trojan
 * date  	05.02.2014
 * MCU		STM32F103C8 with 72.0 Mhz clock
 * target  	Library functions
 * release  3.0
 * Modified for use with CubeMX and ST HAL libraries by Jens Heilig
 *
 *Contents:
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
 *  Driver_RFM12.h:  Header file for the Lib.
 *
 *	Driver_RFM12.c:  This is the RFM12-Library Release 3.0
 *             In your application, you only have to use:
 *               InitRFM12
 *               RFMPutStart
 *               RFMPutReady
 *               RFMgetStart
 *               RFMgetReady
 *
 *
 *
*******************************************************************************

   Message-Format
   ==============

                Byte            Content

   Header:      0               0xAA = RF-Sync
                1               0xAA = RF-Sync
                2               0xAA = RF-Sync
                3               0x2D = FIFO-Sync
                4               0xD4 = FIFO-Sync

   Payload:     0               Node address target
                1               Node address source
                2               Message length from 0..CRC16_low (n + 2)
                3..n            Data of message
                n+1             CRC16 high
                n+2             CRC16 low  (from 0..n)

   Trailer:     0               0xAA = RF-Sync


   Implementation of Hamming coding/decoding
   =========================================

   For a transparent transmission of the payload, Hamming method is introduced
   with Release 3.0 of RFM12 protocol.

   The following shows the message frame of the RFM12 protocol:

               SOF           Sync           Payload   EOF
   Bits        16            16             8*n       8
   Value       0xAA  0xAA    0x2D  0xD4     Data      0xAA

   The control byte 0xAA will be used in the SOF (start of frame) and
   EOF (end of frame).  If in the payload such a control byte appears,
   the RFM12 might detect this as EOF and terminates the transmission.
   If you transmit analog values, your payload may have 0xAA in the message
   or the CRC16 bytes may have such a value as well.

   In order to secure the transmission, one way is to split each byte of the
   payload into two halfbytes and sends them in seperate full bytes.
   If you ecode these bytes according to Hamming, you will gain additionally
   security. Because this code is able to detect multiple bit errors and can
   correct 1-bit errors (not implemented here).
   Of coarse reduce the transmission speed to 50%, but RFM12 is fast enough to
   let it run with double the speed.

   The following scheme shows the encoding:

                  Algorithm                             Sample 0x96
                  8-Bit Value                            10010110
              4-7              0-3                    1001        0110
              HammingE[(4-7)]  HammingE[(0-3)]        0xC7        0x38
              high               low               11000111        00111000
                 16-Bit Codeword                       11000111 00111000



   The Hamming values for the halfbytes are notified here:

   0    1    2    3    4    5    6    7    8    9    A    B    C    D    E    F
   0x15 0x02 0x49 0x5E 0x64 0x73 0x38 0x2F 0xD0 0xC7 0x8C 0x9B 0xA1 0xB6 0xFD 0xEA


   In order to reach maximum range:

    Bitrate: 2k - 10kbit/s.
    Receiver Baseband Bandwidth: 134kHz
    RSSI Threshold: -97dBm
    LNA gain: 0dB. Falls in der Gegend St�rer im gleichen Frequenzbereich sind,
              dann -6dB, ansonsten kann die Eingangsstufe �bersteuern.
    FSK frequency deviation: +/-90kHz
    Output Power: 0dB


   Routines
   ========

   RFMInit:	    Init GPIOs, SPI, RFM-basics
   	   	   	    Off states of receiver and sender
   	   	   	    Interrupts blocked
   	   	   	    Return: 1=OK, 90=SPI-Error

   RFMGetStart: Init RFM for receive
   	   	   	  	Enables interrupts, routine waits for incoming bytes
   	   	   	  	Erase RX-buffer
   	   	   	  	Return: 1=OK, 90=SPI-Error

   RFMGetStop:	Terminates receiving
   	   	   	  	Stops IR
   	   	   	  	Return: 1=OK, 90=SPI-Error

   RFMGetReady: Gives status of get
   	   	   	  	Return: 0=Get not startet
   	   	   	  	        1=Get requested
   	   	   	  	        2=Get running
   	   	   	  	        3=Get ready and OK
   	   	   	  	        8=Get stopped
   	   	   	  	        9=Get error

   RFMPutStart:	Init RFM for send
   	   	   	  	Enables IR, routine perform transmission
   	   	   	  	Return: 1=OK, 90=SPI-Error

   RFMPutStop:	Stops RFM send
   	   	   	  	Disable IR
   	   	   	  	Return: 1=OK, 90=SPI-Error

   RFMPutReady: Gives status of put
     	   	  	Return: 0=Put not startet
   	   	   	  	        1=Put requested
   	   	   	  	        2=Put running
   	   	   	  	        3=Put ready and OK
   	   	   	  	        8=Put stopped
   	   	   	  	        9=Put error

   Variables
   =========

   Rxbu[BUMAX]  Get-buffer
   Txbu[BUMAX]  Put-buffer
   Fmod			Modus of RFMLib
   	   	   	  	        0=Nothing initialized
   	   	   	  	        1=RFM init
   	   	   	  	        2=Get mode
   	   	   	  	        3=Get mode finished
   	   	   	  	        4=Put mode
   	   	   	  	        5=Put mode finished

   Fget			Status of get (as RFMGetReady)
   	   	   	   	   	   	1=Get started
   	   	   	   	   	   	2=Get running
   	   	   	   	   	   	3=Get finished and OK
   	   	   	   	   	   	8=Get not OK, Hamming error
   	   	   	   	   	   	9=Get not OK, CRC error

   Fput			Status of put (as RFMPutReady)
   	   	   	   	   	    1=Put started
   	   	   	   	   	    2=Put running
   	   	   	   	   	    3=Put finished and OK



*******************************************************************************/

#include "stm32f1xx_hal.h"
#include "Driver_RFM12.h"
#include "main.h"

/*******************************************************************************
 * Declare function prototypes
*******************************************************************************/

  uint16_t 	SPIPutGet16 (SPI_TypeDef* SPIx , uint16_t wrd);
  uint16_t 	CRCalc(char bu[], uint16_t istrt, uint16_t iend);
  uint8_t	 	DecHam(uint8_t hambyt);
  void     	delay_us(uint32_t dele);
  void     	delay_ms(uint32_t delms);
  extern void Probe(void);


/*******************************************************************************
* Global variables
*******************************************************************************/
  extern SPI_HandleTypeDef hspi1;

  char Txbu[BUMAX];					   // Put buffer
  char Rxbu[BUMAX];					   // Get buffer

  char 	 Ftxbu[BUMAX];				   // TX buffer
  char 	 Frxbu[BUMAX];				   // RX buffer
  uint16_t Fmod;					   // modus, 1=init, 2=get, 3=get ready, 4=put, 5=put ready
  uint16_t Fget;					   // status get, 1=req., 2=run, 3=OK, 9=err
  uint16_t Fput;					   // status put, 1=req., 2=run, 3=OK, 9=err
  uint16_t Ferr;					   // error numb.
  uint16_t Fidx;					   // index buffer
  uint16_t Flen;					   // msg length
  uint16_t Fint;					   // tx int mode, 1=header, 2=payload, 3=trailer
  uint16_t Fham;					   // Halfbyte indicator, 1=hi, 2=lo for Hamming
  char     Fhi,Flo;					   // Halfbytes for get Int

  uint16_t Fhead[5]  = {0xB8AA,0xB8AA,0xB8AA,0xB82D,0xB8D4};
  char     Fhval[16] = {0x15,0x02,0x49,0x5E,0x64,0x73,0x38,0x2F,
			            0xD0,0xC7,0x8C,0x9B,0xA1,0xB6,0xFD,0xEA};

#define CMAXI  14
uint16_t Cinit [CMAXI] =
           {0x80D8,        // enable register,433MHz,12.5pF
            0x8258,        // enable baseband,synt,!RX, !TX
            0xCC15,		     // only RFM12B, low power
            0xC000,        // clock, brownout
            0xA640,        // 434MHz
            0xC647,        // 4.8kbps 9.6kps=0xC623 19.2kps=0xC611
            0x94A0,        // VDI,FAST,134kHz,0dBm,-103dBm
            0xCED4,        // only RFM12B, sync pattern
            0xC2AC,		     // autobit, digifilter
            0xCA81,		     // FIFO empty, reset off
            0xC4B7,		     // AFC
            0x9860,        // !mp,90kHz deviation,MAX OUT
            0xE000,        // wakeup
            0xC800};       // cyclic RX off

                /*  				  start RX
                 *  0xCA81            FIFO init
                 *  0xCA83            FIFO activ
                 *  0x82D8			  RX on
                 *
                 *  				  get byte
                 *  0x0000			  get status, ask IR
                 *  0xB000			  get byte
                 *
                 *  		          after RX
                 *  0x8258			  !RX, !TX
                 *  0xCA81			  FIFO init
                 *
                 *
                 *  				  start TX
                 *  0x8278			  TX on
                 *
                 *  			      put byte
                 *  0x82bb		      put byte bb
                 *  0x0000			  get status ask IR
                 *
                 *  				  after TX
                 *  0x8258			  !RX, !TX
                 *
                 */

/**
 * RFM12_SendReceive sends one data word (16 bit) to RFM12 module
 * and receives 16 bits of data at the same time
 *
 * @param data - data word (16 bits) to send to RFM12
 * @param recv - pointer to data word (16 bits) which stores data received from RFM12
 */
uint16_t RFM12_SendReceive(uint16_t data)
{
  HAL_StatusTypeDef spi_status;
  uint16_t recv;
  spi_status = HAL_SPI_TransmitReceive(&hspi1, (uint8_t *) &data, (uint8_t *) &recv, 2, 10);
  if (spi_status != HAL_OK)
  {
    Ferr = 90;				   // SPI error
		return(0xFFFF);
  }
  return recv;

}


/*******************************************************************************
 * RFMInit:	Init SPI and RFM12, RFM12 in stand-by
 ******************************************************************************/
uint16_t RFMInit(void)
{
	uint32_t i;
	Ferr = 1;
	for(i=0; i<CMAXI; i++)
		RFM12_SendReceive(Cinit[i]);
	Fmod = 1;
	return(Ferr);
}

/*******************************************************************************
 * RFMGetStart: Moves RFM to get mode
 ******************************************************************************/
uint16_t RFMGetStart(void)
{
	uint32_t i;

	Ferr = 1;
	Fmod = 2;
	Fget = 1;
	Fidx = 0;
	Fham = 1;						   // starts with hi-part
	Flen = BUMAX;
	for(i=0; i<BUMAX; i++)			   // clear RX buffer
		Frxbu[i] = 0;

	__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_8);
	RFM12_SendReceive(0x82D8);  // RX 0n
	RFM12_SendReceive(0xCA81);  // FIFO init
	RFM12_SendReceive(0xCA83);  // FIFO activ
	RFM12_SendReceive(0x0000);  // get status, ask IR
	HAL_Delay(100);	            // start RX

	return(Ferr);
}


/*******************************************************************************
 * RFMGetStop: Stops RX mode
 ******************************************************************************/
uint16_t RFMGetStop(void)
{
	Ferr = 1;
	Fmod = 0;
	Fget = 8;
	__HAL_GPIO_EXTI_CLEAR_IT(RFM12_IRQ_Pin);
	RFM12_SendReceive(0x8258);		   // RX 0ff
	return(Ferr);
}


/*******************************************************************************
 * RFMGetReady: Isolates Fget variable
 ******************************************************************************/
uint16_t RFMGetReady(void)
{
	return(Fget);
}


/*******************************************************************************
 * RFMPutStart: Moves RFM to put mode
 ******************************************************************************/
uint16_t RFMPutStart(void)
{
	uint16_t crc16;

	Ferr = 1;
	Flen = Ftxbu[2];
	if((Flen < 4) ||(Flen > BUMAX))
		return(9);
	Fmod = 4;
	Fput = 1;
	Fidx = 1;
	Fint = 1;						   // header mode

	crc16 = CRCalc(Ftxbu,0,(Flen-2));
	Ftxbu[Flen-1] = (crc16 & 0xFF00) >> 8;
	Ftxbu[Flen]   =  crc16 & 0x00FF;

	__HAL_GPIO_EXTI_CLEAR_IT(RFM12_IRQ_Pin);
	RFM12_SendReceive(0x8278);		   // TX 0n
	HAL_Delay(100);					   // start TX

	RFM12_SendReceive(0xB8AA);		   // start header
	RFM12_SendReceive(0x0000);		   // get status, ask IR
	return(Ferr);
}

/*******************************************************************************
 * RFMGetStop: Stops TX mode
 ******************************************************************************/
uint16_t RFMPutStop(void)
{
	Ferr = 1;
	Fmod = 0;
	Fput = 8;
	__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_8);
	RFM12_SendReceive(0x8258);		   // TX 0ff
	return(Ferr);

}

/*******************************************************************************
 * RFMPutReady: Isolates Fput variable
 ******************************************************************************/
uint16_t RFMPutReady(void)
{
	return(Fput);
}

/*******************************************************************************
 * RFMIntRX: Handles interrupts from RFM in get mode
 ******************************************************************************/
char RFMIntRX(void)
{
	uint16_t wrd,crc16;
	char bhi,blo,byt;

	Fget = 2;						   // get is running
	if(Fidx <= Flen)
	{								   // get byte
		wrd = RFM12_SendReceive(0xB000);
		RFM12_SendReceive(0x0000);	   // get status, ask for IR
		byt = wrd & 0x00FF;
		if(Fham == 1)
		{
			Fhi = DecHam(byt);
		    if (Fhi == 0xFF)
		    {                         // Hamming Error
		        Fget = 8;             // Error-Code
		        Fmod = 3;             // End of Get
		        return(0);
		    }
			Fham = 2;
			return(0);

		}
		if(Fham == 2)
		{
			Flo = DecHam(byt);
		    if (Flo == 0xFF)
		    {                         // Hamming Error
		        Fget = 8;             // Error-Code
		        Fmod = 3;             // End of Get
		        return(0);
		    }
			Fham = 1;
		}
		byt = (Fhi << 4) | Flo;
		Frxbu[Fidx] = byt;
		if(Fidx == 2)
			Flen = Frxbu[2];		   // take the real msg length
		if(Flen > BUMAX)
			Flen = BUMAX;
		Fidx++;
		if(Fidx > Flen)
		{							   // last byte has arrived
			Fget = 3;				   // msg OK
			crc16 = CRCalc(Frxbu,0,(Flen-2));
			bhi   = (crc16 & 0xFF00) >> 8;
			blo   = crc16 & 0x00FF;
			if((Frxbu[Flen-1] != bhi) || (Frxbu[Flen] != blo))
				Fget = 9;			   // msg NOK
			Fmod = 3;				   // mode get is stopped
		}
	}
	return(0);
}

/*******************************************************************************
 * RFMIntTX: Handles interrupts from RFM in put mode
 ******************************************************************************/
void RFMIntTX(void)
{
	uint16_t cmd;
	uint16_t hilo;

	Fput = 2;
	switch(Fint)
	{
		case 1:						   // header
		{
			cmd = 0xB800 | Fhead[Fidx];
			RFM12_SendReceive(cmd);
			RFM12_SendReceive(0x0000);  // get status, ask for IR
			Fidx++;
			if(Fidx > 4)
			{
				Fint = 2;			   // do payload now
				Fidx = 0;
				Fham = 1;			   // start Hamming with hi-half
			}
		    break;
		}
		case 2:						   // payload
		{
			if(Fham == 1)
			{
				hilo = (Ftxbu[Fidx] & 0xF0) >> 4;
				cmd = 0xB800 | Fhval[hilo];
				RFM12_SendReceive(cmd);
				RFM12_SendReceive(0x0000);  // get status, ask for IR
				Fham = 2;
				break;
			}
			if(Fham == 2)
			{
				hilo = (Ftxbu[Fidx] & 0x0F);
				cmd = 0xB800 | Fhval[hilo];
				RFM12_SendReceive(cmd);
				RFM12_SendReceive(0x0000);  // get status, ask for IR
				Fham = 1;
				Fidx++;
				if(Fidx > Flen)
				{
					Fint = 3;
					Fidx = 0;
				}
			}
			break;
		}
		case 3:						   // tailer
		{
			RFM12_SendReceive(0xB8AA);
			RFM12_SendReceive(0x0000);  // get status, ask for IR
			Fput = 3;				   // put OK
			Fmod = 5;				   // stop interrupts
			if(Ferr != 1)
				Fput = 9;			   // put error
			break;
		}
	}
}

/*******************************************************************************
 * CRCalc: Calculates CRC16 of bu[] from start istrt till end iend
 ******************************************************************************/
uint16_t CRCalc(char bu[], uint16_t istrt,uint16_t iend)
{
	uint16_t poly=0xA001;
	uint16_t crc,n,i;
	char b;

	crc = 0xFFFF;
	for (i=istrt; i<=iend; i++)
	{
		b = bu[i];
		crc = crc ^ b;
		for(n=1; n<=8; n++)
		{
			if((crc & 0x0001) != 0)
				crc = (crc>>1) ^ poly;
			else
				crc = crc >> 1;
		}
	}
	return(crc);
}


/*******************************************************************************
 * DecHam: Decodes Hamming code
 ******************************************************************************/
uint8_t DecHam(uint8_t hambyt)
{

  switch (hambyt)
  {
  	  case 0x15:
  	  {
  		  return(0x00);
  	  }
  	  case 0x02:
  	  {
  		  return(0x01);
  	  }
  	  case 0x49:
  	  {
  		  return(0x02);
  	  }
  	  case 0x5E:
  	  {
  		  return(0x03);
  	  }
  	  case 0x64:
  	  {
  		  return(0x04);
  	  }
  	  case 0x73:
  	  {
  		  return(0x05);
  	  }
  	  case 0x38:
  	  {
  		  return(0x06);
  	  }
  	  case 0x2F:
  	  {
  		  return(0x07);
  	  }
  	  case 0xD0:
  	  {
  		  return(0x08);
  	  }
  	  case 0xC7:
  	  {
  		  return(0x09);
  	  }
  	  case 0x8C:
  	  {
  		  return(0x0A);
  	  }
  	  case 0x9B:
  	  {
  		  return(0x0B);
  	  }
  	  case 0xA1:
  	  {
  		  return(0x0C);
  	  }
  	  case 0xB6:
  	  {
  		  return(0x0D);
  	  }
  	  case 0xFD:
  	  {
  		  return(0x0E);
  	  }
  	  case 0xEA:
  	  {
  		  return(0x0F);
  	  }
  	  default:
  	  {
  		  return(0xFF);
  	  }
  }
}

