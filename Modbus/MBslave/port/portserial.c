/*
 * FreeModbus Libary: BARE Port
 * Copyright (C) 2006 Christian Walter <wolti@sil.at>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 * File: $Id$
 */

#include "port.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"
#include "H1DR1.h"

/* ----------------------- static functions ---------------------------------*/
static void prvvUARTTxReadyISR( void );
static void prvvUARTRxISR( void );

/* ----------------------- Start implementation -----------------------------*/
void
vMBPortSerialEnable( BOOL xRxEnable, BOOL xTxEnable )
{
    /* If xRXEnable enable serial receive interrupts. If xTxENable enable
     * transmitter empty interrupts.
     */
if (xRxEnable)
{
__HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
RS485_RECEIVER_EN();
}
else {
__HAL_UART_DISABLE_IT(&huart1, UART_IT_RXNE);
RS485_RECEIVER_DIS();
}
if (xTxEnable)
{
__HAL_UART_ENABLE_IT(&huart1, UART_IT_TXE);
RS485_RECEIVER_DIS();
}
else {
__HAL_UART_DISABLE_IT(&huart1, UART_IT_TXE);
}
}

BOOL
xMBPortSerialInit( UCHAR ucPORT, ULONG ulBaudRate, UCHAR ucDataBits, eMBParity eParity )
{
 /* 
  Do nothing, Initialization is handled by MB_PORT_Init() 
  Fixed port, varying baudrate, databit and parity  
  */
	uint16_t baud_rate;
	uint32_t data_bits;
	uint32_t parity_bit;
	/* Verify the value of baud rate */
	if (ulBaudRate!=9600  && ulBaudRate!=18200  && ulBaudRate!=38400){
		return FALSE;
	}else
	{
	switch(ulBaudRate)
	{
		case(9600): baud_rate=9600; break;
		case(18200): baud_rate=18200; break;
		case(36400): baud_rate=36400; break;
		default: baud_rate=9600; break;
	}
	}	
	/* Verify the value of baud rate */
	if (data_bits!=UART_WORDLENGTH_7B  && data_bits!=UART_WORDLENGTH_8B){
		return FALSE;
	}else
	{
	switch(data_bits)
	{
		case(UART_WORDLENGTH_7B): data_bits=UART_WORDLENGTH_7B; break;
		case(UART_WORDLENGTH_8B): data_bits=UART_WORDLENGTH_8B; break;
		default: data_bits=UART_WORDLENGTH_8B; break;
	}
	}
	/* Verify the value of parity */
	if (eParity==UART_PARITY_NONE  || eParity==UART_PARITY_EVEN  || eParity==UART_PARITY_ODD){
			switch(eParity)
	{
		case(UART_PARITY_NONE): parity_bit=UART_PARITY_NONE; break;
		case(UART_PARITY_EVEN): parity_bit=UART_PARITY_EVEN; break;
		case(UART_PARITY_ODD): parity_bit=UART_PARITY_ODD; break;
		default: parity_bit=UART_PARITY_NONE; break;
	}
	
	MB_PORT_Init(baud_rate,data_bits, parity_bit);
	
		return TRUE;
	}else
	{
	return FALSE;
	}
}

BOOL
xMBPortSerialPutByte( CHAR ucByte )
{
    /* Put a byte in the UARTs transmit buffer. This function is called
     * by the protocol stack if pxMBFrameCBTransmitterEmpty( ) has been
     * called. */
   return (HAL_OK == HAL_UART_Transmit(&huart1, (uint8_t*)&ucByte, 1, 10));
}

BOOL
xMBPortSerialGetByte( CHAR * pucByte )
{
    /* Return the byte in the UARTs receive buffer. This function is called
     * by the protocol stack after pxMBFrameCBByteReceived( ) has been called.
     */
  *pucByte = (uint8_t)(huart1.Instance->RDR & (uint8_t)0x00FF);  
  return TRUE;
}

/* Create an interrupt handler for the transmit buffer empty interrupt
 * (or an equivalent) for your target processor. This function should then
 * call pxMBFrameCBTransmitterEmpty( ) which tells the protocol stack that
 * a new character can be sent. The protocol stack will then call 
 * xMBPortSerialPutByte( ) to send the character.
 */
static void prvvUARTTxReadyISR( void )
{
    pxMBFrameCBTransmitterEmpty(  );
}

/* Create an interrupt handler for the receive interrupt for your target
 * processor. This function should then call pxMBFrameCBByteReceived( ). The
 * protocol stack will then call xMBPortSerialGetByte( ) to retrieve the
 * character.
 */
static void prvvUARTRxISR( void )
{
    pxMBFrameCBByteReceived(  );
}
