/* 
 * MODBUS Library: Skeleton port
 * Copyright (c) 2008 Christian Walter <wolti@sil.at>
 * All rights reserved.
 *
 * $Id: mbportserial.c,v 1.1 2008-04-06 07:46:23 cwalter Exp $
 */

/* ----------------------- System includes ----------------------------------*/
#include <stdlib.h>

/* ----------------------- Platform includes --------------------------------*/
#include "mbport.h"
#include "H1DR1.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include "common/mbtypes.h"
#include "common/mbportlayer.h"
#include "common/mbframe.h"
#include "common/mbutils.h"

/* ----------------------- Defines ------------------------------------------*/
#define IDX_INVALID				( 255 )
#define UART_BAUDRATE_MIN		( 300 )
#define UART_BAUDRATE_MAX		( 115200 )


#define UART_1_ENABLED          ( 1 )   /*!< Set this to 1 to enable USART1 */

#if ( UART_1_ENABLED == 1 )
#define UART_1_PORT             ( MB_UART_1 )
#define UART_1_IDX              ( 0 )
#define NUARTS                  ( 1 )
#endif

#define RS_485_UART_1_INIT(  )	\
do { \
} while( 0 )

#define RS_485_UART_1_ENABLE_TX()	 RS485_RECEIVER_DIS()

#define RS_485_UART_1_DISABLE_TX(  ) RS485_RECEIVER_EN()

/* ----------------------- Defines ------------------------------------------*/
/* ----------------------- Defines (Internal - Don't change) ----------------*/
#define HDL_RESET( x ) do { \
	( x )->ubIdx = IDX_INVALID; \
	( x )->pbMBMTransmitterEmptyFN = NULL; \
	( x )->pvMBMReceiveFN = NULL; \
	( x )->xMBMHdl = MB_HDL_INVALID; \
} while( 0 );

/* ----------------------- Type definitions ---------------------------------*/
typedef struct
{
    UBYTE           ubIdx;
    pbMBPSerialTransmitterEmptyAPIV1CB pbMBMTransmitterEmptyFN;
    pvMBPSerialReceiverAPIV1CB pvMBMReceiveFN;
    xMBHandle       xMBMHdl;
} xSerialHandle;

/* ----------------------- Static variables ---------------------------------*/
STATIC xSerialHandle xSerialHdls[NUARTS];
STATIC BOOL     bIsInitalized = FALSE;
#define MBM_SER_PDU_SIZE_MAX            ( 256 ) /*!< Maximum size of a MODBUS RTU frame. */

/* ----------------------- Static functions ---------------------------------*/
void     prvvMBPUSART1_TC_ISR( void );  
void     prvvMBPUSART1_RXNE_ISR( void );  

/* ----------------------- Start implementation -----------------------------*/

eMBErrorCode
eMBPSerialInit( xMBPSerialHandle * pxSerialHdl, UCHAR ucPort, ULONG ulBaudRate,
                UCHAR ucDataBits, eMBSerialParity eParity, UCHAR ucStopBits, xMBHandle xMBMHdl )
{
    eMBErrorCode    eStatus = MB_ENOERR;
    UBYTE           ubIdx;
	
    // added by Hexabitz H1DR1 firmware
		uint32_t BaudRate;
		uint32_t DataBits;
		uint32_t Parity;
		uint32_t StopBit;
		// end of addition

    MBP_ENTER_CRITICAL_SECTION(  );
    if( !bIsInitalized )
    {
        for( ubIdx = 0; ubIdx < MB_UTILS_NARRSIZE( xSerialHdls ); ubIdx++ )
        {
            HDL_RESET( &xSerialHdls[ubIdx] );
        }
#if UART_1_ENABLED == 1
        RS_485_UART_1_INIT(  );
        RS_485_UART_1_DISABLE_TX(  );
#endif
        bIsInitalized = TRUE;
    }

    if( ( MB_HDL_INVALID == xMBMHdl ) || ( NULL == pxSerialHdl ) )
    {
        eStatus = MB_EINVAL;
    }
    else
    {
        eStatus = MB_ENORES;

        /* Setup baudrate */
        if( ( ulBaudRate > UART_BAUDRATE_MIN ) && ( ulBaudRate < UART_BAUDRATE_MAX ) )
        {
            BaudRate = ulBaudRate;
        }
        else
        {
            eStatus = MB_EINVAL;
        }

        /* Setup stopbits */
        switch ( ucStopBits )
        {
        case 1:
            StopBit = UART_STOPBITS_1;
            break;
        case 2:
            StopBit = UART_STOPBITS_2;
            break;
        default:
            eStatus = MB_EINVAL;
            break;
        }

        /* For STM32 parity is placed on bit 9 (MSB)
         * Select correct number of databits */
        if( ucDataBits == 8 )
        {
            if( eParity != MB_PAR_NONE )
            {
                DataBits = UART_WORDLENGTH_9B;
            }
            else
            {
                DataBits = UART_WORDLENGTH_8B;
            }
        }
        else
        {
            if( eParity != MB_PAR_NONE )
            {
                DataBits = UART_WORDLENGTH_8B;
            }
            else
            {
                eStatus = MB_EINVAL;
            }
        }

        /* Setup parity */
        switch ( eParity )
        {
        case MB_PAR_NONE:
            Parity = UART_PARITY_NONE;
            break;
        case MB_PAR_ODD:
            Parity = UART_PARITY_ODD;
            break;
        case MB_PAR_EVEN:
            Parity = UART_PARITY_EVEN;
            break;
        default:
            eStatus = MB_EINVAL;
            break;
        }

        if( eStatus != MB_EINVAL )
        {
            switch ( ucPort )
            {
#if UART_1_ENABLED == 1
            case UART_1_PORT:
                if( IDX_INVALID == xSerialHdls[UART_1_IDX].ubIdx )
                {
                    /* Configure  USART1 */
                    MB_PORT_Init(BaudRate, DataBits, Parity, UART_STOPBITS_1);
										//HAL_RS485Ex_Init(&huart1, UART_DE_POLARITY_HIGH, 1, 1);
									
                    /* Disable receive and transmit interrupts from the beginning */
                    __HAL_UART_DISABLE_IT(&huart1, UART_IT_RXNE);
										__HAL_UART_DISABLE_IT(&huart1, UART_IT_TC);

                    /* Setup handle to uart */
                    *pxSerialHdl = &xSerialHdls[UART_1_IDX];
                    xSerialHdls[UART_1_IDX].ubIdx = UART_1_IDX;
                    xSerialHdls[UART_1_IDX].xMBMHdl = xMBMHdl;

                    /* Everything is ok */
                    eStatus = MB_ENOERR;
                }
                else
                {
                    eStatus = MB_ENORES;
                }
                break;
#endif
            default:
                break;
            }
        }
    }
    MBP_EXIT_CRITICAL_SECTION(  );
    return eStatus;
}

eMBErrorCode
eMBPSerialClose( xMBPSerialHandle xSerialHdl )
{
    eMBErrorCode    eStatus = MB_EINVAL;
    xSerialHandle  *pxSerialIntHdl = xSerialHdl;

    MBP_ENTER_CRITICAL_SECTION(  );
    if( MB_IS_VALID_HDL( pxSerialIntHdl, xSerialHdls ) )
    {
        switch ( pxSerialIntHdl->ubIdx )
        {
#if UART_1_ENABLED == 1
        case UART_1_IDX:
            if( ( NULL == pxSerialIntHdl->pbMBMTransmitterEmptyFN ) && ( NULL == pxSerialIntHdl->pvMBMReceiveFN ) )
            {
                /* Close USART 1 */
                HAL_UART_DeInit(&huart1);
                /* Force RS485 back to receive mode */
                RS_485_UART_1_DISABLE_TX(  );
                /* Reset handle */
                HDL_RESET( pxSerialIntHdl );
                /* No error */
                eStatus = MB_ENOERR;
            }
            else
            {
                eStatus = MB_EIO;
            }
            break;
#endif
        default:
            MBP_ASSERT( 0 );
            break;
        }
    }
    MBP_EXIT_CRITICAL_SECTION(  );
    return eStatus;
}

eMBErrorCode
eMBPSerialTxEnable( xMBPSerialHandle xSerialHdl, pbMBPSerialTransmitterEmptyCB pbMBMTransmitterEmptyFN )
{
    eMBErrorCode    eStatus = MB_EINVAL;
    xSerialHandle  *pxSerialIntHdl = xSerialHdl;
	
		// next lines added for copying frame packet to buffer
		MBP_ASSERT( IDX_INVALID != xSerialHdls[UART_1_IDX].ubIdx );
    BOOL      bHasMoreData = TRUE;      
    UBYTE     ubTxByte[MBM_SER_PDU_SIZE_MAX]={0};
		UBYTE 		FrameCnt=0;

    MBP_ENTER_CRITICAL_SECTION(  );
    if( MB_IS_VALID_HDL( pxSerialIntHdl, xSerialHdls ) )
    {
        eStatus = MB_ENOERR;
        if( NULL != pbMBMTransmitterEmptyFN )
        {
            MBP_ASSERT( NULL == pxSerialIntHdl->pbMBMTransmitterEmptyFN );
            pxSerialIntHdl->pbMBMTransmitterEmptyFN = pbMBMTransmitterEmptyFN;
            switch ( pxSerialIntHdl->ubIdx )
            {
#if UART_1_ENABLED == 1
            case UART_1_IDX:
                /* RS485 transmit mode */
                RS_485_UART_1_ENABLE_TX(  );
                /* Copy the frame to buffer */
								if( NULL != xSerialHdls[UART_1_IDX].pbMBMTransmitterEmptyFN )
									{
										while (bHasMoreData){
											bHasMoreData = xSerialHdls[UART_1_IDX].pbMBMTransmitterEmptyFN( xSerialHdls[UART_1_IDX].xMBMHdl, &ubTxByte[FrameCnt] );
											FrameCnt++;
										}
										xSerialHdls[UART_1_IDX].pbMBMTransmitterEmptyFN = NULL;
									}
								// send the buffer once
								HAL_UART_Transmit_IT( &huart1, ubTxByte, FrameCnt-1 );
                break;
#endif
            default:
                MBP_ASSERT( 0 );
            }

        }
        else
        {
            pxSerialIntHdl->pbMBMTransmitterEmptyFN = NULL;
            /* The transmitter is disable when the last frame has been sent.
             * This is necessary for RS485 with a half-duplex bus.
             */
            switch ( pxSerialIntHdl->ubIdx )
            {
#if UART_1_ENABLED == 1
            case UART_1_IDX:
                /* Disable transmit register empty interrupt */
								__HAL_UART_DISABLE_IT(&huart1, UART_IT_TXE);
                /* Enable transmit complete interrupt */
								__HAL_UART_ENABLE_IT(&huart1, UART_IT_TC);
                break;
#endif
            default:
                MBP_ASSERT( 0 );
            }
        }
    }
    MBP_EXIT_CRITICAL_SECTION(  );
    return eStatus;
}

eMBErrorCode
eMBPSerialRxEnable( xMBPSerialHandle xSerialHdl, pvMBPSerialReceiverCB pvMBMReceiveFN )
{
    eMBErrorCode    eStatus = MB_EINVAL;
    xSerialHandle  *pxSerialIntHdl = xSerialHdl;

    MBP_ENTER_CRITICAL_SECTION(  );
    if( MB_IS_VALID_HDL( pxSerialIntHdl, xSerialHdls ) )
    {
        eStatus = MB_ENOERR;
        if( NULL != pvMBMReceiveFN )
        {
            MBP_ASSERT( NULL == pxSerialIntHdl->pvMBMReceiveFN );
            pxSerialIntHdl->pvMBMReceiveFN = pvMBMReceiveFN;
            switch ( pxSerialIntHdl->ubIdx )
            {
#if UART_1_ENABLED == 1
            case UART_1_IDX:
                /* Enable USART 1 receive interrupt */
                __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
                break;
#endif
            default:
                MBP_ASSERT( 0 );
            }
        }
        else
        {
            pxSerialIntHdl->pvMBMReceiveFN = NULL;
            switch ( pxSerialIntHdl->ubIdx )
            {
#if UART_1_ENABLED == 1
            case UART_1_IDX:
                /* Disable USART 1 receive interrupt */
                __HAL_UART_DISABLE_IT(&huart1, UART_IT_RXNE);
								
                break;
#endif
            default:
                MBP_ASSERT( 0 );
            }
        }
    }
    MBP_EXIT_CRITICAL_SECTION(  );
    return eStatus;
}

/* USART 1 Transmit Complete interrupt */
void
prvvMBPUSART1_TC_ISR( void )
{
    /* Back to receive mode */
    RS_485_UART_1_DISABLE_TX(  );
    /* Transmission complete. Disable interrupt */
    __HAL_UART_DISABLE_IT(&huart1, UART_IT_TC);
}

/* USART 1 Receive interrupt */
void
prvvMBPUSART1_RXNE_ISR( void )
{
    UBYTE           ubUDR;
    FlagStatus      fs;

    /* Read current flagstatus */
    fs = RESET;

    if( ((&huart1)->Instance->ISR, UART_FLAG_ORE) )
    {
        fs |= 1;
    }
    if( ((&huart1)->Instance->ISR, UART_FLAG_NE) )
    {
        fs |= 2;
    }
    if( ((&huart1)->Instance->ISR, UART_FLAG_FE) )
    {
        fs |= 4;
    }
    if( ((&huart1)->Instance->ISR, UART_FLAG_PE) )
    {
        fs |= 8;
    }

    /* Receive byte from USART1 */
		HAL_UART_Receive_IT(&huart1, &ubUDR, sizeof (ubUDR));

    /* Send data to modbus functions
     * if no error */
    if( fs == RESET )
    {
        /* Pass received data on to modbuslib */
        MBP_ASSERT( IDX_INVALID != xSerialHdls[UART_1_IDX].ubIdx );
        if( NULL != xSerialHdls[UART_1_IDX].pvMBMReceiveFN )
        {
            xSerialHdls[UART_1_IDX].pvMBMReceiveFN( xSerialHdls[UART_1_IDX].xMBMHdl, ubUDR );
        }
    }
}


/************************ (C) MODIFIED BY HEXABITZ *****END OF FILE****/
