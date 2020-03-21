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
#include "H1DR1.h"
#include "mbport.h"

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
#define UART_2_ENABLED          ( 0 )   /*!< Set this to 1 to enable USART2 */

#if ( UART_1_ENABLED == 1 ) && ( UART_2_ENABLED == 1 )
#define UART_1_PORT             ( MB_UART_1 )
#define UART_2_PORT             ( MB_UART_2 )
#define UART_1_IDX              ( 0 )
#define UART_2_IDX              ( 1 )
#define NUARTS                  ( 2 )
#elif ( UART_1_ENABLED == 1 )
#define UART_1_PORT             ( MB_UART_1 )
#define UART_1_IDX              ( 0 )
#define NUARTS                  ( 1 )
#elif ( UART_2_ENABLED == 1 )
#define UART_2_PORT             ( MB_UART_2 )
#define UART_2_IDX              ( 0 )
#define NUARTS                  ( 1 )
#else
#define NUARTS                  ( 0 )
#endif

#define RS_485_UART_1_INIT(  )	\
do { \
} while( 0 )

#define RS_485_UART_1_ENABLE_TX(  )	\
do {\
    /* not implemented yet */\
} while( 0 )

#define RS_485_UART_1_DISABLE_TX(  ) \
do { \
    /* not implemented yet */ \
} while( 0 )

#define RS_485_UART_2_INIT(  )\
do { \
    /* not implemented yet */ \
} while( 0 )

#define RS_485_UART_2_ENABLE_TX(  )	\
do { \
    /* not implemented yet */ \
} while( 0 )

#define RS_485_UART_2_DISABLE_TX(  ) \
do { \
    /* not implemented yet */ \
} while( 0 )

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

/* ----------------------- Static functions ---------------------------------*/
void            vMBPUSART1ISR( void ) __attribute__ ( ( __interrupt__ ) );
void     prvvMBPUSART1_TXE_ISR( void );  //STATIC
void     prvvMBPUSART1_TC_ISR( void );   //STATIC
void     prvvMBPUSART1_RXNE_ISR( void );  //STATIC

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
#if UART_2_ENABLED == 1
        RS_485_UART_2_INIT(  );
        RS_485_UART_2_DISABLE_TX(  );
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
        case MB_PARITY_N:
            Parity = UART_PARITY_NONE;
            break;
        case MB_PARITY_O:
            Parity = UART_PARITY_ODD;
            break;
        case MB_PARITY_E:
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
                    MB_PORT_Init(BaudRate, DataBits, Parity, StopBit);
									
                    /* Disable receive and transmit interrupts from the beginning */
                    //USART_ITConfig( USART1, USART_IT_RXNE, DISABLE );
                    //USART_ITConfig( USART1, USART_IT_TXE, DISABLE );

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
#if UART_2_ENABLED == 1
            case UART_2_PORT:
                if( IDX_INVALID == xSerialHdls[UART_2_IDX].ubIdx )
                {
                    /* Configure STM32 USART2 */
                    USART_Init( USART2, &USART_InitStructure );

                    /* Enable the USART2 */
                    USART_Cmd( USART2, ENABLE );

                    /* Disable receive and transmit interrupts from the beginning */
                    USART_ITConfig( USART2, USART_IT_RXNE, DISABLE );
                    USART_ITConfig( USART2, USART_IT_TXE, DISABLE );

                    /* Setup handle to uart */
                    *pxSerialHdl = &xSerialHdls[UART_2_IDX];
                    xSerialHdls[UART_2_IDX].ubIdx = UART_2_IDX;
                    xSerialHdls[UART_2_IDX].xMBMHdl = xMBMHdl;

                    /* Every thing is ok */
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
#if UART_2_ENABLED == 1
        case UART_2_IDX:
            if( ( NULL == pxSerialIntHdl->pbMBMTransmitterEmptyFN ) && ( NULL == pxSerialIntHdl->pvMBMReceiveFN ) )
            {
                /* Close USART 1 */
                USART_Cmd( USART2, DISABLE );
                USART_DeInit( USART2 );
                /* Force RS485 back to receive mode */
                RS_485_UART_2_DISABLE_TX(  );
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
                /* Enable USART 1 tx interrupt */
								__HAL_UART_DISABLE_IT(&huart1, UART_IT_TXE);
								RS485_RECEIVER_DIS();
                break;
#endif
#if UART_2_ENABLED == 1
            case UART_2_IDX:
                /* RS485 transmit mode */
                RS_485_UART_2_ENABLE_TX(  );
                /* Enable USART 2 tx interrupt */
                USART_ITConfig( USART2, USART_IT_TXE, ENABLE );
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
								__HAL_UART_DISABLE_IT(&huart1, UART_IT_TC);
                break;
#endif
#if UART_2_ENABLED == 1
            case UART_2_IDX:
                /* Disable transmit register empty interrupt */
                USART_ITConfig( USART2, USART_IT_TXE, DISABLE );
                /* Enable transmit complete interrupt */
                USART_ITConfig( USART2, USART_IT_TC, ENABLE );
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
								RS485_RECEIVER_EN();
                break;
#endif
#if UART_2_ENABLED == 1
            case UART_2_IDX:
                /* Enable USART 2 receive interrupt */
                USART_ITConfig( USART2, USART_IT_RXNE, ENABLE );
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
								RS485_RECEIVER_DIS();
                break;
#endif
#if UART_2_ENABLED == 1
            case UART_2_IDX:
                /* Disable USART 2 receive interrupt */
                USART_ITConfig( USART2, USART_IT_RXNE, DISABLE );
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

//#if UART_1_ENABLED == 1
//void
//vMBPUSART1ISR( void )
//{
//    /* Check for receive interrupt */
//    if( USART_GetITStatus( USART1, UART_IT_RXNE ) != RESET )
//    {HAL_UART_GetState(&huart1);
//        /* Handle data incomming data in modbus functions. Interrupt flag is 
//         * cleared when byte is read in receive register.
//         */
//        prvvMBPUSART1_RXNE_ISR(  );
//    }
//    /* Check for transmit interrupt */
//    if( USART_GetITStatus( USART1, USART_IT_TXE ) != RESET )
//    {
//        /* Handle transmission of data in modbus functions. Interrupt flags are
//         * cleared when new byte is written to transmit register.
//         */
//        prvvMBPUSART1_TXE_ISR(  );
//    }
//    /* Check for transmit complete */
//    if( USART_GetITStatus( USART1, UART_IT_TC ) != RESET )
//    {
//        /* Handle transmit complete in modbus library */
//        prvvMBPUSART1_TC_ISR(  );
//    }
//}

void
prvvMBPUSART1_TXE_ISR( void )
{
    MBP_ASSERT( IDX_INVALID != xSerialHdls[UART_1_IDX].ubIdx );
    BOOL            bHasMoreData = TRUE;
    UBYTE           ubTxByte;

    if( NULL != xSerialHdls[UART_1_IDX].pbMBMTransmitterEmptyFN )
    {
        bHasMoreData = xSerialHdls[UART_1_IDX].pbMBMTransmitterEmptyFN( xSerialHdls[UART_1_IDX].xMBMHdl, &ubTxByte );
    }
    if( !bHasMoreData )
    {
        xSerialHdls[UART_1_IDX].pbMBMTransmitterEmptyFN = NULL;
        /* The transmitter is disabled when the last frame has been sent.
         * This is necessary for RS485 with a hald-duplex bus.
         */
					__HAL_UART_DISABLE_IT(&huart1, UART_IT_TXE);
					__HAL_UART_ENABLE_IT(&huart1, UART_IT_TC);
    }
    else
    {
        /* Transmit byte on USART */
        HAL_UART_Transmit_IT( &huart1, &ubTxByte, sizeof (ubTxByte) );
    }
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
		//UART_FLAG_NE
    if( ((&huart1)->Instance->ISR, UART_FLAG_NE) )
    {
        fs |= 2;
    }
		//UART_FLAG_FE
    if( ((&huart1)->Instance->ISR, UART_FLAG_FE) )
    {
        fs |= 4;
    }
		//UART_FLAG_PE
    if( ((&huart1)->Instance->ISR, UART_FLAG_PE) )
    {
        fs |= 8;
    }

    /* Receive byte from USART1 */
		HAL_UART_Receive_IT(&huart1, &ubUDR, sizeof (ubUDR));
    //ubUDR = USART_ReceiveData( USART1 );

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


///* void 
//prrvUSARTTxISR( void )           //STATIC
//{
//    BOOL            bHasMoreData = TRUE;
//    UBYTE           ubTxByte;

//    if( NULL != xSerialHdls[0].pbMBPTransmitterEmptyFN )
//    {
//        bHasMoreData = xSerialHdls[0].pbMBPTransmitterEmptyFN( xSerialHdls[0].xMBMHdl, &ubTxByte );
//    }
//    if( !bHasMoreData )
//    {
//        xSerialHdls[0].pbMBPTransmitterEmptyFN = NULL;
//        
//        /* TODO: Disable the transmitter. */
///*    }
//    else
//    {
//        /* TODO: Place byte ubTxByte in the UART data register. */
///*    }
//}

// void 
//prrvUSARTRxISR( void )           //STATIC
//{
//    UBYTE           ubUDR = 0; /* TODO: Get byte from UART. */;

///*    MBP_ASSERT( IDX_INVALID != xSerialHdls[0].ubIdx );
//    if( NULL != xSerialHdls[0].pvMBPReceiveFN )
//    {
//        xSerialHdls[0].pvMBPReceiveFN( xSerialHdls[0].xMBMHdl, ubUDR );
//    }
//}
//*/
