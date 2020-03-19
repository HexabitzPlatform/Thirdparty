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
STATIC void     prvvMBPUSART1_TXE_ISR( void );
STATIC void     prvvMBPUSART1_TC_ISR( void );
STATIC void     prvvMBPUSART1_RXNE_ISR( void );

void            vMBPUSART2ISR( void ) __attribute__ ( ( __interrupt__ ) );
STATIC void     prvvMBPUSART2_TXE_ISR( void );
STATIC void     prvvMBPUSART2_TC_ISR( void );
STATIC void     prvvMBPUSART2_RXNE_ISR( void );

/* ----------------------- Start implementation -----------------------------*/

eMBErrorCode
eMBPSerialInit( xMBPSerialHandle * pxSerialHdl, UCHAR ucPort, ULONG ulBaudRate,
                UCHAR ucDataBits, eMBSerialParity eParity, UCHAR ucStopBits, xMBHandle xMBMHdl )
{
    eMBErrorCode    eStatus = MB_ENOERR;
    UBYTE           ubIdx;
	
    // added by Hexabitz firmware
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
                    MB_PORT_Init(ulBaudRate, DataBits, Parity, StopBit);
									
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
        if( ( pxSerialIntHdl->pbMBPTransmitterEmptyFN == NULL ) && ( pxSerialIntHdl->pvMBPReceiveFN == NULL ) )
        {
            /* TODO: Close the serial port here. */
            HAL_UART_DeInit(&huart1);
						eStatus = MB_ENOERR;
        }
        else
        {
            eStatus = MB_EAGAIN;
        }
    }
    MBP_EXIT_CRITICAL_SECTION(  );
    return eStatus;
}

eMBErrorCode
eMBPSerialTxEnable( xMBPSerialHandle xSerialHdl, pbMBPSerialTransmitterEmptyCB pbMBPTransmitterEmptyFN )
{
    eMBErrorCode    eStatus = MB_EINVAL;
    xSerialHandle  *pxSerialIntHdl = xSerialHdl;

    MBP_ENTER_CRITICAL_SECTION(  );
    if( MB_IS_VALID_HDL( pxSerialIntHdl, xSerialHdls ) )
    {
        eStatus = MB_ENOERR;
        if( NULL != pbMBPTransmitterEmptyFN )
        {
            MBP_ASSERT( NULL == pxSerialIntHdl->pbMBPTransmitterEmptyFN );
            pxSerialIntHdl->pbMBPTransmitterEmptyFN = pbMBPTransmitterEmptyFN;
            
            /* TODO: Enable the transmitter. */   
						__HAL_UART_ENABLE_IT(&huart1, UART_IT_TXE);
						RS485_RECEIVER_DIS();

        }
        else
        {
            pxSerialIntHdl->pbMBPTransmitterEmptyFN = NULL;
            
            /* TODO: Disable the transmitter. Make sure that all characters have been
             * transmitted in case you do any buffering internally.
             */
						__HAL_UART_DISABLE_IT(&huart1, UART_IT_TXE);
        }
    }
    MBP_EXIT_CRITICAL_SECTION(  );
    return eStatus;
}

eMBErrorCode
eMBPSerialRxEnable( xMBPSerialHandle xSerialHdl, pvMBPSerialReceiverCB pvMBPReceiveFN )
{
    eMBErrorCode    eStatus = MB_EINVAL;
    xSerialHandle  *pxSerialIntHdl = xSerialHdl;

    MBP_ENTER_CRITICAL_SECTION(  );
    if( MB_IS_VALID_HDL( pxSerialIntHdl, xSerialHdls ) )
    {
        eStatus = MB_ENOERR;
        if( NULL != pvMBPReceiveFN )
        {
            MBP_ASSERT( NULL == pxSerialIntHdl->pvMBPReceiveFN );
            pxSerialIntHdl->pvMBPReceiveFN = pvMBPReceiveFN;
            
            /* TODO: Enable the receiver. */
						__HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
						RS485_RECEIVER_EN();

        }
        else
        {
            pxSerialIntHdl->pvMBPReceiveFN = NULL;
            
            /* TODO: Disable the receiver. */
						__HAL_UART_DISABLE_IT(&huart1, UART_IT_RXNE);
						RS485_RECEIVER_DIS();
        }
    }
    MBP_EXIT_CRITICAL_SECTION(  );
    return eStatus;
}

 void 
prrvUSARTTxISR( void )           //STATIC
{
    BOOL            bHasMoreData = TRUE;
    UBYTE           ubTxByte;

    if( NULL != xSerialHdls[0].pbMBPTransmitterEmptyFN )
    {
        bHasMoreData = xSerialHdls[0].pbMBPTransmitterEmptyFN( xSerialHdls[0].xMBMHdl, &ubTxByte );
    }
    if( !bHasMoreData )
    {
        xSerialHdls[0].pbMBPTransmitterEmptyFN = NULL;
        
        /* TODO: Disable the transmitter. */
    }
    else
    {
        /* TODO: Place byte ubTxByte in the UART data register. */
    }
}

 void 
prrvUSARTRxISR( void )           //STATIC
{
    UBYTE           ubUDR = 0; /* TODO: Get byte from UART. */;

    MBP_ASSERT( IDX_INVALID != xSerialHdls[0].ubIdx );
    if( NULL != xSerialHdls[0].pvMBPReceiveFN )
    {
        xSerialHdls[0].pvMBPReceiveFN( xSerialHdls[0].xMBMHdl, ubUDR );
    }
}
