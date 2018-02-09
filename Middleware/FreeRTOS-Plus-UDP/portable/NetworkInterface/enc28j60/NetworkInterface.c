/*
 * FreeRTOS+TCP Labs Build 160919 (C) 2016 Real Time Engineers ltd.
 * Authors include Hein Tibosch and Richard Barry
 *
 *******************************************************************************
 ***** NOTE ******* NOTE ******* NOTE ******* NOTE ******* NOTE ******* NOTE ***
 ***                                                                         ***
 ***                                                                         ***
 ***   FREERTOS+TCP IS STILL IN THE LAB (mainly because the FTP and HTTP     ***
 ***   demos have a dependency on FreeRTOS+FAT, which is only in the Labs    ***
 ***   download):                                                            ***
 ***                                                                         ***
 ***   FreeRTOS+TCP is functional and has been used in commercial products   ***
 ***   for some time.  Be aware however that we are still refining its       ***
 ***   design, the source code does not yet quite conform to the strict      ***
 ***   coding and style standards mandated by Real Time Engineers ltd., and  ***
 ***   the documentation and testing is not necessarily complete.            ***
 ***                                                                         ***
 ***   PLEASE REPORT EXPERIENCES USING THE SUPPORT RESOURCES FOUND ON THE    ***
 ***   URL: http://www.FreeRTOS.org/contact  Active early adopters may, at   ***
 ***   the sole discretion of Real Time Engineers Ltd., be offered versions  ***
 ***   under a license other than that described below.                      ***
 ***                                                                         ***
 ***                                                                         ***
 ***** NOTE ******* NOTE ******* NOTE ******* NOTE ******* NOTE ******* NOTE ***
 *******************************************************************************
 *
 * FreeRTOS+TCP can be used under two different free open source licenses.  The
 * license that applies is dependent on the processor on which FreeRTOS+TCP is
 * executed, as follows:
 *
 * If FreeRTOS+TCP is executed on one of the processors listed under the Special
 * License Arrangements heading of the FreeRTOS+TCP license information web
 * page, then it can be used under the terms of the FreeRTOS Open Source
 * License.  If FreeRTOS+TCP is used on any other processor, then it can be used
 * under the terms of the GNU General Public License V2.  Links to the relevant
 * licenses follow:
 *
 * The FreeRTOS+TCP License Information Page: http://www.FreeRTOS.org/tcp_license
 * The FreeRTOS Open Source License: http://www.FreeRTOS.org/license
 * The GNU General Public License Version 2: http://www.FreeRTOS.org/gpl-2.0.txt
 *
 * FreeRTOS+TCP is distributed in the hope that it will be useful.  You cannot
 * use FreeRTOS+TCP unless you agree that you use the software 'as is'.
 * FreeRTOS+TCP is provided WITHOUT ANY WARRANTY; without even the implied
 * warranties of NON-INFRINGEMENT, MERCHANTABILITY or FITNESS FOR A PARTICULAR
 * PURPOSE. Real Time Engineers Ltd. disclaims all conditions and terms, be they
 * implied, expressed, or statutory.
 *
 * 1 tab == 4 spaces!
 *
 * http://www.FreeRTOS.org
 * http://www.FreeRTOS.org/plus
 * http://www.FreeRTOS.org/labs
 *
 */

/* Standard includes. */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>

/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

/* FreeRTOS+TCP includes. */
#include "FreeRTOS_UDP_IP.h"
#include "FreeRTOS_Sockets.h"
#include "FreeRTOS_IP_Private.h"
#include "NetworkBufferManagement.h"
#include "NetworkInterface.h"

#include "enc28j60.h"

#ifndef    PHY_LS_HIGH_CHECK_TIME_MS
    /* Check if the LinkSStatus in the PHY is still high after 15 seconds of not
    receiving packets. */
    #define PHY_LS_HIGH_CHECK_TIME_MS    15000
#endif

#ifndef    PHY_LS_LOW_CHECK_TIME_MS
    /* Check if the LinkSStatus in the PHY is still low every second. */
    #define PHY_LS_LOW_CHECK_TIME_MS    1000
#endif

/* Interrupt events to process.  Currently only the Rx event is processed
although code for other events is included to allow for possible future
expansion. */
#define EMAC_IF_RX_EVENT                1UL
#define EMAC_IF_TX_EVENT                2UL
#define EMAC_IF_ERR_EVENT               4UL
#define EMAC_IF_ALL_EVENT               ( EMAC_IF_RX_EVENT | EMAC_IF_TX_EVENT | EMAC_IF_ERR_EVENT )

#define ETHERNET_CONF_PHY_ADDR    BOARD_GMAC_PHY_ADDR

#ifdef ipconfigHAS_TX_CRC_OFFLOADING
    #undef ipconfigHAS_TX_CRC_OFFLOADING
#endif
/* Override this define because the KSZ8851 is programmed to set all outgoing CRC's */
#define    ipconfigHAS_TX_CRC_OFFLOADING    1

#ifndef    EMAC_MAX_BLOCK_TIME_MS
    #define    EMAC_MAX_BLOCK_TIME_MS    100ul
#endif

/* Default the size of the stack used by the EMAC deferred handler task to 4x
the size of the stack used by the idle task - but allow this to be overridden in
FreeRTOSConfig.h as configMINIMAL_STACK_SIZE is a user definable constant. */
#ifndef configEMAC_TASK_STACK_SIZE
    #define configEMAC_TASK_STACK_SIZE ( 6 * configMINIMAL_STACK_SIZE )
#endif

#define SPI_PDC_IDLE            0
#define SPI_PDC_RX_START        1
#define SPI_PDC_TX_ERROR        2
#define SPI_PDC_RX_COMPLETE     3
#define SPI_PDC_TX_START        4
#define SPI_PDC_RX_ERROR        5
#define SPI_PDC_TX_PREPARE      6
#define SPI_PDC_TX_COMPLETE     7

#define MICREL_RX_BUFFERS       3
#define MICREL_TX_BUFFERS       1

/* The size of the stack and the priority used by the two echo client tasks. */
#define mainECHO_CLIENT_TASK_STACK_SIZE 	( configMINIMAL_STACK_SIZE * 2 )
#define mainECHO_CLIENT_TASK_PRIORITY		( tskIDLE_PRIORITY + 1 )
/**
 * ksz8851snl driver structure.
 */
typedef struct {
    /** Set to 1 when owner is software (ready to read), 0 for Micrel. */
    uint32_t rx_ready[MICREL_RX_BUFFERS];
    /** Set to 1 when owner is Micrel, 0 for software. */
    uint32_t tx_busy[MICREL_TX_BUFFERS];
    /** RX xNetworkBufferDescriptor_t pointer list */
    xNetworkBufferDescriptor_t *rx_buffers[MICREL_RX_BUFFERS];
    /** TX xNetworkBufferDescriptor_t pointer list */
    xNetworkBufferDescriptor_t *tx_buffers[MICREL_TX_BUFFERS];
    /* xNetworkBufferDescriptor_t *tx_cur_buffer; */

    /** Circular buffer head pointer for packet received. */
    uint32_t us_rx_head;
    /** Circular buffer tail pointer for packet to be read. */
    uint32_t us_rx_tail;
    /** Circular buffer head pointer by upper layer (buffer to be sent). */
    uint32_t us_tx_head;
    /** Circular buffer tail pointer incremented by handlers (buffer sent). */
    uint32_t us_tx_tail;

    uint32_t ul_total_tx;
    uint32_t ul_total_rx;
    /* uint32_t tx_space; */

    /* ul_spi_pdc_status has "SPI_PDC_xxx" values. */
    volatile uint32_t ul_spi_pdc_status;

    /* ul_had_intn_interrupt becomes true within the INTN interrupt. */
    volatile uint32_t ul_had_intn_interrupt;

    uint16_t us_pending_frame;
} xENC28J60_Device_t;

/* Temporary buffer for PDC reception.
declared in ASF\sam\components\ksz8851snl\ksz8851snl.c */
/* extern uint8_t tmpbuf[1536]; */

/* COMPILER_ALIGNED(8) */
static xENC28J60_Device_t xMicrelDevice;

static TaskHandle_t xTransmitHandle;

extern uint8_t ucMACAddress[ 6 ];
ENC_HandleTypeDef encHandle;

/*-----------------------------------------------------------*/

/*
 * Wait a fixed time for the link status to indicate the network is up.
 */
static BaseType_t xGMACWaitLS( TickType_t xMaxTime );

/*
 * A deferred interrupt handler task that processes GMAC interrupts.
 */
static void prvEMACHandlerTask( void *pvParameters );

/*
 * Try to obtain an Rx packet from the hardware.
 */
static uint32_t prvEMACRxPoll( void );

static void enc28j60_low_level_init( void );

static xNetworkBufferDescriptor_t *enc_low_level_input( void );

/*-----------------------------------------------------------*/

/* Bit map of outstanding ETH interrupt events for processing.  Currently only
the Rx interrupt is handled, although code is included for other events to
enable future expansion. */
static volatile uint16_t ulISREvents;

/* A copy of PHY register 1: 'PHY_REG_01_BMSR' */
static uint16_t ulPHYLinkStatus = 0;
static volatile BaseType_t xGMACSwitchRequired;

static void Enc_Update( void );

static void enc_rx_init( void );

static void enc28j60_tx_init( void );

/* Holds the handle of the task used as a deferred interrupt processor.  The
handle is used so direct notifications can be sent to the task for all EMAC/DMA
related interrupts. */
TaskHandle_t xEMACTaskHandle = NULL;

/* The queue used to communicate Ethernet events to the IP task. */
extern xQueueHandle xNetworkEventQueue;

/*-----------------------------------------------------------*/

BaseType_t xNetworkInterfaceInitialise( void )
{
    const TickType_t x5_Seconds = 5000UL;

    if( xEMACTaskHandle == NULL )
    {
        enc28j60_low_level_init();

        /* Wait at most 5 seconds for a Link Status in the PHY. */
        xGMACWaitLS( pdMS_TO_TICKS( x5_Seconds ) );

        /* The handler task is created at the highest possible priority to
        ensure the interrupt handler can return directly to it. */
        xTaskCreate( prvEMACHandlerTask, "ENC28J60", configEMAC_TASK_STACK_SIZE, NULL, configMAX_PRIORITIES - 1, &xEMACTaskHandle );
        configASSERT( xEMACTaskHandle );
    }

    /* When returning non-zero, the stack will become active and
    start DHCP (in configured) */
    ulPHYLinkStatus = Enc_LinkStatus(&encHandle);

    return ( ulPHYLinkStatus & PHSTAT2_LSTAT ) != 0;
}
/*-----------------------------------------------------------*/

BaseType_t xGetPhyLinkStatus( void )
{
    BaseType_t xResult;

    /* This function returns true if the Link Status in the PHY is high. */
    if( ( ulPHYLinkStatus & PHSTAT2_LSTAT ) != 0 )
    {
        xResult = pdTRUE;
    }
    else
    {
        xResult = pdFALSE;
    }

    return xResult;
}
/*-----------------------------------------------------------*/
BaseType_t xNetworkInterfaceOutput( xNetworkBufferDescriptor_t * const pxNetworkBuffer )
{
    BaseType_t xResult = pdFALSE;
    int txHead = xMicrelDevice.us_tx_head;

    /* Make sure the next descriptor is free. */
    if( xMicrelDevice.tx_busy[ txHead ] != pdFALSE )
    {
        /* All TX buffers busy. */
    }
    else if( ( ulPHYLinkStatus & PHSTAT2_LSTAT ) == 0 )
    {
        /* Output: LS low. */
    }
    else
    {
        /* Pass the packet. */
        xMicrelDevice.tx_buffers[ txHead ] = pxNetworkBuffer;
        /* The descriptor is now owned by Micrel. */
        xMicrelDevice.tx_busy[ txHead ] = pdTRUE;

        /* Move the head pointer. */
        if( ++txHead == MICREL_TX_BUFFERS )
        {
            txHead = 0;
        }
        xMicrelDevice.us_tx_head = txHead;
        if( xEMACTaskHandle != NULL )
        {
            xTaskNotifyGive( xEMACTaskHandle );
        }

        xResult = pdTRUE;
    }
    if( xResult == pdFALSE )
    {
        vNetworkBufferRelease( pxNetworkBuffer );
    }
    
    return xResult;
}
/*-----------------------------------------------------------*/

static BaseType_t xGMACWaitLS( TickType_t xMaxTime )
{
    TickType_t xStartTime = xTaskGetTickCount();
    TickType_t xEndTime;
    BaseType_t xReturn;
    const TickType_t xShortTime = pdMS_TO_TICKS( 100UL );

    for( ;; )
    {
        xEndTime = xTaskGetTickCount();

        if( ( xEndTime - xStartTime ) > xMaxTime )
        {
            /* Wated more than xMaxTime, return. */
            xReturn = pdFALSE;
            break;
        }
        
        /* Check the link status again. */
        ulPHYLinkStatus = Enc_LinkStatus(&encHandle);
        if(((ulPHYLinkStatus) & PHSTAT2_LSTAT) != 0 )
        {
            /* Link is up - return. */
            xReturn = pdTRUE;
            break;
        }
        
        /* Link is down - wait in the Blocked state for a short while (to allow
        other tasks to execute) before checking again. */
        vTaskDelay( xShortTime );
    }

    return xReturn;
}
/*-----------------------------------------------------------*/

/**
 * \brief Handler for SPI interrupt.
 */
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
    BaseType_t xDoWakeup = pdFALSE;
    BaseType_t xKSZTaskWoken = pdFALSE;
    
    switch( xMicrelDevice.ul_spi_pdc_status )
    {
        case SPI_PDC_RX_START:
        {
            xMicrelDevice.ul_spi_pdc_status = SPI_PDC_RX_COMPLETE;
            Enc_CsPinDeActive();
            xDoWakeup = pdTRUE;
            break;
        }
        default:
            break;
    }
    
    if( xDoWakeup != pdFALSE )
    {
        if( xEMACTaskHandle != NULL )
        {
            vTaskNotifyGiveFromISR( xEMACTaskHandle, ( BaseType_t * ) &xKSZTaskWoken );
        }
    }
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    BaseType_t xDoWakeup = pdFALSE;
    BaseType_t xKSZTaskWoken = pdFALSE;
    
    switch( xMicrelDevice.ul_spi_pdc_status )
    {
        case SPI_PDC_TX_START:
        {
            xMicrelDevice.ul_spi_pdc_status = SPI_PDC_TX_COMPLETE;
            Enc_CsPinDeActive();
            xDoWakeup = pdTRUE;
            break;
        }
        default:
            break;
    }
    
    if( xDoWakeup != pdFALSE )
    {
        if( xEMACTaskHandle != NULL )
        {
            vTaskNotifyGiveFromISR( xEMACTaskHandle, ( BaseType_t * ) &xKSZTaskWoken );
        }
    }
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    BaseType_t xDoWakeup = pdFALSE;
    BaseType_t xKSZTaskWoken = pdFALSE;
    
    switch( xMicrelDevice.ul_spi_pdc_status )
    {
        case SPI_PDC_RX_START:
        {
            xMicrelDevice.ul_spi_pdc_status = SPI_PDC_RX_ERROR;
            Enc_CsPinDeActive();
            xDoWakeup = pdTRUE;
            break;
        }
        case SPI_PDC_TX_START:
        {
            xMicrelDevice.ul_spi_pdc_status = SPI_PDC_TX_ERROR;
            Enc_CsPinDeActive();
            xDoWakeup = pdTRUE;
            break;
        }
        default:
            break;
    }    /* switch( xMicrelDevice.ul_spi_pdc_status ) */

    if( xDoWakeup != pdFALSE )
    {
        if( xEMACTaskHandle != NULL )
        {
            vTaskNotifyGiveFromISR( xEMACTaskHandle, ( BaseType_t * ) &xKSZTaskWoken );
        }
    }
}
    
/*-----------------------------------------------------------*/

static void INTN_Handler(uint32_t id, uint32_t mask)
{
    BaseType_t xKSZTaskWoken = pdFALSE;

    /* if( ( id == INTN_ID ) &&
        ( mask == INTN_PIN_MSK ) )
    { */
        /* Clear the PIO interrupt flags. */
        /* pio_get_interrupt_status( INTN_PIO ); */

        /* Set the INTN flag. */
        xMicrelDevice.ul_had_intn_interrupt++;
        if( xEMACTaskHandle != NULL )
        {
            vTaskNotifyGiveFromISR( xEMACTaskHandle, &( xKSZTaskWoken ) );
        }
    /* } */
    portEND_SWITCHING_ISR( xKSZTaskWoken );
}
/*-----------------------------------------------------------*/

/**
 * \brief Populate the RX descriptor ring buffers with pbufs.
 *
 * \param Pointer to driver data structure.
 */
static void enc_rx_populate_queue( void )
{
    uint32_t ul_index = 0;
    xNetworkBufferDescriptor_t *pxNetworkBuffer;

    /* Set up the RX descriptors */
    for (ul_index = 0; ul_index < MICREL_RX_BUFFERS; ul_index++) {
        if( xMicrelDevice.rx_buffers[ ul_index ] == NULL )
        {
            /* Allocate a new xNetworkBufferDescriptor_t with the maximum size. */
            pxNetworkBuffer = pxNetworkBufferGet( ipconfigNETWORK_MTU + 36, 100 );
            if( pxNetworkBuffer == NULL )
            {
                configASSERT( 1 == 2 );
            }

            /* Make sure lwIP is well configured so one xNetworkBufferDescriptor_t can contain the maximum packet size. */
            //LWIP_ASSERT("enc_rx_populate_queue: xNetworkBufferDescriptor_t size too small!", pbuf_clen(pxNetworkBuffer) <= 1);

            /* Save xNetworkBufferDescriptor_t pointer to be sent to lwIP upper layer. */
            xMicrelDevice.rx_buffers[ ul_index ] = pxNetworkBuffer;
            /* Pass it to Micrel for reception. */
            xMicrelDevice.rx_ready[ ul_index ] = pdFALSE;
        }
    }
}

/* unsigned tx_space, wait_tx_space, tx_status, fhr_status; */
/* unsigned rx_debug = 0; */
/**
 * \brief Update Micrel state machine and perform required actions.
 *
 * \param netif the lwIP network interface structure for this ethernetif.
 */
static void Enc_Update(void)
{
    /* Check for free PDC. */
    switch( xMicrelDevice.ul_spi_pdc_status )
    {
        case SPI_PDC_TX_ERROR:
        {
            /* Disable asynchronous transfer mode. */
            xMicrelDevice.ul_spi_pdc_status = SPI_PDC_IDLE;

            /* TX step12: stop transmission. */
            Enc_StopTransmission();
        }
        break;

        case SPI_PDC_RX_ERROR:
        {
            /* Disable asynchronous transfer mode. */
            xMicrelDevice.ul_spi_pdc_status = SPI_PDC_IDLE;
            
            Enc_FreeRxBufferSpace(&encHandle);
        }
        break;
    }
    
    switch( xMicrelDevice.ul_spi_pdc_status )
    {
        case SPI_PDC_IDLE:
        {
            int txTail = xMicrelDevice.us_tx_tail;

            /*
             * Handle RX
             */
            if( ( xMicrelDevice.ul_had_intn_interrupt != 0 ) || ( xMicrelDevice.us_pending_frame > 0 ) )
            {
                int      rxHead = xMicrelDevice.us_rx_head;
                /* uint8_t  rsv[6]; */
                uint16_t pktlen;
                /* uint16_t rxstat; */
                bool     retval = false;
                
                xNetworkBufferDescriptor_t *pxNetworkBuffer;
#warning try
                xMicrelDevice.ul_had_intn_interrupt = 0;

                if( xMicrelDevice.us_pending_frame == 0 )
                {
                    /* uint8_t int_status; */
                    /* read interrupt status for INT_RX flag. */
                    /* int_status = enc_rdgreg(ENC_EIR) & EIR_ALLINTS; */

                    /* disable global interrupts. */
                    enc_bfcgreg(ENC_EIE, EIE_INTIE);

                    /* clear RX interrupt flag. */
                    enc_bfcgreg(ENC_EIR, EIR_PKTIF);

                    /* check for received frames. */
                    xMicrelDevice.us_pending_frame = Enc_GetPkCnt(&encHandle);
                    if( xMicrelDevice.us_pending_frame == 0 )
                    {
                        /* enable global interrupt. */
                        enc_bfsgreg(ENC_EIE, EIE_INTIE);
                        return;
                    }
                }
#warning try
                xMicrelDevice.ul_had_intn_interrupt = 0;

                /* Now xMicrelDevice.us_pending_frame != 0 */

                /* Don't break Micrel state machine, wait for a free descriptor first! */
                if( xMicrelDevice.rx_ready[ rxHead ] != pdFALSE )
                {
                    return;
                }
                pxNetworkBuffer = xMicrelDevice.rx_buffers[ rxHead ];

                if( pxNetworkBuffer == NULL )
                {
                    enc_rx_populate_queue();
                    return;
                }

                /* Get RX packet status. */
                retval = Enc_UpdateRxBufferStatus(&encHandle, &pktlen);
                if(retval == false)
                {
                    Enc_FreeRxBufferSpace(&encHandle);
                    
                    /* check for received frames. */
                    xMicrelDevice.us_pending_frame = Enc_GetPkCnt(&encHandle);
                    if( xMicrelDevice.us_pending_frame == 0 )
                    {
                        /* enable INT_RX flag. */
                        enc_bfsgreg(ENC_EIE, EIE_INTIE | EIE_RXERIE);
                    }
                    ulISREvents |= EMAC_IF_ERR_EVENT;
                }
                else
                {
                    size_t xReadLength = pktlen;
                    
                    xMicrelDevice.ul_total_rx++;
                    
                    xMicrelDevice.ul_spi_pdc_status = SPI_PDC_RX_START;
                    
                    /* Copy the data data from the receive buffer to priv->dev.d_buf.
                    * end_rdbuffer (above).
                    */
                    Enc_ReadBufferIT(pxNetworkBuffer->pucEthernetBuffer, xReadLength);
                    
                    /* Remove CRC and update buffer length. */
                    pktlen -= 4;
                    pxNetworkBuffer->xDataLength = pktlen;
                    /* Wait for SPI interrupt to set status 'SPI_PDC_RX_COMPLETE'. */
                }
                break;
            } /* ul_had_intn_interrupt || us_pending_frame */
            
            /*
             * Handle TX
             */
            /* Fetch next packet to be sent. */
            if( ( xMicrelDevice.tx_busy[ txTail ] != pdFALSE ) &&
                ( xMicrelDevice.us_pending_frame == 0 ) &&
                ( xMicrelDevice.ul_had_intn_interrupt == 0 ) )
            {
                xNetworkBufferDescriptor_t *pxNetworkBuffer = xMicrelDevice.tx_buffers[ txTail ];
                size_t xLength = pxNetworkBuffer->xDataLength;
                int iIndex = xLength;

                xLength = 4 * ( ( xLength + 3 ) / 4 );
                while( iIndex < ( int ) xLength )
                {
                    pxNetworkBuffer->pucEthernetBuffer[ iIndex ] = '\0';
                    iIndex++;
                }
                pxNetworkBuffer->xDataLength = xLength;

                /* Check if TX is busy or valid for transmit */
                if((Enc_GetTxStatus(&encHandle) == ENC_TX_IDLE) && \
                   ((PKTMEM_TX_START + xLength + 8) <= PKTMEM_TX_ENDP1))
                {
                    /* disable global interrupts. */
                    enc_bfcgreg(ENC_EIE, EIE_INTIE);
                    
                    if(Enc_PrepareTxBuffer(&encHandle, xLength) != ERR_OK)
                    {
                        /* perform FIFO write operation. */
                        xMicrelDevice.ul_spi_pdc_status = SPI_PDC_TX_PREPARE;
                        /* xMicrelDevice.tx_cur_buffer = pxNetworkBuffer; */
                    
                        xMicrelDevice.ul_total_tx++;
                        Enc_WriteBuffer(pxNetworkBuffer->pucEthernetBuffer, xLength);
                    }
                }
            }
        }
        break;    /* SPI_PDC_IDLE */

        case SPI_PDC_RX_COMPLETE:
        {
            int rxHead = xMicrelDevice.us_rx_head;

            /* Disable asynchronous transfer mode. */
            xMicrelDevice.ul_spi_pdc_status = SPI_PDC_IDLE;

            Enc_FreeRxBufferSpace(&encHandle);
            
            /* update frame count to be read. */
            xMicrelDevice.us_pending_frame -= 1;

            /* enable INT_RX flag if transfer complete. */
            if( xMicrelDevice.us_pending_frame == 0 )
            {
                enc_bfsgreg(ENC_EIE, EIE_INTIE | EIE_RXERIE);
            }

            /* Mark descriptor ready to be read. */
            xMicrelDevice.rx_ready[ rxHead ] = pdTRUE;
            if( ++rxHead == MICREL_RX_BUFFERS )
            {
                rxHead = 0;
            }
            xMicrelDevice.us_rx_head = rxHead;

            /* Tell prvEMACHandlerTask that RX packets are available. */
            ulISREvents |= EMAC_IF_RX_EVENT;
        }    /* case SPI_PDC_RX_COMPLETE */
        break;

        case SPI_PDC_TX_COMPLETE:
        {
            int txTail = xMicrelDevice.us_tx_tail;
            xNetworkBufferDescriptor_t *pxNetworkBuffer = xMicrelDevice.tx_buffers[ txTail ];

            size_t xLength;
            /* TX step9-10: pad with dummy data to keep dword alignment. */
            /* Not necessary: length is already a multiple of 4. */
            xLength = pxNetworkBuffer->xDataLength & 3;
            if( xLength != 0 )
            {
//                ksz8851_fifo_dummy( 4 - xLength );
            }

//            /* TX step11: end TX transfer. */
            /* gpio_set_pin_high( KSZ8851SNL_CSN_GPIO ); */

            /* Disable asynchronous transfer mode. */
            xMicrelDevice.ul_spi_pdc_status = SPI_PDC_IDLE;

            /* TX step12: disable TXQ write access. */
            /* ksz8851_reg_clrbits( REG_RXQ_CMD, RXQ_START ); */

            /* xMicrelDevice.tx_space = ksz8851_reg_read( REG_TX_MEM_INFO ) & TX_MEM_AVAILABLE_MASK; */

            /* TX step12.1: enqueue frame in TXQ. */
            /* ksz8851_reg_setbits( REG_TXQ_CMD, TXQ_ENQUEUE ); */

            /* RX step13: enable INT_RX flag. */
//            ksz8851_reg_write( REG_INT_MASK, INT_RX );
            /* Buffer sent, free the corresponding buffer and mark descriptor as owned by software. */
            vNetworkBufferRelease( pxNetworkBuffer );

            xMicrelDevice.tx_buffers[ txTail ] = NULL;
            xMicrelDevice.tx_busy[ txTail ] = pdFALSE;
            if( ++txTail == MICREL_TX_BUFFERS )
            {
                txTail = 0;
            }

            xMicrelDevice.us_tx_tail = txTail;
            /* Experiment. */
            //xMicrelDevice.ul_had_intn_interrupt = 1;
            if( xTransmitHandle != NULL )
            {
                xTaskNotifyGive( xTransmitHandle );
            }
#warning moved downward
            /* RX step13: enable INT_RX flag. */
            /* ksz8851_reg_write( REG_INT_MASK, INT_RX ); */
            /* Prevent the EMAC task from sleeping a single time. */
            ulISREvents |= EMAC_IF_TX_EVENT;
        }    /* case SPI_PDC_TX_COMPLETE */
        break;
    }    /* switch( xMicrelDevice.ul_spi_pdc_status ) */
}

/**
 * \brief Set up the RX descriptor ring buffers.
 *
 * This function sets up the descriptor list used for RX packets.
 *
 */
static void enc_rx_init(void)
{
    uint32_t ul_index = 0;

    /* Init pointer index. */
    xMicrelDevice.us_rx_head = 0;
    xMicrelDevice.us_rx_tail = 0;

    /* Set up the RX descriptors. */
    for (ul_index = 0; ul_index < MICREL_RX_BUFFERS; ul_index++) {
        xMicrelDevice.rx_buffers[ul_index] = NULL;
        xMicrelDevice.rx_ready[ul_index] = pdFALSE;
    }

    /* Build RX buffer and descriptors. */
    enc_rx_populate_queue();
}

/**
 * \brief Set up the TX descriptor ring buffers.
 *
 * This function sets up the descriptor list used for TX packets.
 *
 */
static void enc28j60_tx_init(void)
{
    uint32_t ul_index = 0;

    /* Init TX index pointer. */
    xMicrelDevice.us_tx_head = 0;
    xMicrelDevice.us_tx_tail = 0;

    /* Set up the TX descriptors */
    for( ul_index = 0; ul_index < MICREL_TX_BUFFERS; ul_index++ )
    {
        xMicrelDevice.tx_busy[ul_index] = pdFALSE;
    }
    /* xMicrelDevice.tx_space = 6144; */
}

/**
 * \brief Initialize ksz8851snl ethernet controller.
 *
 * \note Called from ethernetif_init().
 *
 * \param netif the lwIP network interface structure for this ethernetif.
 */
static void enc28j60_low_level_init( void )
{
    
    enc_rx_init();
    enc28j60_tx_init();

    encHandle.Init.MACAddr = ucMACAddress;
    encHandle.Init.DuplexMode = ETH_MODE_HALFDUPLEX;
    encHandle.Init.ChecksumMode = ETH_CHECKSUM_BY_HARDWARE;
    encHandle.Init.InterruptEnableBits =  EIE_LINKIE | EIE_PKTIE;
  
    /* Initialize SPI link. */
    if( Enc_Init(&encHandle) == false )
    {
        configASSERT(0 == 1);
    }
    
    ENC_SetMacAddr(&encHandle);
}

/**
 * \brief Use pre-allocated pbuf as DMA source and return the incoming packet.
 *
 * \param netif the lwIP network interface structure for this ethernetif.
 *
 * \return a pbuf filled with the received packet (including MAC header).
 * 0 on memory error.
 */
static xNetworkBufferDescriptor_t *enc_low_level_input( void )
{
    xNetworkBufferDescriptor_t *pxNetworkBuffer = NULL;
    int rxTail = xMicrelDevice.us_rx_tail;

    /* Check that descriptor is owned by software (ie packet received). */
    if( xMicrelDevice.rx_ready[ rxTail ] != pdFALSE )
    {

        /* Fetch pre-allocated buffer */
        pxNetworkBuffer = xMicrelDevice.rx_buffers[ rxTail ];

        /* Remove this pbuf from its descriptor. */
        xMicrelDevice.rx_buffers[ rxTail ] = NULL;

        /* Clears rx_ready and sets rx_buffers. */
        enc_rx_populate_queue();

        if( ++rxTail == MICREL_RX_BUFFERS )
        {
            rxTail = 0;
        }
        xMicrelDevice.us_rx_tail = rxTail;
    }

    return pxNetworkBuffer;
}
/*-----------------------------------------------------------*/

static uint32_t prvEMACRxPoll( void )
{
    xNetworkBufferDescriptor_t *pxNetworkBuffer;
    xIPStackEvent_t xRxEvent = { eEthernetRxEvent, NULL };
    uint32_t ulReturnValue = 0;

    for( ;; )
    {
        /* Only for logging. */
        /* int rxTail = xMicrelDevice.us_rx_tail; */

        pxNetworkBuffer = enc_low_level_input();
        if( pxNetworkBuffer == NULL )
        {
            break;
        }

        ulReturnValue++;
        xRxEvent.pvData = ( void * )pxNetworkBuffer;
        /* Send the descriptor to the IP task for processing. */
        if( xQueueSendToBack( xNetworkEventQueue, &xRxEvent, ( TickType_t ) 0 ) == pdFALSE )
        {
            vNetworkBufferRelease( pxNetworkBuffer );
            iptraceETHERNET_RX_EVENT_LOST();
        }
        else
        {
            iptraceNETWORK_INTERFACE_RECEIVE();
        }
    }

    return ulReturnValue;
}
/*-----------------------------------------------------------*/

static void prvEMACHandlerTask( void *pvParameters )
{
    TimeOut_t xPhyTime;
    TickType_t xPhyRemTime;
    TickType_t xLoggingTime;
    BaseType_t xResult = 0;
    uint16_t xStatus;
    const TickType_t ulMaxBlockTime = pdMS_TO_TICKS( EMAC_MAX_BLOCK_TIME_MS );
#if( ipconfigCHECK_IP_QUEUE_SPACE != 0 )
    UBaseType_t uxLastMinQueueSpace = 0;
#endif

    /* Remove compiler warnings about unused parameters. */
    ( void ) pvParameters;

    configASSERT( xEMACTaskHandle );

    vTaskSetTimeOutState( &xPhyTime );
    xPhyRemTime = pdMS_TO_TICKS( PHY_LS_LOW_CHECK_TIME_MS );
    xLoggingTime = xTaskGetTickCount();

    for( ;; )
    {
        /* Run the state-machine of the ksz8851 driver. */
        Enc_Update();

        if( ( ulISREvents & EMAC_IF_ALL_EVENT ) == 0 )
        {
            /* No events to process now, wait for the next. */
            ulTaskNotifyTake( pdTRUE, ulMaxBlockTime );
        }

        if( ( xTaskGetTickCount() - xLoggingTime ) > 10000 )
        {
            xLoggingTime += 10000;
        }

        if( ( ulISREvents & EMAC_IF_RX_EVENT ) != 0 )
        {
            ulISREvents &= ~EMAC_IF_RX_EVENT;

            /* Wait for the EMAC interrupt to indicate that another packet has been
            received. */
            xResult = prvEMACRxPoll();
        }

        if( ( ulISREvents & EMAC_IF_TX_EVENT ) != 0 )
        {
            /* Future extension: code to release TX buffers if zero-copy is used. */
            ulISREvents &= ~EMAC_IF_TX_EVENT;
        }

        if( ( ulISREvents & EMAC_IF_ERR_EVENT ) != 0 )
        {
            /* Future extension: logging about errors that occurred. */
            ulISREvents &= ~EMAC_IF_ERR_EVENT;
        }

        if( xResult > 0 )
        {
            /* As long as packets are being received, assume that
            the Link Status is high. */
            ulPHYLinkStatus |= PHSTAT2_LSTAT;
            /* A packet was received. No need to check for the PHY status now,
            but set a timer to check it later on. */
            vTaskSetTimeOutState( &xPhyTime );
            xPhyRemTime = pdMS_TO_TICKS( PHY_LS_HIGH_CHECK_TIME_MS );
            xResult = 0;
        }
        else if( ( xTaskCheckForTimeOut( &xPhyTime, &xPhyRemTime ) != pdFALSE ) &&
            ( xMicrelDevice.ul_spi_pdc_status == SPI_PDC_IDLE ) )
        {
            /* Check the link status again. */
            xStatus = Enc_LinkStatus(&encHandle);

            if( ( ulPHYLinkStatus & PHSTAT2_LSTAT ) != ( xStatus & PHSTAT2_LSTAT ) )
            {
                ulPHYLinkStatus = xStatus;
            }

            vTaskSetTimeOutState( &xPhyTime );
            if( ( ulPHYLinkStatus & PHSTAT2_LSTAT ) != 0 )
            {
                xPhyRemTime = pdMS_TO_TICKS( PHY_LS_HIGH_CHECK_TIME_MS );
            }
            else
            {
                xPhyRemTime = pdMS_TO_TICKS( PHY_LS_LOW_CHECK_TIME_MS );
            }
        }
    }
}
/*-----------------------------------------------------------*/
/* Called by FreeRTOS+UDP when the network connects. */
void vApplicationIPNetworkEventHook( eIPCallbackEvent_t eNetworkEvent )
{
static BaseType_t xTaskAlreadyCreated = pdFALSE;
#if 0
	if( eNetworkEvent == eNetworkUp )
	{
		/* Create the tasks that transmit to and receive from a standard
		echo server (see the web documentation for this port) in both
		standard and zero copy mode. */
		if( xTaskAlreadyCreated == pdFALSE )
		{
			vStartEchoClientTasks( mainECHO_CLIENT_TASK_STACK_SIZE, mainECHO_CLIENT_TASK_PRIORITY );
			xTaskAlreadyCreated = pdTRUE;
		}
	}
#endif
}

/*-----------------------------------------------------------*/
/* Called by FreeRTOS+UDP when a reply is received to an outgoing ping request. */
void vApplicationPingReplyHook( ePingReplyStatus_t eStatus, uint16_t usIdentifier )
{
    static const char *pcSuccess = "\r\n\r\nPing reply received - ";
    static const char *pcInvalidChecksum = "\r\n\r\nPing reply received with invalid checksum - ";
    static const char *pcInvalidData = "\r\n\r\nPing reply received with invalid data - ";
    static char cMessage[ 50 ];
    /* void vOutputString( const char * const pcMessage ); */

	switch( eStatus )
	{
		case eSuccess	:
			/* vOutputString( pcSuccess ); */
			break;

		case eInvalidChecksum :
			/* vOutputString( pcInvalidChecksum ); */
			break;

		case eInvalidData :
			/* vOutputString( pcInvalidData ); */
			break;

		default :
			/* It is not possible to get here as all enums have their own
			case. */
			break;
	}

	sprintf( cMessage, "identifier %d\r\n\r\n", ( int ) usIdentifier );
	/* vOutputString( cMessage ); */
}

/*-----------------------------------------------------------*/

