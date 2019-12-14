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

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"
#include "BOS.h"

/* ----------------------- Variables ----------------------------------------*/
static eMBEventType eQueuedEvent;
static BOOL     xEventInQueue;

/* ----------------------- Start implementation -----------------------------*/
BOOL
xMBPortEventInit( void )
{
    BOOL            bStatus = FALSE;
    if( 0 != ( xQueueHdl = xQueueCreate( 1, sizeof( eMBEventType ) ) ) )
    {
        bStatus = TRUE;
    }
    return bStatus;
}

void
vMBPortEventClose( void )
{
    if( 0 != xQueueHdl )
    {
        vQueueDelete( xQueueHdl );
        xQueueHdl = 0;
    }
}

BOOL
xMBPortEventPost( eMBEventType eEvent )
{
    BOOL            bStatus = TRUE;
    if( bMBPortIsWithinException(  ) )
    {
        ( void )xQueueSendFromISR( xQueueHdl, ( const void * )&eEvent, pdFALSE );
    }
    else
    {
        ( void )xQueueSend( xQueueHdl, ( const void * )&eEvent, pdFALSE );
    }

    return bStatus;
}

BOOL
xMBPortEventGet( eMBEventType * peEvent )
{
    BOOL            xEventHappened = FALSE;

    if( pdTRUE == xQueueReceive( xQueueHdl, peEvent, portTICK_RATE_MS * 50 ) )
    {
        xEventHappened = TRUE;
    }
    return xEventHappened;
}