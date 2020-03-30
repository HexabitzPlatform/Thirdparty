/*
 * MODBUS Library: ARM STM32 Port (FWLIB 2.0x)
 * Copyright (c) Christian Walter <cwalter@embedded-solutions.at>
 * All rights reserved.
 *
 * ARM STM32 Port by Niels Andersen, Elcanic A/S <niels.andersen.elcanic@gmail.com>
 *
 * $Id: mbportother.c,v 1.1 2008-12-14 19:33:32 cwalter Exp $
 */

/* ----------------------- System includes ----------------------------------*/
#include <stdlib.h>

/* ----------------------- Platform includes --------------------------------*/
#include "mbport.h"
#include "common/mbtypes.h"
#include "common/mbframe.h"
#include "common/mbutils.h"
#include "H1DR1.h"

/* ----------------------- Modbus includes ----------------------------------*/

/* ----------------------- Defines ------------------------------------------*/
#define PORT_INTERRUPT_PRIORITY_MAX     ( 1 )

/* ----------------------- Type definitions ---------------------------------*/

/* ----------------------- Static variables ---------------------------------*/

static UBYTE    ubNesting = 0;

/* ----------------------- Static functions ---------------------------------*/

/* ----------------------- Start implementation -----------------------------*/

void
assert_failed( uint8_t * file, uint32_t line )
{
    ( void )file;
    ( void )line;
    vMBPAssert(  );
}

void
vMBPAssert( void )
{
    volatile BOOL   bBreakOut = FALSE;

    vMBPEnterCritical(  );
    while( !bBreakOut );
}

void
vMBPEnterCritical( void )
{
	portENTER_CRITICAL(  );
	
}

void
vMBPExitCritical( void )
{
	portEXIT_CRITICAL(  );
 
}
