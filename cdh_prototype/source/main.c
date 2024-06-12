/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2019 NXP
 * All rights reserved.
 *
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/* FreeRTOS kernel includes. */
#include <rtwdog_prototype.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"

/* Freescale includes. */
#include "fsl_debug_console.h"
#include "fsl_common.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "board.h"

/* Peripherals includes. */
#include "peripherals.h"

/* Tasks includes.*/
#include "obc_task.h"

#include <stdbool.h>
#include "com_protocol_helper.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define CPU_NAME "iMXRT1021"

/*******************************************************************************
 * Code
 ******************************************************************************/
int main( void ) {
	/* Init board hardware. */
    BOARD_ConfigMPU();
    BOARD_InitPins();
    BOARD_InitBootClocks();
    //BOARD_InitDebugConsole();
    BOARD_InitPeripherals();
    initializeRTWDOG(); // set up the RTWDOG software system

    PRINTF( "SOC-i Flight software start====================================================================\r\n" );
    if( xTaskCreate(obc_task, "obc_task", configMINIMAL_STACK_SIZE + 100, NULL, 4, NULL ) != pdPASS ) {
        PRINTF( "Task creation failed!.\r\n" );
        while( 1 );
	}
    vTaskStartScheduler();
    for(;;);
}

//void PRINTF() { }
