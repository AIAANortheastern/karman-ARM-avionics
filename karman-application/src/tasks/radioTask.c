/*
 * radioTask.c
 *
 *  Created on: Feb 28, 2018
 *      Author: Andrew
 */

#include "radioTask.h"

#include "sensorDefs.h"
#include "Board.h"
#include "debug_printf.h"

#include <ti/drivers/UART.h>
#include <pthread.h>
#include <errno.h>

#include <FreeRTOS.h>
#include <task.h>

int init_radio(void)
{
    int ret = 0;
    UART_Handle uart;
    UART_Params uartParams;

    /* Call driver init functions */
    UART_init();

    /* Create a UART with data processing off. */
    UART_Params_init(&uartParams);
    uartParams.writeDataMode = UART_DATA_TEXT;
    uartParams.readDataMode = UART_DATA_BINARY;
    uartParams.readReturnMode = UART_RETURN_FULL;
    uartParams.readEcho = UART_ECHO_OFF;
    uartParams.baudRate = 115200;

    uart = UART_open(Board_UART1, &uartParams);

    if (uart == NULL) {
        /* UART_open() failed */
        ret = -1;
    }

    return ret;
}
void *radioTask(void *arg0)
{
    if(init_radio())
        while(1);

    while(1);
}

