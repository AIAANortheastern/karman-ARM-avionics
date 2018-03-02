/*
 * radioTask.c
 *
 *  Created on: Feb 28, 2018
 *      Author: Andrew
 */

#include "radioTask.h"

#include "appDefs.h"
#include "sensorDefs.h"
#include "Board.h"
#include "debug_printf.h"

#include <ti/drivers/UART.h>
#include <pthread.h>
#include <errno.h>
#include <stdbool.h>

#include <FreeRTOS.h>
#include <task.h>

#define RADIO_BUFFSIZE 128

uint8_t buf[RADIO_BUFFSIZE];
UART_Handle RadioUart;

bool init_radio(void)
{
    bool ret = true;
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

    RadioUart = UART_open(Board_UART1, &uartParams);

    if (RadioUart == NULL) {
        /* UART_open() failed */
        ret = false;
    }

  // TODO REMOVE THIS FOR REAL HARDWARE
  //  char test[] = "Hello from radio uart\r\n";
  //  UART_write(RadioUart, test, sizeof(test));

    return ret;
}

void *radioTask(void *arg0)
{
    if(!init_radio())
        while(1);

    pthread_barrier_wait(&startThreadBarrier);

    configASSERT(RADIO_BUFFSIZE > sizeof(sensor_data_t));

    for(;;)
    {
        BaseType_t ret;
        TickType_t xLastWaketime = xTaskGetTickCount();
        TickType_t xFrequency = portTICK_PERIOD_MS * 20;

        // wait up to 20ms to get next sample
        ret = xQueueReceive(gQueueSensorRadio, (void *)buf, (TickType_t) xFrequency );
        if(ret != pdTRUE)
        {
            debug_printf("Failed to receive sensor data on time\n");
        }
        else
        {
            buf[sizeof(sensor_data_t)] = '\n';
            UART_write(RadioUart, buf, sizeof(sensor_data_t) + 1);
        }

        vTaskDelayUntil( &xLastWaketime, xFrequency );
    }


    return NULL;
}

