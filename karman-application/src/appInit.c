/* \file appInit.c
 *
 *  Based of TI timer GPIO demo app.
 *
 * Andrew Kaster
 * Jan 31 2018
 *  */


/*
 * Copyright (c) 2016-2017, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ======== timerled.c ========
 */

#include <stddef.h>
#include <unistd.h>
#include <time.h>

/* Driver Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/Timer.h>
#include <ti/Display/display.h>

#include <FreeRTOS.h>
#include <task.h>
#include <pthread.h>

/* Board Header file */
#include "Board.h"

#include "appDefs.h"
#include "sensorDefs.h"
#include "sensorTask.h"
#include "IMUTask.h"

void appInit(void)
{
    int ret = 0;
    /* Period and duty in microseconds */
    Timer_Handle timer0;
    Timer_Params params;

    /* Call driver init functions */
    GPIO_init();
    Timer_init();
    Display_init();
    SPI_init();
    I2C_init();

    /* Create Display and display mutex */
    gTheDisplay = Display_open(Display_Type_UART, NULL);
    if(!gTheDisplay)
    {
        while(1);
    }

    /* no pthread_mutexattr_t needed because we don't need a recursive mutex on the display */
    ret = pthread_mutex_init(&gDisplayMuxtex, NULL);
    if (ret != 0)
    {
        /* pthread_mutex_init() failed */
        while(1);
    }

    /** PIN CONFIGURATION **/

    /* Configure the LED pin */
    GPIO_setConfig(Board_GPIO_LED0, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);

    /* Configure the Altimeter CS Pin*/
    GPIO_setConfig(ALTIMETER_CS, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_STR_LOW);
    GPIO_write(ALTIMETER_CS, CHIP_SELECT_HIGH);

    /* Turn off user LED */
    GPIO_write(Board_GPIO_LED0, Board_GPIO_LED_OFF);

    /* Setting up the timer in continuous callback mode that calls the callback
     * function every 1,000,000 microseconds, or 1 second.
     */
    Timer_Params_init(&params);
    params.period = 1000000;
    params.periodUnits = Timer_PERIOD_US;
    params.timerMode = Timer_CONTINUOUS_CALLBACK;
    params.timerCallback = timerCallback;

    timer0 = Timer_open(Board_TIMER0, &params);

    if (timer0 == NULL) {
        /* Failed to initialized timer */
        while (1);
    }

    if (Timer_start(timer0) == Timer_STATUS_ERROR) {
        /* Failed to start timer */
        while (1);
    }
}

/*
 * This callback is called every 1,000,000 microseconds, or 1 second. Because
 * the LED is toggled each time this function is called, the LED will blink at
 * a rate of once every 2 seconds.
 * */
void timerCallback(Timer_Handle myHandle)
{
    GPIO_toggle(Board_GPIO_LED0);
}


