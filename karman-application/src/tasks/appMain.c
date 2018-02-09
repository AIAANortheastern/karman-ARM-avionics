

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

/*
 *  ======== mainThread ========
 */
void *mainThread(void *arg0)
{
    TickType_t xLastWaketime = xTaskGetTickCount();
    TickType_t xFrequency = portTICK_PERIOD_MS * 100;

    struct timespec         currtime;

    for(;;)
    {
        clock_gettime(CLOCK_REALTIME, &currtime);

        if(currtime.tv_nsec == 0)
        {
            pthread_mutex_lock(&gDisplayMuxtex);
            Display_printf(gTheDisplay, 0, 0, "Hello World! Time: %d.%d seconds\n", currtime.tv_sec, currtime.tv_nsec);
            pthread_mutex_unlock(&gDisplayMuxtex);
        }
        vTaskDelayUntil( &xLastWaketime, xFrequency );
    }



    return (NULL);
}
