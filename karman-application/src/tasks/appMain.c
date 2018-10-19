

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
#include <sensorDefs.h>

/* Board Header file */
#include "Board.h"
#include "appDefs.h"

//global data struct
sensor_data_t gSensorData;

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
            Display_printf(gTheDisplay, 0, 0, "Euler data: x=%d, y=%d, z=%d", gSensorData.imuData.euler.x, gSensorData.imuData.euler.y, gSensorData.imuData.euler.z);
            Display_printf(gTheDisplay, 0, 0, "Accelerometer data: x=%d, y=%d, z=%d", gSensorData.imuData.accele.x, gSensorData.imuData.accele.y, gSensorData.imuData.accele.z);
            Display_printf(gTheDisplay, 0, 0, "Magnetometer data: x=%d, y=%d, z=%d", gSensorData.imuData.magnet.x, gSensorData.imuData.magnet.y, gSensorData.imuData.magnet.z);
            Display_printf(gTheDisplay, 0, 0, "Gyroscope data: x=%d, y=%d, z=%d", gSensorData.imuData.gyros.x, gSensorData.imuData.gyros.y, gSensorData.imuData.gyros.z);
            pthread_mutex_unlock(&gDisplayMuxtex);
        }
        vTaskDelayUntil( &xLastWaketime, xFrequency );
    }



    return (NULL);
}
