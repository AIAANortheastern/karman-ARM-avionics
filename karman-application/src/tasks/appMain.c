

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
extern imu_sensor_data_t gSensorData;
extern ms5607_02ba03_data_t altimeterData;

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
            //pthread_mutex_lock(&gSensorDataMutex);
            //Display_printf(gTheDisplay, 0, 0, "Euler data: x=%f, y=%f, z=%f", gSensorData.imuData.euler.x, gSensorData.imuData.euler.y, gSensorData.imuData.euler.z);
            Display_printf(gTheDisplay, 0, 0, "Accelerometer data: x=%f, y=%f, z=%f", gSensorData.accele.x, gSensorData.accele.y, gSensorData.accele.z);
            Display_printf(gTheDisplay, 0, 0, "Magnetometer data: x=%f, y=%f, z=%f", gSensorData.magnet.x, gSensorData.magnet.y, gSensorData.magnet.z);
            Display_printf(gTheDisplay, 0, 0, "Gyroscope data: x=%f, y=%f, z=%f", gSensorData.gyros.x, gSensorData.gyros.y, gSensorData.gyros.z);
            Display_printf(gTheDisplay, 0, 0, "Temperature data: %f", altimeterData.temp);
            Display_printf(gTheDisplay, 0, 0, "Pressure data: %f", altimeterData.pressure);
            pthread_mutex_unlock(&gDisplayMuxtex);
            //pthread_mutex_unlock(&gSensorDataMutex);
        }
        vTaskDelayUntil( &xLastWaketime, xFrequency );
    }



    return (NULL);
}
