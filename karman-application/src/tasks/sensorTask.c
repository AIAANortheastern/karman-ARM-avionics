/**
 * @file SensorTask.c
 *
 * @brief Sensor Task Function and Intialization
 *
 * Created: 12/1/2016 7:19:19 PM
 *  Author: Andrew Kaster
 */

#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>

#include "sensorTask.h"

#include "ms5607-02ba03.h"

#include "appDefs.h"
#include "sensorDefs.h"
#include "Board.h"
#include "debug_printf.h"

#include <ti/drivers/SPI.h>
#include <pthread.h>
#include <errno.h>
#include <time.h>

#define QUEUE_DEPTH 16

/** Contains all current sensor values for use in ... TBD. Processing. */
sensor_data_t gCurrSensorValues;

SPI_Handle sensorSPIHandle;
pthread_mutex_t sensorSPIMutex;

static volatile uint32_t current_CS;

/* lets sensors know when their transaction has completed asyncronously */
static void sensorSPICallbackFunction (SPI_Handle handle,
                                       SPI_Transaction *transaction);

/**
 * @brief Initialize all things the sensor task needs

 * Intialize all sensor drivers
 */
void init_sensor_task(void)
{
    int ret = 0;

    SPI_init();
    /* Initialize SPI interface*/
    SPI_Params slaveSpiParams;
    SPI_Params_init(&slaveSpiParams);
    slaveSpiParams.frameFormat = SPI_POL0_PHA0;
    slaveSpiParams.transferMode = SPI_MODE_CALLBACK;
    slaveSpiParams.transferCallbackFxn = sensorSPICallbackFunction;

    sensorSPIHandle = SPI_open(Board_SPI0, &slaveSpiParams);

    if(sensorSPIHandle == NULL)
    {
        while(1); // Sensor SPI Handle failed
    }

    debug_printf("Sensor SPI Opened");

    /* no pthread_mutexattr_t needed because we don't need a recursive mutex on the display */
    ret = pthread_mutex_init(&sensorSPIMutex, NULL);
    if (ret != 0)
    {
        /* pthread_mutex_init() failed */
        while(1);
    }

    gQueueSensorRadio = xQueueCreate(QUEUE_DEPTH, sizeof(sensor_data_t));

    if(gQueueSensorRadio == NULL)
    {
        /* Sensor queue unable to be created! */
        while(1);
    }

    /* run initialization for all sensors */
    /* altimeter/pressure */
    ms5607_02ba03_init(&sensorSPIHandle);
#if 0
    /* magnetometer */
    bmx055_mag_init(&sensorSPIHandle);

    /* gyro */
    bmx500Gyro_init(&sensorSPIHandle);
#endif
}

/**
 * @brief High level sensor operations
 *
 * This task runs all sensor state machine functions and passes the updated data to
 * the main control loop and the radio.
 */
void *sensor_task_func(void *arg0)
{
    static sensor_status_t curr_status;
    struct timespec curr_time;
    static struct timespec time_20ms;

    /* Initialize task. Must be done here to allow for sleeps in initialization code */
    init_sensor_task();

    clock_gettime(CLOCK_REALTIME, &curr_time);
    clock_gettime(CLOCK_REALTIME, &time_20ms);
    debug_printf("Sensor task initialized at time %d s %l ns", curr_time.tv_sec, curr_time.tv_nsec);

    pthread_barrier_wait(&startThreadBarrier);

    for(;;)
    {
        TickType_t xLastWaketime = xTaskGetTickCount();
        TickType_t xFrequency = portTICK_PERIOD_MS * 1;
        clock_gettime(CLOCK_REALTIME, &curr_time);

        curr_status = ms5607_02ba03_run();

        /** Data acquision rate: 50Hz (20ms) **/
        if (curr_status == SENSOR_COMPLETE)
        {
            /* Do fancy things with current temp/pressure data */
            ms5607_02ba03_get_data(&(gCurrSensorValues.altimeter));

            debug_printf("Pressure %d", gCurrSensorValues.altimeter.pressure);
            debug_printf("Temp: %d", gCurrSensorValues.altimeter.temp);
        }

        /* ----TEMPLATE----
         * curr_status = <foo>_run();
         * if (curr_status == SENSOR_COMPLETE)
         * {
         *    Do fancy things with current sensor's data
         *    <foo>_get_data(&(gCurrSensorValues.<foo_type>))
         * }
         *
         */

        // send data to radio @ 50Hz
        // Copy data from IMU @ 50Hz
        if(timespec_compare(&time_20ms, &curr_time, 20))
        {
            // Copy IMU data into sensor values struct
            xQueueReceive(gQueueIMUSensor, (void *) &(gCurrSensorValues.imu), (TickType_t) 0);
            // Send All sensor values to radio
            xQueueSend(gQueueSensorRadio, (void *) &gCurrSensorValues, (TickType_t) 0);

            clock_gettime(CLOCK_REALTIME, &time_20ms);
        }

        // sleep until the next millisecond
        vTaskDelayUntil( &xLastWaketime, xFrequency );
    } /* infinite loop */

    return NULL;
}

/* @brief Transfer the sensor's transaction on the sensor bus
 *
 * @return True on success, false on failure
 */
bool sensor_spi_transfer(SPI_Transaction *transaction, uint32_t cs_pin)
{
    bool ret = false;
    int32_t mutexRet = EBUSY;

    mutexRet = pthread_mutex_trylock(&sensorSPIMutex);

    if(!mutexRet)
    {
        current_CS = cs_pin;

        GPIO_write(current_CS, CHIP_SELECT_LOW);

        ret = (int32_t)SPI_transfer(sensorSPIHandle, transaction);


        pthread_mutex_unlock(&sensorSPIMutex);
    }

    return ret;
}

static void sensorSPICallbackFunction (SPI_Handle handle,
                                SPI_Transaction *transaction)
{
    /* let sensor know its transaction finished/was stopped */
    if(transaction->arg)
        *(bool *)transaction->arg = true;

    GPIO_write(current_CS, CHIP_SELECT_HIGH);
}

