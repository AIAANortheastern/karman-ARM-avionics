/**
 * @file SensorTask.c
 *
 * @brief Sensor Task Function and Intialization
 *
 * Created: 12/1/2016 7:19:19 PM
 *  Author: Andrew Kaster
 */

#include "sensorTask.h"

#include "ms5607-02ba03.h"
//#include "BMX055Mag.h"
//#include "BMX005Gyro.h"

#include "sensorDefs.h"
#include "Board.h"
#include "debug_printf.h"

#include <ti/drivers/SPI.h>
#include <pthread.h>
#include <errno.h>

#include <FreeRTOS.h>
#include <task.h>

/** Contains all current sensor values for use in ... TBD. Processing. */
sensor_data_t gCurrSensorValues;

SPI_Handle sensorSPIHandle;
pthread_mutex_t sensorSPIMutex;


/* Keeps track of the current chip select pin being used by the Sensor SPI Bus
 *  value >= 0: Index of type MSP_EXP432P401R_GPIOName into gpioPinConfigs in src/board/MSP_EXP432P401R.c
 *  value < 0: Unused, valid to make new sensor bus transaction
 *  Must be initialized to -1 or we'll never be able to use the bus
 */
static volatile int32_t current_CS = -1;

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
    current_CS = -1; // init to "not busy"

    /* no pthread_mutexattr_t needed because we don't need a recursive mutex on the display */
    ret = pthread_mutex_init(&sensorSPIMutex, NULL);
    if (ret != 0)
    {
        /* pthread_mutex_init() failed */
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

    /* Initialize task. Must be done here to allow for sleeps in initialization code */
    init_sensor_task();

    debug_printf("Sensor task initalized");

    for(;;)
    {
        TickType_t xLastWaketime = xTaskGetTickCount();
        TickType_t xFrequency = portTICK_PERIOD_MS * 100;

        curr_status = ms5607_02ba03_run();

        if (curr_status == SENSOR_COMPLETE)
        {
            /* Do fancy things with current temp/pressure data */
            ms5607_02ba03_get_data(&(gCurrSensorValues.altimeter));

            debug_printf("Pressure %d", gCurrSensorValues.altimeter.pressure);
            debug_printf("Temp: %d", gCurrSensorValues.altimeter.temp);
        }
#if 0
        /* make this fit the new template scheme */
        curr_status = bmx055_mag_run();

        if(curr_status == SENSOR_COMPLETE)
        {
            /* do stuff with mag data */
            bmx055_mag_get_data(&(gCurrSensorValues.magnetometer));
        }

        curr_status = gyro_state_machine();

        if(curr_status == SENSOR_COMPLETE)
        {
            gyro_get_data(&(gCurrSensorValues.gyro));
        }
#endif
        /* ----TEMPLATE----
         * curr_status = <foo>_run();
         * if (curr_status == SENSOR_COMPLETE)
         * {
         *    Do fancy things with current sensor's data
         *    <foo>_get_data(&(gCurrSensorValues.<foo_type>))
         * }
         *
         */

        vTaskDelayUntil( &xLastWaketime, xFrequency );
    } /* infinite loop */

    return NULL;
}

/* @brief Transfer the sensor's transaction on the sensor bus
 *
 *
 * @return True on success, false on failure
 */
bool sensor_spi_transfer(SPI_Transaction *transaction, int32_t cs_pin)
{
    bool ret = false;
    int32_t mutexRet = EBUSY;

    // Really, the only task that should ever ever use this transfer
    // function is sensor task, but hey, could change.
    // TODO if it turns out that we'll never call this from any other task,
    // remove the mutex all together
    mutexRet = pthread_mutex_trylock(&sensorSPIMutex);

    // Able to lock mutex, valid chip select, bus not busy
    if(0 == mutexRet && cs_pin >= 0 && current_CS < 0)
    {
        current_CS = cs_pin;

        GPIO_write(current_CS, CHIP_SELECT_LOW);

        ret = SPI_transfer(sensorSPIHandle, transaction);

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
    /* reset chip select to "not busy" */
    current_CS = -1;
}

