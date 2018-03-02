/*
 * BNO055Task.cpp
 *
 *  Created on: Feb 11, 2018
 *      Author: Andrew
 */


#include <ti/drivers/I2C.h>
#include <stdint.h>
#include "Adafruit_BNO055.h"

#include "appDefs.h"
#include "sensorDefs.h"
#include "IMUTask.h"
#include "debug_printf.h"

#include <unistd.h>

#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>

#define QUEUE_DEPTH 16

I2C_Handle imuI2CHandle;
Adafruit_BNO055 bno;
QueueHandle_t gQueueIMUSensor;
imu_data_t currIMUData;

bool init_imu_task(void)
{
    bool ret = false;
    I2C_Params masterParams;

    gQueueIMUSensor = xQueueCreate(QUEUE_DEPTH, sizeof(imu_data_t));

    if(gQueueIMUSensor == NULL)
    {
        /* IMU queue unable to be created! */
        while(1);
    }

    I2C_init();

    I2C_Params_init(&masterParams);
    masterParams.transferMode = I2C_MODE_BLOCKING;
    masterParams.bitRate = I2C_100kHz;
    imuI2CHandle = I2C_open(IMU_I2C, &masterParams);

    if(imuI2CHandle)
    {
        bno.setBus(imuI2CHandle);
        ret = bno.begin();
        if(ret)
        {
            usleep(1000000);
            debug_printf(const_cast<char *>("temperature: %d C"), bno.getTemp());
            bno.setExtCrystalUse(true);
        }
    }

    return ret;

}


void *IMUTask(void *arg0)
{
    int task_iterations = 0;
    if(!init_imu_task())
        while(1);

    pthread_barrier_wait(&startThreadBarrier);

    for(;;)
    {
        TickType_t xLastWaketime = xTaskGetTickCount();
        TickType_t xFrequency = portTICK_PERIOD_MS * 20;

        // Possible vector values can be:
        // - VECTOR_ACCELEROMETER - m/s^2
        // - VECTOR_MAGNETOMETER  - uT
        // - VECTOR_GYROSCOPE     - rad/s
        // - VECTOR_EULER         - degrees
        // - VECTOR_LINEARACCEL   - m/s^2
        // - VECTOR_GRAVITY       - m/s^2
        imu::Vector<3> vector = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);

        currIMUData.Accel[0] = vector[0];
        currIMUData.Accel[1] = vector[1];
        currIMUData.Accel[2] = vector[2];

        //debug_printf(const_cast<char *>("X: %f Y: %f Z: %f"), vector.x(), vector.y(), vector.z());

        vector = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

        currIMUData.Gyro[0] = vector[0];
        currIMUData.Gyro[1] = vector[1];
        currIMUData.Gyro[2] = vector[2];

        vector = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);

        currIMUData.Mag[0] = vector[0];
        currIMUData.Mag[1] = vector[1];
        currIMUData.Mag[2] = vector[2];

        // Transmit into queue for sensor task to aggregate
        if(pdFALSE == xQueueSend(gQueueIMUSensor, (void *) &currIMUData, (TickType_t) 0))
        {
            debug_printf(const_cast<char *>("Failed to write into IMU to Sensor Task Queue\n"));
        }

        debug_printf(const_cast<char *>("IMUTASK %d\n"), task_iterations);
        task_iterations++;
        // sleep until the next 20 milliseconds
        vTaskDelayUntil( &xLastWaketime, xFrequency );
    }


}

