/*
 * BNO055Task.cpp
 *
 *  Created on: Feb 11, 2018
 *      Author: Andrew
 */


#include <ti/drivers/I2C.h>
#include <stdint.h>
#include "Adafruit_BNO055.h"

#include "sensorDefs.h"
#include "IMUTask.h"
#include "debug_printf.h"

#include <unistd.h>

extern imu_sensor_data_t gSensorData;
//extern pthread_mutex_t gSensorDataMutex;

I2C_Handle imuI2CHandle;
Adafruit_BNO055 bno;

bool init_imu_task(void)
{
    bool ret = false;
    I2C_Params masterParams;

    I2C_init();

    I2C_Params_init(&masterParams);
    masterParams.transferMode = I2C_MODE_BLOCKING;
    masterParams.bitRate = I2C_100kHz;
    imuI2CHandle = I2C_open(IMU_I2C, &masterParams);


    if(imuI2CHandle)
    {
        bno.setBus(imuI2CHandle);
        ret = bno.begin(Adafruit_BNO055::OPERATION_MODE_AMG);
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
    if(!init_imu_task())
        while(1);


    for(;;)
    {
        // create vectors to store data from Adafruit
        // Possible vector values can be:
        // - VECTOR_ACCELEROMETER - m/s^2
        // - VECTOR_MAGNETOMETER  - uT
        // - VECTOR_GYROSCOPE     - rad/s
        // - VECTOR_EULER         - degrees
        // - VECTOR_LINEARACCEL   - m/s^2
        // - VECTOR_GRAVITY       - m/s^2
        //imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
        imu::Vector<3> accele = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
        imu::Vector<3> magnet = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
        imu::Vector<3> gyros = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

        //pthread_mutex_lock(&gSensorDataMutex);

        //gSensorData.imuData.euler.x=euler.x();
        //gSensorData.imuData.euler.y=euler.y();
        //gSensorData.imuData.euler.z=euler.z();
        gSensorData.accele.x=accele.x();
        gSensorData.accele.y=accele.y();
        gSensorData.accele.z=accele.z();
        gSensorData.magnet.x=magnet.x();
        gSensorData.magnet.y=magnet.y();
        gSensorData.magnet.z=magnet.z();
        gSensorData.gyros.x=gyros.x();
        gSensorData.gyros.y=gyros.y();
        gSensorData.gyros.z=gyros.z();

        //(&gSensorDataMutex);

        //debug_printf(const_cast<char *>("X: %f Y: %f Z: %f"), accele.x(), accele.y(), accele.z());

        /* Display calibration status for each sensor. */
        uint8_t system, gyro, accel, mag = 0;
        bno.getCalibration(&system, &gyro, &accel, &mag);

        //debug_printf(const_cast<char *>("CALIBRATION: Sys=%d Gyro=%d Accel=%d Mag=%d"), system, gyro, accel, mag);

        usleep(100000);
    }


}

