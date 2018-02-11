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
    if(!init_imu_task())
        while(1);


    for(;;)
    {
        // Possible vector values can be:
        // - VECTOR_ACCELEROMETER - m/s^2
        // - VECTOR_MAGNETOMETER  - uT
        // - VECTOR_GYROSCOPE     - rad/s
        // - VECTOR_EULER         - degrees
        // - VECTOR_LINEARACCEL   - m/s^2
        // - VECTOR_GRAVITY       - m/s^2
        imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

        debug_printf(const_cast<char *>("X: %f Y: %f Z: %f"), euler.x(), euler.y(), euler.z());

        /* Display calibration status for each sensor. */
        uint8_t system, gyro, accel, mag = 0;
        bno.getCalibration(&system, &gyro, &accel, &mag);

        debug_printf(const_cast<char *>("CALIBRATION: Sys=%d Gyro=%d Accel=%d Mag=%d"), system, gyro, accel, mag);

        usleep(100000);
    }


}

