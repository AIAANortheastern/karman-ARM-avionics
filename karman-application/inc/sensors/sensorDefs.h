/**
 * @file SensorDefs.h
 *
 * @brief Put all common sensor declarations here
 *
 * Should hold all data structures that depends on every sensor,
 * or on multiple sensors.
 *
 * Created: 2/15/2017 8:14:23 PM
 *  Author: Andrew
 */


#ifndef SENSORDEFS_H_
#define SENSORDEFS_H_

#include "ms5607-02ba03.h"
//#include "BMX055Mag.h"
//#include "BMX005Gyro.h"
#include "Board.h"

#include <ti/drivers/SPI.h>
#include <pthread.h>

/* chip select defines */
#define ALTIMETER_CS Board_SPI_CS1
#define IMU_I2C Board_I2C_TMP

#define CHIP_SELECT_LOW (0)
#define CHIP_SELECT_HIGH (1)

/** contains data for every sensor */
typedef struct
{
    ms5607_02ba03_data_t altimeter; /**< Temp and pressure */
    //bmx055_mag_data_t magnetometer; /**< Magnetometer data */
    //gyro_data_raw_t gyro; /**< Gyro data */
    /* TODO add all sensors' data */
} sensor_data_t;

extern SPI_Handle sensorSPIHandle;
extern pthread_mutex_t sensorSPIMutex;


#endif /* SENSORDEFS_H_ */
