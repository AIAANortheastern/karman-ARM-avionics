/*
 * IMUTask.h
 *
 *  Created on: Feb 11, 2018
 *      Author: Andrew
 */

#ifndef INC_SENSORS_IMUTASK_H_
#define INC_SENSORS_IMUTASK_H_

#ifdef __cplusplus
extern "C" {
#endif

void *IMUTask(void *arg0);
bool init_imu_task(void);

typedef struct imu_data_s
{
    double Accel[3];
    double Gyro[3];
    double Mag[3];
} imu_data_t;



#ifdef __cplusplus
}
#endif

#endif /* INC_SENSORS_IMUTASK_H_ */
