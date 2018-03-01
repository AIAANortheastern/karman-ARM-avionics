/*
 * appDefs.h
 *
 *  Created on: Jan 31, 2018
 *      Author: Andrew Kaster
 */

#ifndef INC_APPDEFS_H_
#define INC_APPDEFS_H_

#include <ti/Display/display.h>
#include <ti/drivers/Timer.h>
#include <pthread.h>
#include <time.h>
#include <stdbool.h>

#include <FreeRTOS.h>
#include <queue.h>

extern Display_Handle gTheDisplay;
extern pthread_mutex_t gDisplayMuxtex;
extern QueueHandle_t gQueueSensorRadio;
extern QueueHandle_t gQueueIMUSensor;
extern pthread_barrier_t startThreadBarrier;

void appInit(void);

/* Callback used for toggling the LED. */
void timerCallback(Timer_Handle myHandle);

/**
 * @brief check if correct number of milliseconds have elapsed
 *
 * @return true if elapsed, false if not elapsed
 */
bool timespec_compare(struct timespec *start, struct timespec *stop,
                   int32_t milliseconds);

#endif /* INC_APPDEFS_H_ */
