/*
 * appGlobals.c
 *
 *  Created on: Jan 31, 2018
 *      Author: Andrew Kaster
 *
 *   Global Variables.
 *   Defined in appDefs.h
 */

#include "appDefs.h"

Display_Handle gTheDisplay;
pthread_mutex_t gDisplayMuxtex;
QueueHandle_t gQueueSensorRadio;
pthread_barrier_t startThreadBarrier;
