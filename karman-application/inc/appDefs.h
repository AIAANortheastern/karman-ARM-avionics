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

extern Display_Handle gTheDisplay;
extern pthread_mutex_t gDisplayMuxtex;

void appInit(void);

/* Callback used for toggling the LED. */
void timerCallback(Timer_Handle myHandle);

#endif /* INC_APPDEFS_H_ */
