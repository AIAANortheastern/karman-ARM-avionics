/*
 * debug_printf.c
 *
 *  Created on: Feb 8, 2018
 *      Author: Andrew
 */

#include "debug_printf.h"
#include <stdarg.h>
#include <pthread.h>
#include <ti/display/Display.h>

extern pthread_mutex_t gDisplayMuxtex;
extern Display_Handle gTheDisplay;

void debug_printf(char* fmt, ...)
{
    va_list va;
    va_start(va, fmt);

    pthread_mutex_lock(&gDisplayMuxtex);

    gTheDisplay->fxnTablePtr->vprintfFxn(gTheDisplay, 0, 0, fmt, va);

    pthread_mutex_unlock(&gDisplayMuxtex);

    va_end(va);
}

