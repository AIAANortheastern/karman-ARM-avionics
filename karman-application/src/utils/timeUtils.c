/*
 * timeUtils.c
 *
 *  Created on: Feb 7, 2018
 *      Author: Andrew Kaster
 */

#include <time.h>
#include <stdbool.h>

bool timespec_compare(struct timespec *start, struct timespec *stop,
                   int32_t milliseconds)
{
    struct timespec result;
    bool ret = false;

    /* store difference */
    if ((stop->tv_nsec - start->tv_nsec) < 0) {
        result.tv_sec = stop->tv_sec - start->tv_sec - 1;
        result.tv_nsec = stop->tv_nsec - start->tv_nsec + 1000000000;
    } else {
        result.tv_sec = stop->tv_sec - start->tv_sec;
        result.tv_nsec = stop->tv_nsec - start->tv_nsec;
    }

    /* compare to input */
    if((1000*result.tv_sec + (result.tv_nsec)/1000000) - milliseconds > 0)
    {
        ret = true;
    }

    return ret;
}

