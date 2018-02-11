/*
 * debug_printf.h
 *
 *  Created on: Feb 8, 2018
 *      Author: Andrew
 */

#ifndef INC_COMMON_DEBUG_PRINTF_H_
#define INC_COMMON_DEBUG_PRINTF_H_

#include <stdarg.h>
#include <ti/display/Display.h>

#ifdef __cplusplus
extern "C" {
#endif

void debug_printf(char* fmt, ...);

#ifdef __cplusplus
}
#endif

#endif /* INC_COMMON_DEBUG_PRINTF_H_ */
