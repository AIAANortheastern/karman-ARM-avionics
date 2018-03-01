/*
 * radioTask.h
 *
 *  Created on: Feb 8, 2018
 *      Author: Andrew Kaster
 */

#ifndef INC_RADIO_RADIOTASK_H_
#define INC_RADIO_RADIOTASK_H_

#ifdef __cplusplus
extern "C" {
#endif

int init_radio(void);

void *radioTask(void *arg0);

#ifdef __cplusplus
}
#endif
#endif /* INC_RADIO_RADIOTASK_H_ */
