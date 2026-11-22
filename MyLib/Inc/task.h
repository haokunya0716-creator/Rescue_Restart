/*
 * task.h
 *
 *  Created on: May 5, 2025
 *      Author: gaoxi
 */

#ifndef INC_TASK_H_
#define INC_TASK_H_

#include "main.h"

#define PERIODIC(T) \
static uint32_t nxt = 0; \
if(HAL_GetTick() < nxt) return; \
nxt += (T);

/* T is a positive interval in milliseconds. The first call is due immediately. */
#define PERIODIC_START(NAME, T) \
    { \
        static uint8_t NAME##_started = 0U; \
        static uint32_t NAME##_last = 0U; \
        const uint32_t NAME##_now = HAL_GetTick(); \
        if (NAME##_started == 0U || (uint32_t)(NAME##_now - NAME##_last) >= (T)) { \
            NAME##_started = 1U; \
            NAME##_last = NAME##_now;

#define PERIODIC_END \
        } \
    }

#endif /* INC_TASK_H_ */
