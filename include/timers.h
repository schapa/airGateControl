/*
 * timers.h
 *
 *  Created on: September 12, 2016
 *      Author: shapa
 */

#ifndef TIMERS_H_
#define TIMERS_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

#define INVALID_HANDLE (0)

typedef void (*onTimerFire_t)(uint32_t id, void *data);

uint32_t Timer_new(uint32_t tout, _Bool isPeriodic, onTimerFire_t cb, void *cbData);
uint32_t Timer_newArmed(uint32_t tout, _Bool isPeriodic, onTimerFire_t cb, void *cbData);
void Timer_delete(uint32_t id);
void Timer_rearm(uint32_t id);
void Timer_disarm(uint32_t id);
void Timer_makeTick(void);

#ifdef __cplusplus
}
#endif

#endif /* TIMERS_H_ */
