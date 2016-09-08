/*
 * systemStatus.h
 *
 *  Created on: Jan 9, 2016
 *      Author: pavelgamov
 */

#ifndef SYSTEMSTATUS_H_
#define SYSTEMSTATUS_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f0xx.h"

typedef enum {
	INFORM_INIT,
	INFORM_IDLE,
	INFORM_SLEEP,
	INFORM_CONNECTION_LOST,
	INFORM_ERROR,
	INFORM_LAST
} systemStatus_t;

typedef void (*ledOutputControl_t)(FunctionalState);

void System_init(void);
void System_setStatusLedControl(ledOutputControl_t);
void System_setStatus(systemStatus_t);

void System_delayMsDummy(uint32_t delay);
uint32_t System_getUptime(void);

#ifdef __cplusplus
}
#endif

#endif /* SYSTEMSTATUS_H_ */

