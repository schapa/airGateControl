/*
 * bsp.h
 *
 *  Created on: Sep 8, 2016
 *      Author: shapa
 */

#ifndef BSP_H_
#define BSP_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f0xx.h"
#include <stdbool.h>
//#include "Queue.h"

typedef void (*control_f) (FunctionalState);
typedef bool (*getState_f) (void);
typedef bool (*sendData_f) (uint32_t id, uint8_t *data, uint8_t size);
typedef bool (*sendAirQuality_f) (uint8_t value);
typedef bool (*sendHartbeat_f) (void);
typedef struct {
	struct {
		control_f setSTB;
		control_f setEN;
		getState_f getERR;
	} hardwareLine;
	sendData_f sendData;
} ifaceControl_t, *ifaceControl_p;

/* Should be called Once */
void BSP_init(void);
void BSP_start(void);
uint8_t BSP_startCAN(void);
ifaceControl_p BSP_CANControl(void);


void BSP_SetLedState(FunctionalState);
void BSP_SetGateState(_Bool isOpen);
void BSP_GatePeriodic(void);



#ifdef __cplusplus
}
#endif

#endif /* BSP_H_ */
