/*
 * gate.c
 *
 *  Created on: Sep 9, 2016
 *      Author: shapa
 */


#include <stdbool.h>
#include <stddef.h>

#include "bsp.h"
#include "systemStatus.h"

#define CAN_CONTROL_UNIT_ID 0x50
#define CAN_SENSOR_ID 0x135
#define CAN_DEVICE_TIMEOUT (1000) // 1 sec

#define DEBOUNCE_PUSH_BUTTON 100
#define DEBOUNCE_STATE_CHANGE 10*1000
#define DEBOUNCE_USER_STATE_CHANGE (60*1000) // minute

static void setState(const _Bool isOpen);
static void releaseButton(void);
static void pushButton(void);

static struct {
	_Bool volatile requestedState;
	_Bool volatile realState;
	size_t volatile debounce;
} s_gateControl;

static size_t volatile s_canMsgTimeout = CAN_DEVICE_TIMEOUT;


void Gate_Periodic(void) {
	if (s_gateControl.debounce) {
		s_gateControl.debounce--;
	} else if (s_gateControl.realState != s_gateControl.requestedState) {
		_Bool spike = s_gateControl.requestedState;
//		s_gateControl.requestedState = !s_gateControl.requestedState;
//		setState(spike);
	} else {
		releaseButton();
	}
	if (s_canMsgTimeout) {
		System_setStatus(s_gateControl.realState ? INFORM_SLEEP : INFORM_IDLE);
		s_canMsgTimeout--;
	} else {
		System_setStatus(INFORM_CONNECTION_LOST);
		BSP_CANControl()->hardwareLine.setSTB(ENABLE);
//		System_delayMsDummy(20);
	}
}

void Gate_onLedStateChange(const _Bool ledState) {
	if (s_gateControl.realState == s_gateControl.requestedState) {
		/* state changed by user */
		s_gateControl.realState = s_gateControl.requestedState = ledState;
		s_gateControl.debounce = DEBOUNCE_USER_STATE_CHANGE;
	} else {
		s_gateControl.realState = s_gateControl.requestedState;
		s_gateControl.debounce = DEBOUNCE_STATE_CHANGE;
	}
}

void Gate_onCanRx(const CanRxMsg *rx) {
	switch (rx->StdId) {
	case CAN_SENSOR_ID:
		setState(rx->Data[0] < 12);
	case CAN_CONTROL_UNIT_ID:
		s_canMsgTimeout = CAN_DEVICE_TIMEOUT;
		break;
	}
	s_canMsgTimeout = CAN_DEVICE_TIMEOUT;
}

static void setState(_Bool isOpen) {
	if (s_gateControl.requestedState != isOpen) {
		s_gateControl.realState = !BSP_GetLedState();
		if ((s_gateControl.realState == isOpen) && !s_gateControl.debounce) {
			/* State change to real state, Just exit */
			s_gateControl.debounce = 0;
			s_gateControl.requestedState = s_gateControl.realState;
			releaseButton();
			return;
		}
		s_gateControl.requestedState = isOpen;
		if (s_gateControl.debounce) {
			/* already waiting. Exit */
			return;
		}
		pushButton();
	}
}

static void releaseButton(void) {
	if (GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_1) != Bit_RESET) {
		GPIO_WriteBit(GPIOA, GPIO_Pin_1, Bit_RESET);
		s_gateControl.debounce = DEBOUNCE_STATE_CHANGE;
	}
}

static void pushButton(void) {
	s_gateControl.debounce = DEBOUNCE_PUSH_BUTTON;
	GPIO_WriteBit(GPIOA, GPIO_Pin_1, Bit_SET);
}
