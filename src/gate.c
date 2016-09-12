/*
 * gate.c
 *
 *  Created on: Sep 9, 2016
 *      Author: shapa
 */


#include <stdbool.h>
#include <stddef.h>
#include <assert.h>

#include "bsp.h"
#include "systemStatus.h"
#include "timers.h"

#define DEBOUNCE_STATE_CHANGE 10*1000
#define USER_BLOCK_TIMEOUT (60*1000) // minute

static void setState(const _Bool isClose);
static void onUserBlockTimeout(uint32_t id, void *data);
static void onStateBlockTimeout(uint32_t id, void *data);

static struct {
	_Bool volatile requestedState;
	_Bool volatile realState;
	uint32_t userBlockTimId;
	uint32_t sateBlockTimId;
} s_gateControl;


void Gate_SetState(const _Bool newState) {
	setState(newState);
}

void Gate_onLedStateChange(const _Bool ledState) {
	s_gateControl.realState = ledState;
	if (ledState != s_gateControl.requestedState) {
		/* state changed by user */
		s_gateControl.requestedState = ledState;
		if (s_gateControl.userBlockTimId) {
			/* user toggled state. Unblock */
			onUserBlockTimeout(s_gateControl.userBlockTimId, 0);
		} else {
			s_gateControl.userBlockTimId = Timer_newArmed(USER_BLOCK_TIMEOUT, false, onUserBlockTimeout, NULL);
		}
	} else {
		/* success set */
		s_gateControl.userBlockTimId = Timer_newArmed(DEBOUNCE_STATE_CHANGE, false, onStateBlockTimeout, NULL);
	}
}

static void setState(_Bool isClose) {
	if (s_gateControl.realState ^ isClose) {
		s_gateControl.requestedState = isClose;
		if (s_gateControl.userBlockTimId) // active timer means user block
			return;
		BSP_PushButton();
	}
}

static void onUserBlockTimeout(uint32_t id, void *data) {
	(void)data;
	assert(s_gateControl.userBlockTimId == id);
	s_gateControl.userBlockTimId = INVALID_HANDLE;
	if (s_gateControl.realState ^ s_gateControl.requestedState) {
		setState(s_gateControl.requestedState);
	}
}

static void onStateBlockTimeout(uint32_t id, void *data) {
	(void)data;
	assert(s_gateControl.sateBlockTimId == id);
	s_gateControl.sateBlockTimId = INVALID_HANDLE;
	if (s_gateControl.realState ^ s_gateControl.requestedState) {
		setState(s_gateControl.requestedState);
	}
}
