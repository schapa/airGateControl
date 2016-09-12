/*
 * timers.c
 *
 *  Created on: September 12, 2016
 *      Author: shapa
 */

#include "timers.h"
#include "memman.h"
#include "stm32f0xx.h"

typedef enum {
	IS_ACTIVE = (1<<0),
	IS_PERIODIC = (1<<1),

	IS_IN_CALLBACK = (1<<16),
} timerFlags;

typedef struct timerNode {
	struct timerNode *next;
	uint32_t id;
	uint32_t timeout;
	timerFlags flags;
	onTimerFire_t cb;
	void *cbData;
	uint32_t cnt;
} timerNode_t, *timerNode_p;

static timerNode_p s_timersListHead = NULL;
static timerNode_p s_timersListTail = NULL;

static uint32_t getNewHandle(void);
static _Bool isTimerHandleUnique(uint32_t handle);
static timerNode_p findTimerById(uint32_t handle);


uint32_t Timer_new(uint32_t tout, _Bool isPeriodic, onTimerFire_t cb, void *cbData) {
	uint32_t handle = INVALID_HANDLE;
	if (!cb || !tout)
		return INVALID_HANDLE;
	do {
		handle = getNewHandle();
	} while (!isTimerHandleUnique(handle));
	timerNode_p last = s_timersListTail;
	uint32_t primask = __get_PRIMASK();
	__disable_irq();
	if (!last) {
		last = MEMMAN_malloc(sizeof(timerNode_t));
		if (!last) {
			return 0;
		}
		s_timersListTail = s_timersListHead = last;
	} else {
		s_timersListTail = MEMMAN_malloc(sizeof(timerNode_t));
		if (!s_timersListTail) {
			return 0;
		}
		last->next = s_timersListTail;
		last = s_timersListTail;
	}
	last->id = handle;
	last->timeout = tout;
	last->flags = isPeriodic ? IS_PERIODIC : 0;
	last->cb = cb;
	last->cbData = cbData;
	last->next = NULL;
	if (!primask) {
		__enable_irq();
	}
}

uint32_t Timer_newArmed(uint32_t tout, _Bool isPeriodic, onTimerFire_t cb, void *cbData) {
	uint32_t handle = Timer_new(tout, isPeriodic, cb, cbData);
	Timer_rearm(handle);
	return handle;
}

void Timer_rearm(uint32_t id) {
	if (id == INVALID_HANDLE)
		return;
	uint32_t primask = __get_PRIMASK();
	__disable_irq();
	timerNode_p node = findTimerById(id);
	if (node) {
		node->flags |= IS_ACTIVE;
		node->cnt = node->timeout;
	}
	if (!primask) {
		__enable_irq();
	}
}

void Timer_disarm(uint32_t id) {
	if (id == INVALID_HANDLE)
		return;
	uint32_t primask = __get_PRIMASK();
	__disable_irq();
	timerNode_p node = s_timersListHead;
	while (node) {
		if (node->id == id) {
			node->flags &= ~IS_ACTIVE;
			break;
		}
		node = node->next;
	}
	if (!primask) {
		__enable_irq();
	}
}

void Timer_delete(uint32_t id) {
	if (id == INVALID_HANDLE)
		return;
	timerNode_p node = s_timersListHead;
	timerNode_p prev = NULL;
	uint32_t primask = __get_PRIMASK();
	__disable_irq();
	while (node) {
		if ((node->id == id) && !(node->flags & IS_IN_CALLBACK)) {
			break;
		}
		prev = node;
		node = node->next;
	}
	if (node) {
		if (prev) {
			prev->next = node->next;
			if (node == s_timersListTail)
				s_timersListTail = prev;
			MEMMAN_free(node);
		} else {
			MEMMAN_free(s_timersListHead);
			s_timersListHead = s_timersListTail = NULL;
		}
	}
	if (!primask) {
		__enable_irq();
	}
}

void Timer_makeTick(void) {
	timerNode_p node = s_timersListHead;
	uint32_t primask = __get_PRIMASK();
	__disable_irq();
	while (node) {
		if ((node->flags & IS_ACTIVE) && !node->cnt--) {
			node->flags = IS_IN_CALLBACK | (node->flags & ~IS_ACTIVE);
			node->cb(node->id, node->cbData);
			node->flags &= ~IS_IN_CALLBACK;
			if (node->flags & IS_PERIODIC) {
				node->flags |= IS_ACTIVE;
				node->cnt = node->timeout;
			} else if (!(node->flags & IS_ACTIVE)) { // could be rearmed
				uint32_t id = node->id;
				node = node->next;
				Timer_delete(id);
				continue;
			}
		}
		node = node->next;
	}
	if (!primask) {
		__enable_irq();
	}
}

static uint32_t getNewHandle(void) {
	static uint32_t handle = 0;
	if (!handle || !++handle)
		handle = 1;
	return handle;
}

static _Bool isTimerHandleUnique(uint32_t handle) {
	timerNode_p node = s_timersListHead;
	while (node) {
		if (node->id == handle)
			return false;
		node = node->next;
	}
	return true;
}

static timerNode_p findTimerById(uint32_t handle) {
	timerNode_p node = s_timersListHead;
	while (node) {
		if (handle == node->id)
			return node;
		node = node->next;
	}
	return NULL;
}
