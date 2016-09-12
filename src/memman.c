/*
 * memman.c
 *
 *  Created on: Apr 21, 2016
 *      Author: shapa
 */

#include "memman.h"
#include <stdlib.h>
#include <stdint.h>
#include "stm32f0xx.h"

void *MEMMAN_malloc(size_t size) {
	uint32_t primask = __get_PRIMASK();
	__disable_irq();
	void *ptr = malloc(size);
	if (!primask) {
		__enable_irq();
	}
	return ptr;
}


void MEMMAN_free(void *ptr) {
	uint32_t primask = __get_PRIMASK();
	__disable_irq();
	free (ptr);
	if (!primask) {
		__enable_irq();
	}
}
