
#include <stdio.h>
#include <stdlib.h>
#include "diag/Trace.h"

#include "bsp.h"
#include "stm32f0xx_dbgmcu.h"

int main(int argc, char* argv[]) {
	(void)argc;
	(void)argv;

	switch (DBGMCU_GetDEVID()) {
		case 0x444:
			/* system is STM32F03x */
			BSP_start();
			break;
		case 0x445:
			/* system is STM32F04x */
			BSP_startCAN();
			break;
	}

	while(1);

	return 0;
}
