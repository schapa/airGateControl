/*
 * bsp.c
 *
 *  Created on: Dec 28, 2015
 *      Author: shapa
 */

#include "stm32f0xx_rcc.h"
#include "stm32f0xx_gpio.h"
#include "stm32f0xx_can.h"
#include "stm32f0xx_exti.h"
#include "stm32f0xx_syscfg.h"
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>

#include "bsp.h"
#include "systemStatus.h"

#define CAN_DEVICE_ID 0x135
#define CAN_DEVICE_TIMEOUT (6*1000) // minute

#define DEBOUNCE_PUSH_BUTTON 10
#define DEBOUNCE_STATE_CHANGE 10*1000
#define DEBOUNCE_USER_STATE_CHANGE (60*1000) // minute

static void initialize_RCC(void);
static void initialize_GPIO_CAN(void);
static void initialize_GPIO_LED(void);
static void initialize_GPIO_CONTROL(void);

static uint8_t configure_CAN(void);
static void configure_CAN_NVIC(void);
static void configure_GPIO_NVIC(void);

static void setSTBState(FunctionalState);
static void setENState(FunctionalState);
static void wakeUpTrasciever();
static bool getERRState(void);

static bool sendData(uint32_t id, uint8_t *data, uint8_t size);

static void releaseButton(void);
static void pushButton(void);

static ifaceControl_t s_canInterface = {
		{setSTBState, setENState, getERRState},
		.sendData = sendData,
};
static bool s_isInitialized = false;

static struct {
	_Bool volatile requestedState;
	_Bool volatile realState;
	size_t volatile debounce;
} s_gateControl;
static size_t volatile s_canMsgTimeout = CAN_DEVICE_TIMEOUT;

void BSP_init(void) {

	System_init();
	System_setStatusLedControl(BSP_SetLedState);
	initialize_RCC();
	initialize_GPIO_CAN();
	initialize_GPIO_LED();
	initialize_GPIO_CONTROL();
	configure_CAN_NVIC();
	configure_GPIO_NVIC();
	s_isInitialized = true;
}

void BSP_start(void) {
	if (!s_isInitialized) {
		BSP_init();
	}
	System_setStatus(INFORM_INIT);
}

uint8_t BSP_startCAN(void) {
	uint8_t result = true;
	if (!s_isInitialized) {
		BSP_init();
	}
	System_setStatus(INFORM_INIT);
	result &= configure_CAN();
	return result;
}

ifaceControl_p BSP_CANControl(void) {
	return &s_canInterface;
}

void BSP_SetLedState(FunctionalState state) {
	BitAction val = (state == DISABLE) ? Bit_RESET : Bit_SET;
	GPIO_WriteBit(GPIOA, GPIO_Pin_0, val);
}

void BSP_SetGateState(_Bool isOpen) {
	if (s_gateControl.requestedState != isOpen) {
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

void BSP_GatePeriodic(void) {
	if (s_gateControl.debounce) {
		s_gateControl.debounce--;
	} else {
		releaseButton();
	}
	if (s_canMsgTimeout) {
		s_canMsgTimeout--;
	} else {
		BSP_CANControl()->hardwareLine.setSTB(ENABLE);
		System_delayMsDummy(20);
	}
}

static void releaseButton(void) {
	if (GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_1) != Bit_RESET) {
		s_gateControl.debounce = DEBOUNCE_STATE_CHANGE;
		GPIO_WriteBit(GPIOA, GPIO_Pin_1, Bit_RESET);
	}
}

static void pushButton(void) {
	s_gateControl.debounce = DEBOUNCE_PUSH_BUTTON;
	GPIO_WriteBit(GPIOA, GPIO_Pin_1, Bit_SET);
}

/* private */
static void initialize_RCC(void) {

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN, ENABLE);

	GPIO_DeInit(GPIOA);
}

static void initialize_GPIO_CAN(void) {

	GPIO_InitTypeDef iface = {0};
	GPIO_InitTypeDef ifaceControl = {0};
	GPIO_InitTypeDef ifaceFeedback = {0};

	iface.GPIO_Mode = GPIO_Mode_AF;
	iface.GPIO_OType = GPIO_OType_PP;
	iface.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_12;
	iface.GPIO_PuPd = GPIO_PuPd_NOPULL;
	iface.GPIO_Speed = GPIO_Speed_Level_1;

	ifaceControl.GPIO_Mode = GPIO_Mode_OUT;
	ifaceControl.GPIO_OType = GPIO_OType_PP;
	ifaceControl.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
	ifaceControl.GPIO_PuPd = GPIO_PuPd_NOPULL;
	ifaceControl.GPIO_Speed = GPIO_Speed_Level_1;

	ifaceFeedback.GPIO_Mode = GPIO_Mode_IN;
	ifaceFeedback.GPIO_OType = GPIO_OType_PP;
	ifaceFeedback.GPIO_Pin = GPIO_Pin_1;
	ifaceFeedback.GPIO_PuPd = GPIO_PuPd_UP;
	ifaceFeedback.GPIO_Speed = GPIO_Speed_Level_1;

	/* remap pins */
	SYSCFG->CFGR1 |= (uint32_t)SYSCFG_CFGR1_PA11_PA12_RMP;

	GPIO_Init(GPIOA, &iface);
	GPIO_Init(GPIOA, &ifaceControl);
	GPIO_Init(GPIOB, &ifaceFeedback);

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource11, GPIO_AF_4);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource12, GPIO_AF_4);

	/* turn off transmitter */
	setSTBState(ENABLE);
	setENState(DISABLE);
}

static void initialize_GPIO_LED(void) {

	GPIO_InitTypeDef iface = {0};
	iface.GPIO_Mode = GPIO_Mode_OUT;
	iface.GPIO_OType = GPIO_OType_PP;
	iface.GPIO_Pin = GPIO_Pin_0;
	iface.GPIO_PuPd = GPIO_PuPd_NOPULL;
	iface.GPIO_Speed = GPIO_Speed_Level_1;
	GPIO_Init(GPIOA, &iface);
}

static void initialize_GPIO_CONTROL(void) {

	GPIO_InitTypeDef iface = {0};
	iface.GPIO_Mode = GPIO_Mode_OUT;
	iface.GPIO_OType = GPIO_OType_PP;
	iface.GPIO_Pin = GPIO_Pin_1;
	iface.GPIO_PuPd = GPIO_PuPd_NOPULL;
	iface.GPIO_Speed = GPIO_Speed_Level_1;
	GPIO_Init(GPIOA, &iface);

	iface.GPIO_Mode = GPIO_Mode_IN;
	iface.GPIO_Pin = GPIO_Pin_2;
	iface.GPIO_PuPd = GPIO_PuPd_UP;
	iface.GPIO_Speed = GPIO_Speed_Level_1;
	GPIO_Init(GPIOA, &iface);

	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource2);
}

static void setSTBState(FunctionalState state) {
	/* low means StandBy */
	BitAction val = (state == DISABLE) ? Bit_SET : Bit_RESET;
	GPIO_WriteBit(GPIOA, GPIO_Pin_7, val);
}

static void setENState(FunctionalState state) {
	BitAction val = (state == DISABLE) ? Bit_RESET : Bit_SET;
	GPIO_WriteBit(GPIOA, GPIO_Pin_6, val);
}

static void wakeUpTrasciever() {
	BitAction val = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_5) ? Bit_RESET : Bit_SET;
	GPIO_WriteBit(GPIOA, GPIO_Pin_5, val);
}

static bool getERRState(void) {
	/* low means Error, or WakeUp - handled by interrupt */
	return (bool)!GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_1);
}

static uint8_t configure_CAN(void) {
	uint8_t initResult = 0;
	CAN_InitTypeDef iface = {0};
	CAN_FilterInitTypeDef  ifaceFilter ={0};
	const uint32_t baudRate = 125000;
	RCC_ClocksTypeDef RCC_Clocks;

	wakeUpTrasciever();
	s_canInterface.hardwareLine.setSTB(DISABLE);
	s_canInterface.hardwareLine.setEN(ENABLE);
	RCC_GetClocksFreq(&RCC_Clocks);

	iface.CAN_TTCM = DISABLE;
	iface.CAN_ABOM = DISABLE;
//	iface.CAN_AWUM = ENABLE;
	iface.CAN_AWUM = DISABLE;
	iface.CAN_NART = ENABLE;
	iface.CAN_RFLM = DISABLE;
	iface.CAN_TXFP = DISABLE;
	iface.CAN_Mode = CAN_Mode_Normal;
	iface.CAN_SJW = CAN_SJW_1tq;
	iface.CAN_BS1 = CAN_BS1_4tq;
	iface.CAN_BS2 = CAN_BS2_3tq;
	iface.CAN_Prescaler = RCC_Clocks.PCLK_Frequency/(baudRate*(1+4+3)); //(CAN_SJW + CAN_BS1 + CAN_BS2)

	CAN_DeInit(CAN);
	initResult = CAN_Init(CAN, &iface);

	ifaceFilter.CAN_FilterNumber = 0;
	ifaceFilter.CAN_FilterMode = CAN_FilterMode_IdMask;
	ifaceFilter.CAN_FilterScale = CAN_FilterScale_32bit;
	ifaceFilter.CAN_FilterIdHigh = 0x0000;
	ifaceFilter.CAN_FilterIdLow = 0x0000;
	ifaceFilter.CAN_FilterMaskIdHigh = 0x0000;
	ifaceFilter.CAN_FilterMaskIdLow = 0x0000;
	ifaceFilter.CAN_FilterFIFOAssignment = CAN_FIFO0;
	ifaceFilter.CAN_FilterActivation = ENABLE;

	CAN_FilterInit(&ifaceFilter);

	CAN_ITConfig(CAN, CAN_IT_FMP0, ENABLE);
	CAN_ITConfig(CAN, CAN_IT_FMP1, ENABLE);
	CAN_ITConfig(CAN, CAN_IT_TME, ENABLE);

	CAN_ITConfig(CAN, CAN_IT_EWG, ENABLE);
	CAN_ITConfig(CAN, CAN_IT_EPV, ENABLE);
	CAN_ITConfig(CAN, CAN_IT_BOF, ENABLE);
	CAN_ITConfig(CAN, CAN_IT_LEC, ENABLE);
	CAN_ITConfig(CAN, CAN_IT_ERR, ENABLE);

	return initResult;
}

static void configure_CAN_NVIC(void){
	NVIC_InitTypeDef nvic = {
			CEC_CAN_IRQn,
			0,
			ENABLE
	};
	NVIC_Init(&nvic);
}

static void configure_GPIO_NVIC(void) {
	NVIC_InitTypeDef nvic = {
			EXTI2_3_IRQn,
			0,
			ENABLE
	};
	EXTI_InitTypeDef exti = {
			EXTI_Line2,
			EXTI_Mode_Interrupt,
			EXTI_Trigger_Rising_Falling,
			ENABLE
	};

	NVIC_Init(&nvic);
	EXTI_Init(&exti);
}

static bool sendData(uint32_t id, uint8_t *data, uint8_t size) {
	CanTxMsg txMess = {
			id,
			id,
			CAN_Id_Standard,
			CAN_RTR_Data,
			size,
			{0}
	};
	if (size > 8 || id > 0x1FFFFFFF)
		return false;

	txMess.RTR = (size || data) ? CAN_RTR_Data : CAN_RTR_Remote;
	txMess.IDE = id > 0x7FF ? CAN_Id_Extended : CAN_Id_Standard;
	memcpy(txMess.Data, data, size);

	return CAN_Transmit(CAN, &txMess) != CAN_TxStatus_NoMailBox;
}

void EXTI2_3_IRQHandler(void) {
	if (EXTI_GetFlagStatus(EXTI_Line2)) {
		const _Bool ledState = !GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_2);
		/* wait at least 1 sec */
		if (System_getUptime() > 1) {
			if (s_gateControl.realState == s_gateControl.requestedState) {
				/* state changed by user */
				s_gateControl.realState = s_gateControl.requestedState = ledState;
				s_gateControl.debounce = DEBOUNCE_USER_STATE_CHANGE;
			} else {
				s_gateControl.realState = s_gateControl.requestedState;
				s_gateControl.debounce = DEBOUNCE_STATE_CHANGE;
			}
		}
		EXTI_ClearFlag(EXTI_Line2);
	}
}

void CEC_CAN_IRQHandler(void) {

	if (CAN_GetITStatus(CAN, CAN_IT_FMP0)) {
		CanRxMsg rx = {0};
		CAN_Receive(CAN, CAN_FIFO0, &rx);
//		s_canMsgTimeout = CAN_DEVICE_TIMEOUT;
	}
	if (CAN_GetITStatus(CAN, CAN_IT_FMP1)) {
		trace_printf("CAN_IT_FMP1 \n");
	}
	if (CAN_GetITStatus(CAN, CAN_IT_TME)) {
		trace_printf("CAN_IT_TME \n");
		trace_printf("\t MAIL 0 %d\n", CAN_TransmitStatus(CAN, 0));
		trace_printf("\t MAIL 1 %d\n", CAN_TransmitStatus(CAN, 1));
		trace_printf("\t MAIL 2 %d\n", CAN_TransmitStatus(CAN, 2));
		CAN_ClearITPendingBit(CAN, CAN_IT_TME);
	}

	if (CAN_GetITStatus(CAN, CAN_IT_EWG)) {
		trace_printf("EWG \n");
		CAN_ClearITPendingBit(CAN, CAN_IT_EWG);
	}
	if (CAN_GetITStatus(CAN, CAN_IT_EPV)) {
		trace_printf("EPV \n");
		CAN_ClearITPendingBit(CAN, CAN_IT_EPV);
	}
	if (CAN_GetITStatus(CAN, CAN_IT_BOF)) {
		trace_printf("BOF \n");
		CAN_ClearITPendingBit(CAN, CAN_IT_BOF);
	}
	if (CAN_GetITStatus(CAN, CAN_IT_LEC)) {
		trace_printf("LEC \n");
		CAN_ClearITPendingBit(CAN, CAN_IT_LEC);
	}
	if (CAN_GetITStatus(CAN, CAN_IT_ERR)) {
		trace_printf("ERR \n");
		CAN_ClearITPendingBit(CAN, CAN_IT_ERR);
	}
}
