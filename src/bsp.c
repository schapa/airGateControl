/*
 * bsp.c
 *
 *  Created on: Dec 28, 2015
 *      Author: shapa
 */

#include "stm32f0xx_rcc.h"
#include "stm32f0xx_gpio.h"
#include "stm32f0xx_tim.h"
#include "stm32f0xx_can.h"
#include "stm32f0xx_exti.h"
#include "stm32f0xx_syscfg.h"
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>

#include "bsp.h"
#include "systemStatus.h"

#define DEVICE_CAN_ID 0x135

static void initialize_RCC(void);
static void initialize_GPIO_CAN(void);
static void initialize_GPIO_LED(void);
static void initialize_GPIO_CONTROL(void);

static uint8_t configure_CAN(void);
static void configure_CAN_NVIC(void);
static void configure_GPIO_NVIC(void);
static void configure_TIM1(void);

static void setSTBState(FunctionalState);
static void setENState(FunctionalState);
static void wakeUpTrasciever();
static bool getERRState(void);

static bool sendData(uint32_t id, uint8_t *data, uint8_t size);

static ifaceControl_t s_canInterface = {
		{setSTBState, setENState, getERRState},
		.sendData = sendData,
};
static bool s_isInitialized = false;

//static volatile EventQueue_p s_eventQueue;

void BSP_init(void) {

	System_init();
	System_setStatusLedControl(BSP_SetLedState);
	initialize_RCC();
	initialize_GPIO_CAN();
	initialize_GPIO_LED();
	initialize_GPIO_CONTROL();
	configure_CAN_NVIC();
	configure_GPIO_NVIC();
	configure_TIM1();
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

void BSP_SetButtonState(FunctionalState state) {
	BitAction val = (state == DISABLE) ? Bit_RESET : Bit_SET;
	GPIO_WriteBit(GPIOA, GPIO_Pin_1, val);
}
//void BSP_queuePush(Event_p pEvent) {
//	uint32_t primask = __get_PRIMASK();
//	__disable_irq();
//	s_eventQueue = Queue_pushEvent(s_eventQueue, pEvent);
//	if (!primask) {
//		__enable_irq();
//	}
//}
//
//void BSP_pendEvent(Event_p pEvent) {
//	while (!s_eventQueue);
//	uint32_t primask = __get_PRIMASK();
//	__disable_irq();
//	s_eventQueue = Queue_getEvent(s_eventQueue, pEvent);
//	if (!primask) {
//		__enable_irq();
//	}
//}

/* private */
static void initialize_RCC(void) {

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

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

static void configure_TIM1(void) {
//	TIM_TimeBaseInitTypeDef iface;
//	iface.TIM_ClockDivision = TIM_CKD_DIV4;
//	iface.TIM_CounterMode = TIM_CounterMode_Up;
//	iface.TIM_Period = 0xFF;
//	iface.TIM_Prescaler = 470;
//	iface.TIM_RepetitionCounter = 0;
//
//	TIM_TimeBaseInit(TIM1, &iface);
//	TIM_SetAutoreload(TIM1, iface.TIM_Period);
//	TIM_SelectOutputTrigger(TIM1, TIM_TRGOSource_Update);
//
//	TIM_Cmd(TIM1, ENABLE);
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
		BSP_SetButtonState(!GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_1));
		EXTI_ClearFlag(EXTI_Line2);
	}
}
