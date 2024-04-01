/*
 BitzOS (BOS) V0.3.2 - Copyright (C) 2017-2024 Hexabitz
 All rights reserved

 File Name     : H1FR5_timers.c
 Description   : Peripheral timers setup source file.

 Required MCU resources :

 >> Timer 14 for micro-sec delay.
 >> Timer 15 for milli-sec delay.

 */

/* Includes ------------------------------------------------------------------*/
#include "BOS.h"

/*----------------------------------------------------------------------------*/
/* Configure Timers                                                              */
/*----------------------------------------------------------------------------*/

/* Variables ---------------------------------------------------------*/
GPIO_InitTypeDef GPIO_InitStruct = {0};
//TIM_HandleTypeDef htim14; /* micro-second delay counter */
TIM_HandleTypeDef htim16; /* micro-second delay counter */
//TIM_HandleTypeDef htim15; /* milli-second delay counter */
TIM_HandleTypeDef htim17; /* milli-second delay counter */

/*  Micro-seconds timebase init function - TIM14 (16-bit)
 */
void TIM_USEC_Init(void){
//	TIM_MasterConfigTypeDef sMasterConfig;
//
//	/* Peripheral clock enable */
//	__TIM14_CLK_ENABLE();
//
//	/* Peripheral configuration */
//	htim14.Instance = TIM14;
//	htim14.Init.Prescaler =HAL_RCC_GetPCLK1Freq() / 1000000;
//	htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
//	htim14.Init.Period =0xFFFF;
//	HAL_TIM_Base_Init(&htim14);
//
//	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
//	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
//	HAL_TIMEx_MasterConfigSynchronization(&htim14,&sMasterConfig);
//
//	HAL_TIM_Base_Start(&htim14);


	  __TIM16_CLK_ENABLE();

	  htim16.Instance = TIM16;
	  htim16.Init.Prescaler = 47;
	  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
	  htim16.Init.Period = 0XFFFF;
	  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	  htim16.Init.RepetitionCounter = 0;
	  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	  HAL_TIM_Base_Init(&htim16);

	  HAL_TIM_Base_Start(&htim16);

}

/*-----------------------------------------------------------*/

/*-----------------------------------------------------------*/

/*  Milli-seconds timebase init function - TIM15 (16-bit)
 */
void TIM_MSEC_Init(void){
//	TIM_MasterConfigTypeDef sMasterConfig;
//
//	/* Peripheral clock enable */
//	__TIM15_CLK_ENABLE();
//
//	/* Peripheral configuration */
//	htim15.Instance = TIM15;
//	htim15.Init.Prescaler =HAL_RCC_GetPCLK1Freq() / 1000;
//	htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
//	htim15.Init.Period =0xFFFF;
//	HAL_TIM_Base_Init(&htim15);
//
//	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
//	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
//	HAL_TIMEx_MasterConfigSynchronization(&htim15,&sMasterConfig);
	
	  __TIM17_CLK_ENABLE();
	  htim17.Instance = TIM17;
	  htim17.Init.Prescaler = 47999;
	  htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
	  htim17.Init.Period = 0xFFFF;
	  htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	  htim17.Init.RepetitionCounter = 0;
	  htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	  HAL_TIM_Base_Init(&htim17);

	  HAL_TIM_Base_Start(&htim17);
}

/*-----------------------------------------------------------*/

/* --- Load and start micro-second delay counter --- 
 */
void StartMicroDelay(uint16_t Delay){
	uint32_t t0 =0;
	
	portENTER_CRITICAL();
	
	if(Delay){
		t0 =htim16.Instance->CNT;
		
		while(htim16.Instance->CNT - t0 <= Delay){};
	}

	portEXIT_CRITICAL();
}

/*-----------------------------------------------------------*/

/* --- Load and start milli-second delay counter --- 
 */
void StartMilliDelay(uint16_t Delay){
	uint32_t t0 =0;
	
	portENTER_CRITICAL();
	
	if(Delay){
		t0 =htim17.Instance->CNT;
		
		while(htim17.Instance->CNT - t0 <= Delay){};
	}

	portEXIT_CRITICAL();
}
/*-----------------------------------------------------------*/

/************************ (C) COPYRIGHT HEXABITZ *****END OF FILE****/
