/*
 BitzOS (BOS) V0.2.9 - Copyright (C) 2017-2023 Hexabitz
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
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2; /* Timer for RGB */
extern void MX_TIM1_Init(void);
extern void MX_TIM2_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
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
void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};
  TIM_ClockConfigTypeDef sClockSourceConfig;
  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = (uint32_t )(HAL_RCC_GetSysClockFreq() / PWM_TIMER_CLOCK) - 1;;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period =1023;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	HAL_TIM_ConfigClockSource(&htim1,&sClockSourceConfig);
  HAL_TIM_PWM_Init(&htim1);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig);

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1);

  HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3);

  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig);

  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);
  HAL_NVIC_SetPriority(TIM1_BRK_UP_TRG_COM_IRQn,0,0);
  	HAL_NVIC_EnableIRQ(TIM1_BRK_UP_TRG_COM_IRQn);
  	HAL_TIM_PWM_ConfigChannel(&htim1,&sConfigOC,_RGB_RED_TIM_CH);
  	HAL_TIM_PWM_ConfigChannel(&htim1,&sConfigOC,_RGB_BLUE_TIM_CH);
}
/* TIM2 init function */
void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_ClockConfigTypeDef sClockSourceConfig;
  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = (uint32_t )(HAL_RCC_GetSysClockFreq() / PWM_TIMER_CLOCK) - 1;;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1023;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  HAL_TIM_PWM_Init(&htim2);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig);

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 4294967295;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1);

  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  	HAL_TIM_ConfigClockSource(&htim2,&sClockSourceConfig);
  HAL_TIM_MspPostInit(&htim2);
  HAL_NVIC_SetPriority(TIM2_IRQn,0,0);
  	HAL_NVIC_EnableIRQ(TIM2_IRQn);
	HAL_TIM_PWM_ConfigChannel(&htim2,&sConfigOC,_RGB_GREEN_TIM_CH);
}
void HAL_TIM_MspPostInit(TIM_HandleTypeDef* timHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(timHandle->Instance==TIM1)
  {
  /* USER CODE BEGIN TIM1_MspPostInit 0 */

  /* USER CODE END TIM1_MspPostInit 0 */
    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**TIM1 GPIO Configuration
    PB1     ------> TIM1_CH3N
    PB13     ------> TIM1_CH1N
    */
    GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN TIM1_MspPostInit 1 */

  /* USER CODE END TIM1_MspPostInit 1 */
  }
  else if(timHandle->Instance==TIM2)
  {
  /* USER CODE BEGIN TIM2_MspPostInit 0 */

  /* USER CODE END TIM2_MspPostInit 0 */

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**TIM2 GPIO Configuration
    PA5     ------> TIM2_CH1
    */
    GPIO_InitStruct.Pin = GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN TIM2_MspPostInit 1 */

  /* USER CODE END TIM2_MspPostInit 1 */
  }

}
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
