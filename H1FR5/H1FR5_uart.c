/*
 BitzOS (BOS) V0.3.6 - Copyright (C) 2017-2024 Hexabitz
 All rights reserved

 File Name     : H1FR5_uart.c
 Description   : Source Code provides configuration for USART instances.

 */

/* Includes ------------------------------------------------------------------*/
#include "BOS.h"
/*  */
//#ifndef __N
//uint16_t arrayPortsDir[MaxNumOfModules]; /* Array ports directions */
//#else
//uint16_t arrayPortsDir[__N ];
//#endif

DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart3_rx;
DMA_HandleTypeDef hdma_usart4_rx;
DMA_HandleTypeDef hdma_usart5_rx;
DMA_HandleTypeDef hdma_usart6_rx;

/* USART1 init function */
#ifdef _Usart1
void MX_USART1_UART_Init(void){
	  huart1.Instance = USART1;
	  huart1.Init.BaudRate = DEF_ARRAY_BAUDRATE;
	  huart1.Init.WordLength = UART_WORDLENGTH_8B;
	  huart1.Init.StopBits = UART_STOPBITS_1;
	  huart1.Init.Parity = UART_PARITY_NONE;
	  huart1.Init.Mode = UART_MODE_TX_RX;
	  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
	  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	  HAL_UART_Init(&huart1);
	  HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8);
	  HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8);
	  HAL_UARTEx_DisableFifoMode(&huart1);
#if _P4pol_reversed
		huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_SWAP_INIT;
		huart1.AdvancedInit.Swap = UART_ADVFEATURE_SWAP_ENABLE;
	  HAL_UART_Init(&huart1);
	#endif	
}
#endif

/* USART2 init function */
#ifdef _Usart2
void MX_USART2_UART_Init(void){
	  huart2.Instance = USART2;
	  huart2.Init.BaudRate = DEF_ARRAY_BAUDRATE;
	  huart2.Init.WordLength = UART_WORDLENGTH_8B;
	  huart2.Init.StopBits = UART_STOPBITS_1;
	  huart2.Init.Parity = UART_PARITY_NONE;
	  huart2.Init.Mode = UART_MODE_TX_RX;
	  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
	  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	  HAL_UART_Init(&huart2);
	  HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8);
	  HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8);
	  HAL_UARTEx_DisableFifoMode(&huart2);

#if _P2pol_reversed
		huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_SWAP_INIT;
		huart2.AdvancedInit.Swap = UART_ADVFEATURE_SWAP_ENABLE;
		HAL_UART_Init(&huart2);
	#endif	
}
#endif

/* USART3 init function */
#ifdef _Usart3
void MX_USART3_UART_Init(void){
	  huart3.Instance = USART3;
	  huart3.Init.BaudRate = DEF_ARRAY_BAUDRATE;
	  huart3.Init.WordLength = UART_WORDLENGTH_8B;
	  huart3.Init.StopBits = UART_STOPBITS_1;
	  huart3.Init.Parity = UART_PARITY_NONE;
	  huart3.Init.Mode = UART_MODE_TX_RX;
	  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
	  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
	  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	  HAL_UART_Init(&huart3);
	  HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8);
	  HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8);
	  HAL_UARTEx_DisableFifoMode(&huart3);

#if _P3pol_reversed
		huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_SWAP_INIT;
		huart3.AdvancedInit.Swap = UART_ADVFEATURE_SWAP_ENABLE;
	  HAL_UART_Init(&huart3);
	#endif	
}
#endif

/* USART4 init function */
#ifdef _Usart4
void MX_USART4_UART_Init(void){
	  huart4.Instance = USART4;
	  huart4.Init.BaudRate = DEF_ARRAY_BAUDRATE;
	  huart4.Init.WordLength = UART_WORDLENGTH_8B;
	  huart4.Init.StopBits = UART_STOPBITS_1;
	  huart4.Init.Parity = UART_PARITY_NONE;
	  huart4.Init.Mode = UART_MODE_TX_RX;
	  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
	  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	  huart4.Init.ClockPrescaler = UART_PRESCALER_DIV1;
	  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	  HAL_UART_Init(&huart4);

#if _P1pol_reversed
		huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_SWAP_INIT;
		huart4.AdvancedInit.Swap = UART_ADVFEATURE_SWAP_ENABLE;
	  HAL_UART_Init(&huart4);
	#endif	
}
#endif

/* USART5 init function */
#ifdef _Usart5
void MX_USART5_UART_Init(void){
	  huart5.Instance = USART5;
	  huart5.Init.BaudRate = 9600;
	  huart5.Init.WordLength = UART_WORDLENGTH_8B;
	  huart5.Init.StopBits = UART_STOPBITS_1;
	  huart5.Init.Parity = UART_PARITY_NONE;
	  huart5.Init.Mode = UART_MODE_TX_RX;
	  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
	  huart5.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	  huart5.Init.ClockPrescaler = UART_PRESCALER_DIV1;
	  huart5.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	HAL_UART_Init(&huart5);
#if _P6pol_reversed
		huart5.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_SWAP_INIT;
		huart5.AdvancedInit.Swap = UART_ADVFEATURE_SWAP_ENABLE;
	  HAL_UART_Init(&huart5);
	#endif	
}
#endif

/* USART6 init function */
#ifdef _Usart6
void MX_USART6_UART_Init(void){
	  huart6.Instance = USART6;
	  huart6.Init.BaudRate = DEF_ARRAY_BAUDRATE;
	  huart6.Init.WordLength = UART_WORDLENGTH_8B;
	  huart6.Init.StopBits = UART_STOPBITS_1;
	  huart6.Init.Parity = UART_PARITY_NONE;
	  huart6.Init.Mode = UART_MODE_TX_RX;
	  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
	  huart6.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	  huart6.Init.ClockPrescaler = UART_PRESCALER_DIV1;
	  huart6.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	  HAL_UART_Init(&huart6);

#if _P5pol_reversed
		huart6.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_SWAP_INIT;
		huart6.AdvancedInit.Swap = UART_ADVFEATURE_SWAP_ENABLE;
	  HAL_UART_Init(&huart6);
	#endif	
}
#endif

void HAL_UART_MspInit(UART_HandleTypeDef *huart){
	
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
	if(huart->Instance == USART1){
	#ifdef _Usart1
		 PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
		 PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_HSI;
		 HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);
		    /* USART1 clock enable */
		    __HAL_RCC_USART1_CLK_ENABLE();

		    __HAL_RCC_GPIOA_CLK_ENABLE();
		    /**USART1 GPIO Configuration
		    PA9     ------> USART1_TX
		    PA10     ------> USART1_RX
		    */
		    GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10;
		    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		    GPIO_InitStruct.Pull = GPIO_NOPULL;
		    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		    GPIO_InitStruct.Alternate = GPIO_AF1_USART1;
		    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

		    /* USART1 DMA Init */
		    /* USART1_RX Init */
		    hdma_usart1_rx.Instance = DMA1_Channel1;
		    hdma_usart1_rx.Init.Request = DMA_REQUEST_USART1_RX;
		    hdma_usart1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
		    hdma_usart1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
		    hdma_usart1_rx.Init.MemInc = DMA_MINC_ENABLE;
		    hdma_usart1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
		    hdma_usart1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
		    hdma_usart1_rx.Init.Mode = DMA_CIRCULAR;
		    hdma_usart1_rx.Init.Priority = DMA_PRIORITY_LOW;

		    UARTDMAHandler[(GetPort(huart)-1)] = &hdma_usart1_rx;

	  	    HAL_DMA_Init(&hdma_usart1_rx);
		    __HAL_LINKDMA(huart,hdmarx,hdma_usart1_rx);

		    /* USART1 interrupt Init */
		    HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
		    HAL_NVIC_EnableIRQ(USART1_IRQn);

		    __HAL_DMA_DISABLE_IT(&hdma_usart1_rx , DMA_IT_HT);

#endif
	}
	else if(huart->Instance == USART2){
	#ifdef _Usart2
		 PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
		    PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_HSI;
		    HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);
		    /* USART2 clock enable */
		    __HAL_RCC_USART2_CLK_ENABLE();

		    __HAL_RCC_GPIOA_CLK_ENABLE();
		    /**USART2 GPIO Configuration
		    PA2     ------> USART2_TX
		    PA3     ------> USART2_RX
		    */
		    GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
		    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		    GPIO_InitStruct.Pull = GPIO_NOPULL;
		    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		    GPIO_InitStruct.Alternate = GPIO_AF1_USART2;
		    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

		    /* USART2 DMA Init */
		    /* USART2_RX Init */
		    hdma_usart2_rx.Instance = DMA1_Channel2;
		    hdma_usart2_rx.Init.Request = DMA_REQUEST_USART2_RX;
		    hdma_usart2_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
		    hdma_usart2_rx.Init.PeriphInc = DMA_PINC_DISABLE;
		    hdma_usart2_rx.Init.MemInc = DMA_MINC_ENABLE;
		    hdma_usart2_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
		    hdma_usart2_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
		    hdma_usart2_rx.Init.Mode = DMA_CIRCULAR;
		    hdma_usart2_rx.Init.Priority = DMA_PRIORITY_LOW;

		    UARTDMAHandler[(GetPort(huart)-1)] = &hdma_usart2_rx;

		    HAL_DMA_Init(&hdma_usart2_rx);

		    __HAL_LINKDMA(huart,hdmarx,hdma_usart2_rx);

		    /* USART2 interrupt Init */
		    HAL_NVIC_SetPriority(USART2_LPUART2_IRQn, 0, 0);
		    HAL_NVIC_EnableIRQ(USART2_LPUART2_IRQn);

		    __HAL_DMA_DISABLE_IT(&hdma_usart2_rx , DMA_IT_HT);
#endif
	}
	else if(huart->Instance == USART3){
	#ifdef _Usart3
		 PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART3;
		 PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_HSI;
		 HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

		    /* USART3 clock enable */
		    __HAL_RCC_USART3_CLK_ENABLE();

		    __HAL_RCC_GPIOB_CLK_ENABLE();
		    /**USART3 GPIO Configuration
		    PB10     ------> USART3_TX
		    PB11     ------> USART3_RX
		    */
		    GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
		    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		    GPIO_InitStruct.Pull = GPIO_NOPULL;
		    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		    GPIO_InitStruct.Alternate = GPIO_AF4_USART3;
		    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

		    /* USART3 DMA Init */
		    /* USART3_RX Init */
		    hdma_usart3_rx.Instance = DMA1_Channel3;
		    hdma_usart3_rx.Init.Request = DMA_REQUEST_USART3_RX;
		    hdma_usart3_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
		    hdma_usart3_rx.Init.PeriphInc = DMA_PINC_DISABLE;
		    hdma_usart3_rx.Init.MemInc = DMA_MINC_ENABLE;
		    hdma_usart3_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
		    hdma_usart3_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
		    hdma_usart3_rx.Init.Mode = DMA_CIRCULAR;
		    hdma_usart3_rx.Init.Priority = DMA_PRIORITY_LOW;

		    UARTDMAHandler[(GetPort(huart)-1)] = &hdma_usart3_rx;

		    HAL_DMA_Init(&hdma_usart3_rx);

		    __HAL_LINKDMA(huart,hdmarx,hdma_usart3_rx);

		    /* USART3 interrupt Init */
		    HAL_NVIC_SetPriority(USART3_4_5_6_LPUART1_IRQn, 0, 0);
		    HAL_NVIC_EnableIRQ(USART3_4_5_6_LPUART1_IRQn);

		    __HAL_DMA_DISABLE_IT(&hdma_usart3_rx , DMA_IT_HT);
#endif
	}
	else if(huart->Instance == USART4){
	#ifdef _Usart4
		/* USART4 clock enable */
		    __HAL_RCC_USART4_CLK_ENABLE();

		    __HAL_RCC_GPIOA_CLK_ENABLE();
		    /**USART4 GPIO Configuration
		    PA0     ------> USART4_TX
		    PA1     ------> USART4_RX
		    */
		    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
		    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		    GPIO_InitStruct.Pull = GPIO_NOPULL;
		    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		    GPIO_InitStruct.Alternate = GPIO_AF4_USART4;
		    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

		    /* USART4 DMA Init */
		    /* USART4_RX Init */
		    hdma_usart4_rx.Instance = DMA1_Channel4;
		    hdma_usart4_rx.Init.Request = DMA_REQUEST_USART4_RX;
		    hdma_usart4_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
		    hdma_usart4_rx.Init.PeriphInc = DMA_PINC_DISABLE;
		    hdma_usart4_rx.Init.MemInc = DMA_MINC_ENABLE;
		    hdma_usart4_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
		    hdma_usart4_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
		    hdma_usart4_rx.Init.Mode = DMA_CIRCULAR;
		    hdma_usart4_rx.Init.Priority = DMA_PRIORITY_LOW;

		    UARTDMAHandler[(GetPort(huart)-1)] = &hdma_usart4_rx;


		    HAL_DMA_Init(&hdma_usart4_rx);
		
		    __HAL_LINKDMA(huart,hdmarx,hdma_usart4_rx);

		    /* USART4 interrupt Init */
		    HAL_NVIC_SetPriority(USART3_4_5_6_LPUART1_IRQn, 0, 0);
		    HAL_NVIC_EnableIRQ(USART3_4_5_6_LPUART1_IRQn);

		    __HAL_DMA_DISABLE_IT(&hdma_usart4_rx , DMA_IT_HT);
#endif
	}
	else if(huart->Instance == USART5){
	#ifdef _Usart5
		/* Peripheral clock enable */
	    __HAL_RCC_USART5_CLK_ENABLE();

	    __HAL_RCC_GPIOB_CLK_ENABLE();
		
		/* USART5 GPIO Configuration */
	    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
	    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	    GPIO_InitStruct.Pull = GPIO_NOPULL;
	    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	    GPIO_InitStruct.Alternate = GPIO_AF8_USART5;
	    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
		
	    /* USART5 DMA Init */
	    /* USART5_RX Init */
	    hdma_usart5_rx.Instance = DMA1_Channel5;
	    hdma_usart5_rx.Init.Request = DMA_REQUEST_USART5_RX;
	    hdma_usart5_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
	    hdma_usart5_rx.Init.PeriphInc = DMA_PINC_DISABLE;
	    hdma_usart5_rx.Init.MemInc = DMA_MINC_ENABLE;
	    hdma_usart5_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
	    hdma_usart5_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
	    hdma_usart5_rx.Init.Mode = DMA_CIRCULAR;
	    hdma_usart5_rx.Init.Priority = DMA_PRIORITY_LOW;
	    HAL_DMA_Init(&hdma_usart5_rx);

	    __HAL_LINKDMA(huart,hdmarx,hdma_usart5_rx);

	    __HAL_DMA_DISABLE_IT(&hdma_usart5_rx , DMA_IT_HT);
		/* Peripheral interrupt init*/
//		HAL_NVIC_SetPriority(USART3_4_5_6_LPUART1_IRQn,1,0);
//		HAL_NVIC_EnableIRQ(USART3_4_5_6_LPUART1_IRQn);
		//TOBECHECKED
#endif
	}
	else if(huart->Instance == USART6){
	#ifdef _Usart6
		/* USART6 clock enable */
		    __HAL_RCC_USART6_CLK_ENABLE();

		    __HAL_RCC_GPIOB_CLK_ENABLE();
		    /**USART6 GPIO Configuration
		    PB8     ------> USART6_TX
		    PB9     ------> USART6_RX
		    */
		    GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
		    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		    GPIO_InitStruct.Pull = GPIO_NOPULL;
		    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		    GPIO_InitStruct.Alternate = GPIO_AF8_USART6;
		    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

		    /* USART6 DMA Init */
		    /* USART6_RX Init */
		    hdma_usart6_rx.Instance = DMA1_Channel6;
		    hdma_usart6_rx.Init.Request = DMA_REQUEST_USART6_RX;
		    hdma_usart6_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
		    hdma_usart6_rx.Init.PeriphInc = DMA_PINC_DISABLE;
		    hdma_usart6_rx.Init.MemInc = DMA_MINC_ENABLE;
		    hdma_usart6_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
		    hdma_usart6_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
		    hdma_usart6_rx.Init.Mode = DMA_CIRCULAR;
		    hdma_usart6_rx.Init.Priority = DMA_PRIORITY_LOW;

		    UARTDMAHandler[(GetPort(huart)-1)] = &hdma_usart6_rx;

		    HAL_DMA_Init(&hdma_usart6_rx);
		
		    __HAL_LINKDMA(huart,hdmarx,hdma_usart6_rx);

		    /* USART6 interrupt Init */
		    HAL_NVIC_SetPriority(USART3_4_5_6_LPUART1_IRQn, 0, 0);
		    HAL_NVIC_EnableIRQ(USART3_4_5_6_LPUART1_IRQn);

		    __HAL_DMA_DISABLE_IT(&hdma_usart6_rx , DMA_IT_HT);
#endif
	}
}

/* --- Blocking (polling-based) read protected with a semaphore --- 
 */
HAL_StatusTypeDef readPxMutex(uint8_t port,char *buffer,uint16_t n,uint32_t mutexTimeout,uint32_t portTimeout){
	HAL_StatusTypeDef result =HAL_ERROR;
	
	if(GetUart(port) != NULL){
		/* Wait for the semaphore to be available. */
		if(osSemaphoreWait(PxRxSemaphoreHandle[port],mutexTimeout) == osOK){
			while(result != HAL_OK && result != HAL_TIMEOUT){
				result =HAL_UART_Receive(GetUart(port),(uint8_t* )buffer,n,portTimeout);
			}
			/* Give back the semaphore. */
			osSemaphoreRelease(PxRxSemaphoreHandle[port]);
		}
	}
	
	return result;
}

/* --- Blocking (polling-based) write protected with a semaphore --- 
 */
HAL_StatusTypeDef writePxMutex(uint8_t port,char *buffer,uint16_t n,uint32_t mutexTimeout,uint32_t portTimeout){
	HAL_StatusTypeDef result =HAL_ERROR;
	
	if(GetUart(port) != NULL){
		/*/ Wait for the semaphore to be available. */
		if(osSemaphoreWait(PxTxSemaphoreHandle[port],mutexTimeout) == osOK){
			while(result != HAL_OK && result != HAL_TIMEOUT){
				result =HAL_UART_Transmit(GetUart(port),(uint8_t* )buffer,n,portTimeout);
			}
			/* Give back the semaphore. */
			osSemaphoreRelease(PxTxSemaphoreHandle[port]);
		}
	}
	
	return result;
}

/* --- Non-blocking (interrupt-based) read protected with a semaphore --- 
 */
HAL_StatusTypeDef readPxITMutex(uint8_t port,char *buffer,uint16_t n,uint32_t mutexTimeout){
	HAL_StatusTypeDef result =HAL_ERROR;
	
	if(GetUart(port) != NULL){
		/* Wait for the mutex to be available. */
		if(osSemaphoreWait(PxRxSemaphoreHandle[port],mutexTimeout) == osOK){
			result =HAL_UART_Receive_IT(GetUart(port),(uint8_t* )buffer,n);
		}
	}
	
	return result;
}

/* --- Non-blocking (interrupt-based) write protected with a semaphore --- 
 */
HAL_StatusTypeDef writePxITMutex(uint8_t port,char *buffer,uint16_t n,uint32_t mutexTimeout){
	HAL_StatusTypeDef result =HAL_ERROR;
	
	if(GetUart(port) != NULL){
		/* Wait for the mutex to be available. */
		if(osSemaphoreWait(PxTxSemaphoreHandle[port],mutexTimeout) == osOK){
			result =HAL_UART_Transmit_IT(GetUart(port),(uint8_t* )buffer,n);
		}
	}
	
	return result;
}

///* --- Non-blocking (DMA-based) write protected with a semaphore ---
// */
//HAL_StatusTypeDef writePxDMAMutex(uint8_t port,char *buffer,uint16_t n,uint32_t mutexTimeout){
//	HAL_StatusTypeDef result =HAL_ERROR;
//	UART_HandleTypeDef *hUart =GetUart(port);
//
//	if(hUart != NULL){
//		/* Wait for the mutex to be available. */
//		if(osSemaphoreWait(PxTxSemaphoreHandle[port],mutexTimeout) == osOK){
//			/* Setup TX DMA on this port */
//			DMA_MSG_TX_Setup(hUart);
//			/* Transmit the message */
//			result =HAL_UART_Transmit_DMA(hUart,(uint8_t* )buffer,n);
//		}
//	}
//
//	return result;
//}

/* --- Update baudrate for this port --- 
 */
BOS_Status UpdateBaudrate(uint8_t port,uint32_t baudrate){
	BOS_Status result =BOS_OK;
	UART_HandleTypeDef *huart =GetUart(port);
	
	huart->Init.BaudRate =baudrate;
	HAL_UART_Init(huart);
	
	return result;
}

/*-----------------------------------------------------------*/

/* --- Get the UART for a given port. 
 */
UART_HandleTypeDef* GetUart(uint8_t port){
	switch(port){
#ifdef _P1
		case P1:
			return P1uart;
#endif
#ifdef _P2
		case P2:
			return P2uart;
#endif
#ifdef _P3
		case P3:
			return P3uart;
#endif
#ifdef _P4
		case P4:
			return P4uart;
#endif
#ifdef _P5
		case P5:
			return P5uart;
#endif
#ifdef _P6
		case P6:
			return P6uart;
#endif
#ifdef _P7
		case P7 :
			return P7uart;
	#endif
#ifdef _P8
		case P8 :
			return P8uart;
	#endif
#ifdef _P9
		case P9 :
			return P9uart;
	#endif
#ifdef _P10
		case P10 :
			return P10uart;
	#endif
		default:
			return 0;
	}
}

/*-----------------------------------------------------------*/

/* --- Swap UART pins ( NORMAL | REVERSED )--- 
 */
void SwapUartPins(UART_HandleTypeDef *huart,uint8_t direction){
	if(huart != NULL){
		if(direction == REVERSED){
			arrayPortsDir[myID - 1] |=(0x8000 >> (GetPort(huart) - 1)); /* Set bit to one */
			huart->AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_SWAP_INIT;
			huart->AdvancedInit.Swap = UART_ADVFEATURE_SWAP_ENABLE;
			HAL_UART_Init(huart);
			HAL_UARTEx_ReceiveToIdle_DMA(huart,(uint8_t* )&UARTRxBuf[GetPort(huart) - 1],MSG_RX_BUF_SIZE);
		}
		else if(direction == NORMAL){
			arrayPortsDir[myID - 1] &=(~(0x8000 >> (GetPort(huart) - 1))); /* Set bit to zero */
			huart->AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_SWAP_INIT;
			huart->AdvancedInit.Swap = UART_ADVFEATURE_SWAP_DISABLE;
			HAL_UART_Init(huart);
			HAL_UARTEx_ReceiveToIdle_DMA(huart,(uint8_t* )&UARTRxBuf[GetPort(huart) - 1],MSG_RX_BUF_SIZE);
		}
	}
}
/* --- Read Ports directions when a pre-defined topology file is used ---
 */
BOS_Status ReadPortsDir(void) {
	BOS_Status result = BOS_OK;
	/* Ask all other modules for their ports directions */
	for (uint8_t i = 1; i <= N; i++) {
		if (i != myID) {
			SendMessageToModule(i, CODE_READ_PORT_DIR, 0);
			Delay_ms_no_rtos(50);
			if (responseStatus != BOS_OK) {
				result = BOS_ERR_NoResponse;
			}
		} else {
			/* Check my own ports */
			for (uint8_t p = 1; p <= NumOfPorts; p++) {
				arrayPortsDir[myID - 1] |= (0x0000); /* Set bit to 1 */
			}
		}
	}

	return result;
}

/* --- Read Ports directions when a pre-defined topology file is used ---
 */
BOS_Status ReadPortsDirMSG(uint8_t SourceModule) {
	BOS_Status result = BOS_OK;
	uint16_t temp =0;
	/* Check my own ports */
	for (int p = 1; p <= NumOfPorts; p++) {
		if (GetUart(p)->AdvancedInit.Swap== UART_ADVFEATURE_SWAP_ENABLE) {
			messageParams[temp++] = p;
		}
	}
	/* Send response */
	SendMessageToModule(SourceModule, CODE_READ_PORT_DIR_RESPONSE, temp);
	return result;
}
/*-----------------------------------------------------------*/
#ifndef __N
/* --- Update module port directions based on what is stored in eeprom ---
*/
BOS_Status UpdateMyPortsDir(void)
{
	BOS_Status result = BOS_OK;

	/* Check port direction */
	for (uint8_t p=1 ; p<=NumOfPorts ; p++)
	{
		if ( !(arrayPortsDir[myID-1] & (0x8000>>(p-1))) ) {
			/* Port is normal */
			SwapUartPins(GetUart(p), NORMAL);
		} else {
			/* Port is reversed */
			SwapUartPins(GetUart(p), REVERSED);
		}
	}

	return result;
}
#endif

/*-----------------------------------------------------------*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
