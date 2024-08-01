/*
 BitzOS (BOS) V0.3.6 - Copyright (C) 2017-2024 Hexabitz
 All rights reserved

 File Name     : H1FR5_gpio.c
 Description   : Source code provides code for the configuration of all used GPIO pins .

 */

/* Includes ------------------------------------------------------------------*/
#include "BOS.h"

/*  */
BOS_Status GetPortGPIOs(uint8_t port,uint32_t *TX_Port,uint16_t *TX_Pin,uint32_t *RX_Port,uint16_t *RX_Pin);

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/

/** Pinout Configuration
 */
void GPIO_Init(void){
	/* GPIO Ports Clock Enable */
	__GPIOC_CLK_ENABLE();
	__GPIOA_CLK_ENABLE();
	__GPIOD_CLK_ENABLE();
	__GPIOB_CLK_ENABLE();
	__GPIOF_CLK_ENABLE();		// for HSE and Boot0
	
	IND_LED_Init();
}

//-- Configure indicator LED
void IND_LED_Init(void){
	GPIO_InitTypeDef GPIO_InitStruct;
	
	GPIO_InitStruct.Pin = _IND_LED_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(_IND_LED_PORT,&GPIO_InitStruct);
}

/*-----------------------------------------------------------*/

/* --- Check for factory reset condition: 
 - P1 TXD is connected to last port RXD    
 */
uint8_t IsFactoryReset(void){
	GPIO_InitTypeDef GPIO_InitStruct;
	uint32_t P1_TX_Port, P1_RX_Port, P_last_TX_Port, P_last_RX_Port;
	uint16_t P1_TX_Pin, P1_RX_Pin, P_last_TX_Pin, P_last_RX_Pin;
	
	/* -- Setup GPIOs -- */

	/* Enable all GPIO Ports Clocks */
	__GPIOA_CLK_ENABLE();
	__GPIOB_CLK_ENABLE();
	__GPIOC_CLK_ENABLE();
	__GPIOD_CLK_ENABLE();
	
	/* Get GPIOs */
	GetPortGPIOs(P1,&P1_TX_Port,&P1_TX_Pin,&P1_RX_Port,&P1_RX_Pin);
	GetPortGPIOs(P_LAST,&P_last_TX_Port,&P_last_TX_Pin,&P_last_RX_Port,&P_last_RX_Pin);
	
	/* TXD of first port */
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Pin =P1_TX_Pin;
	HAL_GPIO_Init((GPIO_TypeDef* )P1_TX_Port,&GPIO_InitStruct);
	
	/* RXD of last port */
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	GPIO_InitStruct.Pin =P_last_RX_Pin;
	HAL_GPIO_Init((GPIO_TypeDef* )P_last_RX_Port,&GPIO_InitStruct);
	
	/* Check for factory reset conditions */
	HAL_GPIO_WritePin((GPIO_TypeDef* )P1_TX_Port,P1_TX_Pin,GPIO_PIN_RESET);
	Delay_ms_no_rtos(5);
	if(HAL_GPIO_ReadPin((GPIO_TypeDef* )P_last_RX_Port,P_last_RX_Pin) == RESET){
		HAL_GPIO_WritePin((GPIO_TypeDef* )P1_TX_Port,P1_TX_Pin,GPIO_PIN_SET);
		Delay_ms_no_rtos(5);
		if(HAL_GPIO_ReadPin((GPIO_TypeDef* )P_last_RX_Port,P_last_RX_Pin) == SET){
			return 1;
		}
	}
	
	/* Clear flag for formated EEPROM if it was already set */
	/* Flag address (STM32F09x) - Last 4 words of SRAM */
	*((unsigned long* )0x20007FF0) =0xFFFFFFFF;
	
	return 0;
}

/*-----------------------------------------------------------*/

/* --- Get GPIO pins and ports of this array port
 */
BOS_Status GetPortGPIOs(uint8_t port,uint32_t *TX_Port,uint16_t *TX_Pin,uint32_t *RX_Port,uint16_t *RX_Pin){
	BOS_Status result =BOS_OK;
	
	/* Get port UART */
	UART_HandleTypeDef *huart =GetUart(port);
	
	if(huart == &huart1){
#ifdef _Usart1		
		*TX_Port =(uint32_t ) USART1_TX_PORT;
		*TX_Pin = USART1_TX_PIN;
		*RX_Port =(uint32_t ) USART1_RX_PORT;
		*RX_Pin = USART1_RX_PIN;
#endif
	}
#ifdef _Usart2	
	else if(huart == &huart2){
		*TX_Port =(uint32_t ) USART2_TX_PORT;
		*TX_Pin = USART2_TX_PIN;
		*RX_Port =(uint32_t ) USART2_RX_PORT;
		*RX_Pin = USART2_RX_PIN;
	}
#endif
#ifdef _Usart3	
	else if(huart == &huart3){
		*TX_Port =(uint32_t ) USART3_TX_PORT;
		*TX_Pin = USART3_TX_PIN;
		*RX_Port =(uint32_t ) USART3_RX_PORT;
		*RX_Pin = USART3_RX_PIN;
	}
#endif
#ifdef _Usart4
	else if(huart == &huart4){
		*TX_Port =(uint32_t ) USART4_TX_PORT;
		*TX_Pin = USART4_TX_PIN;
		*RX_Port =(uint32_t ) USART4_RX_PORT;
		*RX_Pin = USART4_RX_PIN;
	}
#endif
#ifdef _Usart5	
	else if(huart == &huart5){
		*TX_Port =(uint32_t ) USART5_TX_PORT;
		*TX_Pin = USART5_TX_PIN;
		*RX_Port =(uint32_t ) USART5_RX_PORT;
		*RX_Pin = USART5_RX_PIN;
	}
#endif
#ifdef _Usart6	
	else if(huart == &huart6){
		*TX_Port =(uint32_t ) USART6_TX_PORT;
		*TX_Pin = USART6_TX_PIN;
		*RX_Port =(uint32_t ) USART6_RX_PORT;
		*RX_Pin = USART6_RX_PIN;
	}
#endif
#ifdef _Usart7
	else if (huart == &huart7) 
	{		
		*TX_Port = (uint32_t)USART7_TX_PORT;
		*TX_Pin = USART7_TX_PIN;
		*RX_Port = (uint32_t)USART7_RX_PORT;
		*RX_Pin = USART7_RX_PIN;
	} 
#endif
#ifdef _Usart8	
	else if (huart == &huart8) 
	{	
		*TX_Port = (uint32_t)USART8_TX_PORT;
		*TX_Pin = USART8_TX_PIN;
		*RX_Port = (uint32_t)USART8_RX_PORT;
		*RX_Pin = USART8_RX_PIN;
	} 
#endif
	else
		result =BOS_ERROR;
	
	return result;
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
