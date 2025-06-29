/*
 BitzOS (BOS) V0.4.0 - Copyright (C) 2017-2025 Hexabitz
 All rights reserved

 File Name  : H1FR5_inputs.h
 Description: Declares functions for ADC channel management.
 ADC: Configure channels for analog input, temperature, voltage on ports 1, 2.
 GPIO: Define pin mappings for ports P1, P2.
 */

/* Includes ****************************************************************/
#include "BOS.h"

/* ADC Macro Definitions ***************************************************/
/* Port-ADC Definitions */
#define ADC_CH1_PIN   		GPIO_PIN_2
#define ADC_CH2_PIN   		GPIO_PIN_3
#define ADC_CH3_PIN   		GPIO_PIN_0
#define ADC_CH4_PIN   		GPIO_PIN_1
#define ADC12_PORT  		P2
#define ADC34_PORT			P1
#define ADC12_GPIO_PORT  	GPIOA
#define ADC34_GPIO_PORT		GPIOA
#define ADC_CH1_USART   	USART2
#define ADC_CH2_USART   	USART2
#define ADC_CH3_USART   	USART4
#define ADC_CH4_USART   	USART4
#define ADC_CH1_CHANNEL   	ADC_CHANNEL_2
#define ADC_CH2_CHANNEL   	ADC_CHANNEL_3
#define ADC_CH3_CHANNEL   	ADC_CHANNEL_0
#define ADC_CH4_CHANNEL   	ADC_CHANNEL_1

/* Constant Macros */
#define VREF_CAL            ((uint16_t *)((uint32_t)0x1FFF75AA))
#define AVG_SLOPE           4.3
#define V25                 1.41

/***************************************************************************/
/* Exported Functions Prototypes *******************************************/
/***************************************************************************/
extern void ReadTempAndVref(float *temp,float *Vref);
extern BOS_Status ReadADCChannel(uint8_t adcPort, ModuleLayer_t side,float *adcVoltage);
extern BOS_Status ADCSelectPort(uint8_t adcPort);
extern BOS_Status GetReadPercentage(uint8_t adcPort,ModuleLayer_t side,float *precentageadcVoltage);
extern BOS_Status ADCDeinitChannel(uint8_t adcPort);
/************************ (C) COPYRIGHT HEXABITZ *****END OF FILE****/
