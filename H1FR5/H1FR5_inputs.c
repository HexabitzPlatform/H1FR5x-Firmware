/*
 BitzOS (BOS) V0.4.0 - Copyright (C) 2017-2025 Hexabitz
 All rights reserved

 File Name     : H1FR5_inputs.c
 Description   : Source code for Bitz digital and analog inputs.

 */

/* Includes ****************************************************************/
#include "H1FR5_inputs.h"

/* Exported Variables ******************************************************/
bool DelayButtonStateReset = false;
bool NeedToDelayButtonStateReset = false;

/* Private variables *******************************************************/
/* Buttons */
uint8_t dblCounter[NUM_OF_PORTS + 1] ={0};
uint32_t PressCounter[NUM_OF_PORTS + 1] ={0};
uint32_t ReleaseCounter[NUM_OF_PORTS + 1] ={0};

Button_t Button[NUM_OF_PORTS + 1] ={0};

/* ADC */
uint8_t adcSelectFlag[2] ={0};
uint8_t adcEnableFlag =0;
uint8_t adcChannelRank =0;
uint16_t Channel =0;
uint16_t adcChannelValue[4] ={0};
uint16_t adcValueTemp =0;
uint16_t adcValueVref =0;
float Percentage =0.0f;
float Current =0.0f;
uint8_t adcDeInitFlag;

ADC_HandleTypeDef hadc;
ADC_ChannelConfTypeDef sConfig ={0};

BOS_Status AddPortButton(ButtonType_e buttonType,uint8_t port);
BOS_Status SetButtonEvents(uint8_t port,ButtonState_e buttonState,uint8_t mode);

/* Private buttons function prototypes *************************************/
BOS_Status GetPortGPIOs(uint8_t port,uint32_t *TX_Port,uint16_t *TX_Pin,uint32_t *RX_Port,uint16_t *RX_Pin);
void buttonPressedCallback(uint8_t port);
void buttonReleasedCallback(uint8_t port);
void buttonClickedCallback(uint8_t port);
void buttonDblClickedCallback(uint8_t port);

/* Private ADC function prototypes *****************************************/
void MX_ADC_Init(void);
void Error_Handler(void);
uint8_t GetRank(uint8_t Port,char *side);
uint32_t GetChannel(UART_HandleTypeDef *huart,char *side);

/***************************************************************************/
/* Private Functions *******************************************************/
/***************************************************************************/
/* Configure the global features of the ADC (Clock, Resolution,
 * Data Alignment and number of conversion) to read multiple ADC
 * channel in Port 2 and port 3 and for calculate internal temperature and internal voltage
 */
void MX_ADC_Init(void){
	hadc.Instance = ADC1;
	hadc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
	hadc.Init.Resolution = ADC_RESOLUTION_12B;
	hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
	hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	hadc.Init.LowPowerAutoWait =DISABLE;
	hadc.Init.LowPowerAutoPowerOff =DISABLE;
	hadc.Init.ContinuousConvMode =ENABLE;
	hadc.Init.DiscontinuousConvMode =DISABLE;
	hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc.Init.DMAContinuousRequests =DISABLE;
	hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;

	if(HAL_ADC_Init(&hadc) != HAL_OK){
		Error_Handler();
	}
	adcEnableFlag =1;
}

/***************************************************************************/
void HAL_ADC_MspInit(ADC_HandleTypeDef *adcHandle){
	GPIO_InitTypeDef GPIO_InitStruct ={0};
	/* ADC1 clock enable */
	__HAL_RCC_ADC_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();

	if(adcSelectFlag[0] == 1){
		GPIO_InitStruct.Pin = ADC_CH1_PIN | ADC_CH2_PIN;
		GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(ADC12_GPIO_PORT,&GPIO_InitStruct);
	}
	else{
		GPIO_InitStruct.Pin = ADC_CH3_PIN | ADC_CH4_PIN;
		GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(ADC34_GPIO_PORT,&GPIO_InitStruct);
	}
}

/***************************************************************************/
void HAL_ADC_MspDeInit(ADC_HandleTypeDef *adcHandle){

	if(adcHandle->Instance == ADC1){

		/* Peripheral clock disable */
		__HAL_RCC_ADC_CLK_DISABLE();
		HAL_GPIO_DeInit(ADC12_GPIO_PORT,ADC_CH1_PIN);
		HAL_GPIO_DeInit(ADC12_GPIO_PORT,ADC_CH2_PIN);
		HAL_GPIO_DeInit(ADC34_GPIO_PORT,ADC_CH3_PIN);
		HAL_GPIO_DeInit(ADC34_GPIO_PORT,ADC_CH4_PIN);
	}
}
/***************************************************************************/
void Error_Handler(void){

	HAL_Delay(100);
}

/***************************************************************************/
/*  Get the ADC_channel Number for a given UART */
uint32_t GetChannel(UART_HandleTypeDef *huart,char *side){

	if(huart->Instance == ADC_CH1_USART && !strcmp(side,"top"))
		return ADC_CH1_CHANNEL;
	else if(huart->Instance == ADC_CH2_USART && !strcmp(side,"bottom"))
		return ADC_CH2_CHANNEL;
	else if(huart->Instance == ADC_CH3_USART && !strcmp(side,"top"))
		return ADC_CH3_CHANNEL;
	else if(huart->Instance == ADC_CH3_USART && !strcmp(side,"bottom"))
		return ADC_CH4_CHANNEL;
}

/***************************************************************************/
uint8_t GetRank(uint8_t Port,char *side){

	if(Port == 2 && !strcmp(side,"top"))
		adcChannelRank =0;
	else if(Port == 2 && !strcmp(side,"bottom"))
		adcChannelRank =1;
	else if(Port == 3 && !strcmp(side,"top"))
		adcChannelRank =2;
	else if(Port == 3 && !strcmp(side,"bottom"))
		adcChannelRank =3;
	return adcChannelRank;
}

/***************************************************************************/
/* BOS Exported Functions **************************************************/
/***************************************************************************/
void CheckAttachedButtons(void){
	uint32_t TX_Port, RX_Port;
	uint16_t TX_Pin, RX_Pin;
	uint8_t connected =GPIO_PIN_RESET, state =0;
	static uint8_t clicked;

	for(uint8_t i =1; i <= NUM_OF_PORTS; i++){
		if(Button[i].Type)			// Only check defined butons
		{
			/* 1. Reset button state */
			if(DelayButtonStateReset == false)
				Button[i].State =NONE;

			/* 2. Get button GPIOs */
			GetPortGPIOs(i,&TX_Port,&TX_Pin,&RX_Port,&RX_Pin);

			/* 3. Check if port pins are connected */
			HAL_GPIO_WritePin((GPIO_TypeDef* )TX_Port,TX_Pin,GPIO_PIN_RESET);
			Delay_us(10);
			if(HAL_GPIO_ReadPin((GPIO_TypeDef* )RX_Port,RX_Pin) == GPIO_PIN_RESET){
				HAL_GPIO_WritePin((GPIO_TypeDef* )TX_Port,TX_Pin,GPIO_PIN_SET);
				Delay_us(10);
				connected =HAL_GPIO_ReadPin((GPIO_TypeDef* )RX_Port,RX_Pin);
			}
			HAL_GPIO_WritePin((GPIO_TypeDef* )TX_Port,TX_Pin,GPIO_PIN_RESET);

			/* 4. Determine button state based on port reading and button type */
			switch(Button[i].Type){
				case MOMENTARY_NO:
					if(connected == GPIO_PIN_SET)
						state =CLOSED;
					else if(connected == GPIO_PIN_RESET)
						state =OPEN;
					break;

				case MOMENTARY_NC:
					if(connected == GPIO_PIN_SET)
						state =CLOSED;
					else if(connected == GPIO_PIN_RESET)
						state =OPEN;
					break;

				case ONOFF_NO:
					if(connected == GPIO_PIN_SET)
						state =ON;
					else if(connected == GPIO_PIN_RESET)
						state =OFF;
					break;

				case ONOFF_NC:
					if(connected == GPIO_PIN_SET)
						state =OFF;
					else if(connected == GPIO_PIN_RESET)
						state =ON;
					break;

				default:
					break;
			}

			/* 5. Debounce this state and update button struct if needed */
			/* 5.A. Possible change of state 1: OPEN > CLOSED or OFF >> ON */
			if(state == CLOSED || state == ON){
				if(PressCounter[i] < 0xFFFF)
					++PressCounter[i];			// Advance the debounce counter
				else
					PressCounter[i] =0;			// Reset debounce counter
			}

			/* 5.B. Possible change of state 2: CLOSED > OPEN or ON >> OFF */
			if(state == OPEN || state == OFF){
				if(ReleaseCounter[i] < 0xFFFF)
					++ReleaseCounter[i];		// Advance the debounce counter
				else
					ReleaseCounter[i] =0;		// Reset debounce counter

				if(clicked == 2 && dblCounter[i] <= BOS.Buttons.maxInterClickTime)		// Advance the inter-click counter
					++dblCounter[i];
				else if(dblCounter[i] > BOS.Buttons.maxInterClickTime){
					clicked =0;
					dblCounter[i] =0;			// Reset the inter-click counter
				}
			}

			/* Analyze state */
			/* 5.C. On press: Record a click if pressed less than 1 second */
			if(PressCounter[i] < BOS.Buttons.Debounce){
				// This is noise. Ignore it
			}
			else{
				if(PressCounter[i] == BOS.Buttons.Debounce){
//					Button[i].State = PRESSED;// Record a PRESSED event. This event is always reset on next tick.
					++PressCounter[i];
				}

				if(ReleaseCounter[i] > BOS.Buttons.Debounce)			// Reset ReleaseCounter if needed - to avoid masking PressCounter on NO switches
					ReleaseCounter[i] =0;

				if(PressCounter[i] > BOS.Buttons.SingleClickTime && PressCounter[i] < 500){
					if(clicked == 0)
						clicked =1;		// Record a possible single click
					else if(clicked == 2){
						if(dblCounter[i] > BOS.Buttons.minInterClickTime && dblCounter[i] < BOS.Buttons.maxInterClickTime){
							clicked =3;	// Record a possible double click
							dblCounter[i] =0;	// Reset the inter-click counter
						}
					}
				}
				else if(PressCounter[i] >= 500 && PressCounter[i] < 0xFFFF){
					if(clicked)
						clicked =0;						// Cannot be a click
				}
			}

			/* 5.D. On release: Record a click if pressed less than 1 second */
			if(ReleaseCounter[i] < BOS.Buttons.Debounce){
				// This is noise. Ignore it
			}
			else{
				if(ReleaseCounter[i] == BOS.Buttons.Debounce){
					Button[i].State =RELEASED;	// Record a RELEASED event. This event is always reset on next tick.
					++ReleaseCounter[i];
				}

				if(PressCounter[i] > BOS.Buttons.Debounce)	// Reset PressCounter if needed - to avoid masking ReleaseCounter on NC switches
					PressCounter[i] =0;

				if(ReleaseCounter[i] > BOS.Buttons.SingleClickTime && ReleaseCounter[i] < 500){
					if(clicked == 1){
						Button[i].State =CLICKED;	// Record a single button click event
						clicked =2;			// Prepare for a double click
					}
					else if(clicked == 3){
						Button[i].State =DBL_CLICKED;			// Record a double button click event
						clicked =0;			// Prepare for a single click
					}
				}
			}

			/* 6. Run button callbacks if needed */
			switch(Button[i].State){
				case RELEASED:
					buttonReleasedCallback(i);
					Button[i].State =NONE;
					break;

				case CLICKED:
					if(!DelayButtonStateReset && (Button[i].Event & BUTTON_EVENT_CLICKED)){
						DelayButtonStateReset = true;
						buttonClickedCallback(i);
					}
					break;

				case DBL_CLICKED:
					if(!DelayButtonStateReset && (Button[i].Event & BUTTON_EVENT_DBL_CLICKED)){
						DelayButtonStateReset = true;
						buttonDblClickedCallback(i);
					}
					break;

				default:
					break;
			}
		}
	}
}

/***************************************************************************/
/* Reset state of attached buttons to avoid recurring callbacks */
void ResetAttachedButtonStates(uint8_t *deferReset){
	if(!*deferReset){
		for(uint8_t i =1; i <= NUM_OF_PORTS; i++){
			if(Button[i].State != NONE)
				Button[i].State =NONE;
		}
	}
}

/***************************************************************************/
/* Define a new button attached to one of array ports
 * buttonType: MOMENTARY_NO, MOMENTARY_NC, ONOFF_NO, ONOFF_NC
 * port: array port (P1 - Px)
 */
BOS_Status AddPortButton(ButtonType_e buttonType,uint8_t port){
	BOS_Status result =BOS_OK;
	GPIO_InitTypeDef GPIO_InitStruct;
	uint32_t TX_Port, RX_Port;
	uint16_t TX_Pin, RX_Pin, temp16, res;
	uint8_t temp8 =0;

	/* 1. Stop communication at this port (only if the scheduler is running) - TODO update*/
	if(bosInitialized){
		osSemaphoreRelease(PxRxSemaphoreHandle[port]); /* Give back the semaphore if it was taken */
		osSemaphoreRelease(PxTxSemaphoreHandle[port]);
	}
	PortStatus[port] =PORTBUTTON;

	/* 2. Deinitialize UART (only if module is initialized) */
	if(bosInitialized){
		HAL_UART_DeInit(GetUart(port));
	}

	/* 3. Initialize GPIOs */
	GetPortGPIOs(port,&TX_Port,&TX_Pin,&RX_Port,&RX_Pin);
	/* Ouput (TXD) */
	GPIO_InitStruct.Pin =TX_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init((GPIO_TypeDef* )TX_Port,&GPIO_InitStruct);
	/* Input (RXD) */
	GPIO_InitStruct.Pin =RX_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init((GPIO_TypeDef* )RX_Port,&GPIO_InitStruct);

	/* 4. Update button struct */
	Button[port].Type =buttonType;

	/* 5. Add to EEPROM if not already there */
	res =EE_ReadVariable(_EE_BUTTON_BASE + 4 * (port - 1),&temp16);
	if(!res)											// This variable exists
	{
		temp8 =(uint8_t )(temp16 >> 8);
		if(((temp8 >> 4) == port) && ((temp8 & 0x0F) == buttonType))											// This is same port and same type, do not update
			return BOS_OK;
		else 											// Update the variable
		{
			temp16 =((uint16_t )port << 12) | ((uint16_t )buttonType << 8);
			EE_WriteVariable(_EE_BUTTON_BASE + 4 * (port - 1),temp16);
			/* Reset times */
			EE_WriteVariable(_EE_BUTTON_BASE + 4 * (port - 1) + 1,0);
			EE_WriteVariable(_EE_BUTTON_BASE + 4 * (port - 1) + 2,0);
			EE_WriteVariable(_EE_BUTTON_BASE + 4 * (port - 1) + 3,0);
		}
	}
	else							// Variable does not exist. Create a new one
	{
		temp16 =((uint16_t )port << 12) | ((uint16_t )buttonType << 8);
		EE_WriteVariable(_EE_BUTTON_BASE + 4 * (port - 1),temp16);
		/* Reset times */
		EE_WriteVariable(_EE_BUTTON_BASE + 4 * (port - 1) + 1,0);
		EE_WriteVariable(_EE_BUTTON_BASE + 4 * (port - 1) + 2,0);
		EE_WriteVariable(_EE_BUTTON_BASE + 4 * (port - 1) + 3,0);
	}

	return result;
}

/***************************************************************************/
/* Undefine a button attached to one of array ports and restore the port to default state
 *  port: array port (P1 - Px)
 */
BOS_Status RemovePortButton(uint8_t port){
	BOS_Status result =BOS_OK;
	uint16_t res, temp16;

	/* 1. Remove from button struct */
	Button[port].Type =NONE;
	Button[port].State =NONE;
	Button[port].Event =0;

	/* 2. Remove from EEPROM if it's already there */
	res =EE_ReadVariable(_EE_BUTTON_BASE + 4 * (port - 1),&temp16);
	if(!res)						// This variable exists, reset all to zeros
	{
		EE_WriteVariable(_EE_BUTTON_BASE + 4 * (port - 1),0);
		/* Reset times */
		EE_WriteVariable(_EE_BUTTON_BASE + 4 * (port - 1) + 1,0);
		EE_WriteVariable(_EE_BUTTON_BASE + 4 * (port - 1) + 2,0);
		EE_WriteVariable(_EE_BUTTON_BASE + 4 * (port - 1) + 3,0);
	}

	/* 3. Initialize UART at this port */
	UART_HandleTypeDef *huart =GetUart(port);

	if(huart->Instance == USART1){
#ifdef _USART1
		MX_USART1_UART_Init();
#endif
	}
	else if(huart->Instance == USART2){
#ifdef _USART2
		MX_USART2_UART_Init();
#endif
	}
	else if(huart->Instance == USART3){
#ifdef _USART3
		MX_USART3_UART_Init();
#endif
	}
	else if(huart->Instance == USART4){
#ifdef _USART4
		MX_USART4_UART_Init();
#endif
	}
	else if(huart->Instance == USART5){
#ifdef _USART5
		MX_USART5_UART_Init();
#endif
	}
	else if(huart->Instance == USART6){
#ifdef _USART6
		MX_USART6_UART_Init();
#endif
	}
	else
		result =BOS_ERROR;

	/* 4. free port */
	PortStatus[port] =FREE;
	/* Setup UART DMA */
	DMA_MSG_RX_Setup(huart,UARTDMAHandler[port - 1]);

	return result;
}

/***************************************************************************/
BOS_Status AddButton(uint8_t port,ButtonType_e buttonType,ButtonState_e buttonState){
	BOS_Status Status =BOS_OK;

	AddPortButton(buttonType,port);
	SetButtonEvents(port,buttonState,0);

	return BOS_OK;
}

/***************************************************************************/
/* Setup button events and callback:
 * port: array port (P1 - Px) where the button is attached.
 * buttonState: OFF, ON, OPEN, CLOSED, CLICKED, DBL_CLICKED, RELEASED.
 * mode: BUTTON_EVENT_MODE_CLEAR to clear events marked with 0,
 * BUTTON_EVENT_MODE_OR to OR events marked with 1 with existing events.
 */
BOS_Status SetButtonEvents(uint8_t port,ButtonState_e buttonState,uint8_t mode){
	BOS_Status result =BOS_OK;
	uint16_t res, temp16;
	uint8_t temp8;

	if(Button[port].Type == NONE)
		return BOS_ERR_BUTTON_NOT_DEFINED;

	if(mode == BUTTON_EVENT_MODE_OR || (mode == BUTTON_EVENT_MODE_CLEAR && buttonState == CLICKED))
		Button[port].Event |= BUTTON_EVENT_CLICKED;
	else if(mode == BUTTON_EVENT_MODE_CLEAR && !buttonState == CLICKED)
		Button[port].Event &=~BUTTON_EVENT_CLICKED;

	if(mode == BUTTON_EVENT_MODE_OR || (mode == BUTTON_EVENT_MODE_CLEAR && buttonState == DBL_CLICKED))
		Button[port].Event |= BUTTON_EVENT_DBL_CLICKED;
	else if(mode == BUTTON_EVENT_MODE_CLEAR && !buttonState == DBL_CLICKED)
		Button[port].Event &=~BUTTON_EVENT_DBL_CLICKED;

	/* Add to EEPROM */
	res =EE_ReadVariable(_EE_BUTTON_BASE + 4 * (port - 1),&temp16);
	if(!res)											// This variable exists
	{
		temp8 =(uint8_t )(temp16 >> 8);					// Keep upper byte
		/* Store event flags */
		if((uint8_t )(temp16) != Button[port].Event){					// Update only if different
			temp16 =((uint16_t )temp8 << 8) | (uint16_t )Button[port].Event;
			EE_WriteVariable(_EE_BUTTON_BASE + 4 * (port - 1),temp16);
		}
	}	// TODO - var does not exist after adding button!
	else
		// Variable does not exist. Return error
		return BOS_ERR_BUTTON_NOT_DEFINED;

	return result;
}

/***************************************************************************/
/* Exported Functions ******************************************************/
/***************************************************************************/
/* select port 2 & port 3 for the selected ADC regular channel to be converted */
BOS_Status ADCSelectPort(uint8_t ADC_port){
	BOS_Status Status =BOS_OK;

	if(ADC_port == ADC12_PORT || ADC_port == ADC34_PORT){
		if(ADC_port == ADC12_PORT)
			adcSelectFlag[0] =1;
		else
			adcSelectFlag[1] =1;

		HAL_UART_DeInit(GetUart(ADC_port));
		PortStatus[ADC_port] =CUSTOM;
		if(adcEnableFlag == 0)
		{
			MX_ADC_Init();
			adcDeInitFlag = 0;
		}

	}
	else
		return Status =BOS_ERR_ADC_WRONG_PORT;

	return Status;
}

/***************************************************************************/
BOS_Status ReadADCChannel(uint8_t Port,char *side,float *ADC_Value){
	BOS_Status Status =BOS_OK;

	if(Port == ADC12_PORT || Port == ADC34_PORT){
		if(adcEnableFlag == 1){

			/* Enable chosen channel to be read */
			Channel =GetChannel(GetUart(Port),side);
			adcChannelRank =GetRank(Port,side);

			sConfig.Channel =Channel;
			sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
			sConfig.SamplingTime = ADC_SAMPLETIME_79CYCLES_5;
			HAL_ADC_ConfigChannel(&hadc,&sConfig);

			HAL_ADC_Start(&hadc);
			HAL_ADC_PollForConversion(&hadc,100);
			adcChannelValue[adcChannelRank] =HAL_ADC_GetValue(&hadc);
			HAL_ADC_Stop(&hadc);

			/* Disable chosen channel */
			sConfig.Channel =Channel;
			sConfig.Rank = ADC_RANK_NONE;
			sConfig.SamplingTime = ADC_SAMPLETIME_79CYCLES_5;
			HAL_ADC_ConfigChannel(&hadc,&sConfig);

		}
		*ADC_Value =(float )(adcChannelValue[adcChannelRank] * 3.3 / 4095);
	}
	else
		return BOS_ERR_ADC_WRONG_PORT;

	return Status;

}

/***************************************************************************/
void ReadTempAndVref(float *temp,float *Vref){

	if(0 == adcEnableFlag)
	{
		MX_ADC_Init();
		adcDeInitFlag = 0;
	}


	/* Enable internal temperature channel */
	sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
	sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
	sConfig.SamplingTime = ADC_SAMPLETIME_7CYCLES_5;
	HAL_ADC_ConfigChannel(&hadc,&sConfig);

	HAL_ADC_Start(&hadc);
	HAL_ADC_PollForConversion(&hadc,100);
	adcValueTemp =HAL_ADC_GetValue(&hadc);
	*temp =((3.3 * adcValueTemp / 4095 - V25) / AVG_SLOPE) + 25;
	HAL_ADC_Stop(&hadc);

	/* Disable internal temperature channel */
	sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
	sConfig.Rank = ADC_RANK_NONE;
	sConfig.SamplingTime = ADC_SAMPLETIME_7CYCLES_5;
	HAL_ADC_ConfigChannel(&hadc,&sConfig);

	/* Enable internal Voltage Reference channel */
	sConfig.Channel = ADC_CHANNEL_VREFINT;
	sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
	sConfig.SamplingTime = ADC_SAMPLETIME_79CYCLES_5;
	HAL_ADC_ConfigChannel(&hadc,&sConfig);

	HAL_ADC_Start(&hadc);
	HAL_ADC_PollForConversion(&hadc,100);
	adcValueVref =HAL_ADC_GetValue(&hadc);
	*Vref =3.3 * (*VREF_CAL) / adcValueVref;
	HAL_ADC_Stop(&hadc);

	/* Disable internal Voltage Reference channel */
	sConfig.Channel = ADC_CHANNEL_VREFINT;
	sConfig.Rank = ADC_RANK_NONE;
	sConfig.SamplingTime = ADC_SAMPLETIME_7CYCLES_5;
	HAL_ADC_ConfigChannel(&hadc,&sConfig);

}

/***************************************************************************/
BOS_Status GetReadPercentage(uint8_t port,char *side,float *precentageValue){
	BOS_Status Status =BOS_OK;
	float ADC_Value =0.0f;

	if(BOS_OK == ReadADCChannel(port,side,&ADC_Value))
		*precentageValue =(ADC_Value * 100) / 3.3;
	else
		return BOS_ERR_ADC_WRONG_PORT;

	return Status;
}

/***************************************************************************/

BOS_Status ADCDeinitChannel(uint8_t port){
	BOS_Status Status =BOS_OK;

	if(port == ADC12_PORT || port == ADC34_PORT){
		if(adcDeInitFlag == 0)
		{
			HAL_ADC_DeInit(&hadc);
			adcDeInitFlag = 1;
		}

		UART_HandleTypeDef* huart = GetUart(port);
		HAL_UART_Init(huart);
		PortStatus[port] =FREE;
		DMA_MSG_RX_Setup(huart,UARTDMAHandler[port - 1]);
		adcEnableFlag =0;
	}
	else
		return BOS_ERR_ADC_WRONG_PORT;

	return Status;
}

/***************************************************************************/
/* User Button Functions ***************************************************/
/***************************************************************************/
/* Button press callback. DO NOT MODIFY THIS CALLBACK.
 * This function is declared as __weak to be overwritten by other implementations in user file.
 */
__weak void buttonPressedCallback(uint8_t port){
}

/***************************************************************************/
/* Button release callback. DO NOT MODIFY THIS CALLBACK.
 * This function is declared as __weak to be overwritten by other implementations in user file.
 */
__weak void buttonReleasedCallback(uint8_t port){
}

/***************************************************************************/
/* Button single click callback. DO NOT MODIFY THIS CALLBACK.
 * This function is declared as __weak to be overwritten by other implementations in user file.
 */
__weak void buttonClickedCallback(uint8_t port){
}

/***************************************************************************/
/* Button double click callback. DO NOT MODIFY THIS CALLBACK.
 * This function is declared as __weak to be overwritten by other implementations in user file.
 */
__weak void buttonDblClickedCallback(uint8_t port){
}

/***************************************************************************/
/***************** (C) COPYRIGHT HEXABITZ ***** END OF FILE ****************/
