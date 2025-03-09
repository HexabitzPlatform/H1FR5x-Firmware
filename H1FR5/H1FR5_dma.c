/*
 BitzOS (BOS) V0.3.6 - Copyright (C) 2017-2024 Hexabitz
 All rights reserved

 File Name     : H01R0_dma.c
 Description   : source file Contains Peripheral DMA setup .

 */

/* Includes ------------------------------------------------------------------*/
#include "BOS.h"

/* Variables ---------------------------------------------------------*/
DMA_HandleTypeDef *UARTDMAHandler[6];
CRC_HandleTypeDef hcrc;
/*-----------------------------------------------------------*/

/* Functions ---------------------------------------------------------*/

/*
 * @brief: DMA initialize.
 * @retval: Nothing.
 */

void DMA_Init(void){

  /* DMA controller clock enable */
  __DMA1_CLK_ENABLE();
  __DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);
  /* DMA1_Ch4_7_DMA2_Ch1_5_DMAMUX1_OVR_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Ch4_7_DMA2_Ch1_5_DMAMUX1_OVR_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Ch4_7_DMA2_Ch1_5_DMAMUX1_OVR_IRQn);

}

/****************************************************************************************/

/*
 * @brief: Setup and start Messaging DMAs.
 * @retval: BOS_Status.
 */

BOS_Status SetupMessagingRxDMAs(void){
	BOS_Status Status = BOS_OK;

#ifdef _P1
	if(portStatus[P1] == FREE){
		if(BOS_OK != DMA_MSG_RX_Setup(P1uart,UARTDMAHandler[0]))
			return Status = BOS_ERROR;}
#endif
#ifdef _P2
	if(portStatus[P2] == FREE){
		if(BOS_OK != DMA_MSG_RX_Setup(P2uart,UARTDMAHandler[1]))
			return Status = BOS_ERROR;}
#endif
#ifdef _P3	
	if(portStatus[P3] == FREE){
		if(BOS_OK != DMA_MSG_RX_Setup(P3uart,UARTDMAHandler[2]))
			return Status = BOS_ERROR;}
#endif
#ifdef _P4		
	if(portStatus[P4] == FREE){
		if(BOS_OK != DMA_MSG_RX_Setup(P4uart,UARTDMAHandler[3]))
			return Status = BOS_ERROR;}
#endif
#ifdef _P5		
	if(portStatus[P5] == FREE){
		if(BOS_OK != DMA_MSG_RX_Setup(P5uart,UARTDMAHandler[4]))
			return Status = BOS_ERROR;}
#endif
#ifdef _P6
	if(portStatus[P6] == FREE){
		if(BOS_OK != DMA_MSG_RX_Setup(P6uart,UARTDMAHandler[5]))
			return Status = BOS_ERROR;}
#endif

	return Status;
}

/****************************************************************************************/

/*
 * @brief: Messaging DMA RX setup.
 * @param1: UART handler.
 * @param2: DMA handler.
 * @retval: BOS_Status.
 */

BOS_Status DMA_MSG_RX_Setup(UART_HandleTypeDef *huart,DMA_HandleTypeDef *hDMA){
	BOS_Status Status = BOS_OK;

	if(HAL_OK != HAL_UARTEx_ReceiveToIdle_DMA(huart,(uint8_t* )&UARTRxBuf[GetPort(huart) - 1],MSG_RX_BUF_SIZE))
		return Status = BOS_ERROR;
	 __HAL_DMA_DISABLE_IT(hDMA , DMA_IT_HT);

	 return Status;
}

/****************************************************************************************/

/*
 * @brief: Streaming DMA setup.
 * param1: UART source handler.
 * param2: UART destination handler.
 * param3: data size.
 * @retval: BOS_Status.
 */

BOS_Status DMA_STREAM_Setup(UART_HandleTypeDef *huartSrc,UART_HandleTypeDef *huartDst,uint16_t num){
	BOS_Status Status = BOS_OK;
	DMA_HandleTypeDef *hDMA;
	uint8_t port,dstPort;

	port = GetPort(huartSrc);
	dstPort = GetPort(huartDst);
	hDMA = UARTDMAHandler[port - 1];
	/* dstPort = 0 this mean we will receive stream data on RAM memory incoming from a destination module */
	if(dstPort == 0){
		/* set DMA index corresponding to UART to zero, so that the DMA starts writing from the beginning of the specific buffer */
		index_process[GetPort(huartSrc) - 1] = 0;
		memset(streamBuffer,0,STREAM_BUF_SIZE);
		if(HAL_OK != HAL_UARTEx_ReceiveToIdle_DMA(huartSrc,streamBuffer,num))
			return Status = BOS_ERROR;
		__HAL_DMA_DISABLE_IT(hDMA , DMA_IT_HT);
	}
	else{
		if(HAL_OK != HAL_UARTEx_ReceiveToIdle_DMA(huartSrc,(uint8_t* )(&(huartDst->Instance->TDR)),num))
			return Status = BOS_ERROR;
		__HAL_DMA_DISABLE_IT(hDMA , DMA_IT_HT);
	}

	return Status;
}

/****************************************************************************************/

/*
 * @brief: Stop (Stream or message) DMA.
 * param1: port
 * @retval: BOS_Status.
 */

BOS_Status StopDMA(uint8_t port){
	BOS_Status Status = BOS_OK;
	DMA_HandleTypeDef *hDMA;
	UART_HandleTypeDef *huartSrc;

	huartSrc = GetUart(port);
	hDMA = UARTDMAHandler[port - 1];

	if(HAL_OK != HAL_UART_DMAStop(huartSrc))
		return Status = BOS_ERROR;

	hDMA->Instance->CNDTR =0;
	if(portStatus[port] == STREAM)
	{
		dmaStreamCount[port - 1] = 0;
		dmaStreamTotal[port - 1] = 0;
	}

	return Status;
}

/****************************************************************************************/

/*
 * @brief: Switch messaging DMA channels to streaming.
 * @param1: port
 * @retval: BOS_Status.
 */

BOS_Status SwitchMsgDMAToStream(uint8_t port){
	BOS_Status Status = BOS_OK;
	UART_HandleTypeDef *huartSrc;

	if(BOS_OK != StopDMA(port))
		return Status = BOS_ERROR;

	huartSrc=GetUart(port);
	HAL_UART_MspInit(huartSrc);

	return Status;
}

/****************************************************************************************/

/*
 * @brief: Switch streaming DMA channel to messaging.
 * @param1: port
 * @retval: BOS_Status.
 */

BOS_Status SwitchStreamDMAToMsg(uint8_t port) {
	BOS_Status Status = BOS_OK;
	UART_HandleTypeDef *huartSrc;

	if(BOS_OK != StopDMA(port))
		return Status = BOS_ERROR;

	huartSrc=GetUart(port);
	/* Initialize a messaging DMA using same channels */
	HAL_UART_MspInit(huartSrc);
	/* change port status */
	portStatus[GetPort(UARTDMAHandler[port - 1]->Parent)] = FREE;

	dmaStreamDst[port - 1] = 0;
	index_process[port - 1] = 0;
	/* Read this port again in messaging mode */
	if(BOS_OK != DMA_MSG_RX_Setup(GetUart(port), UARTDMAHandler[port - 1]))
		return Status = BOS_ERROR;

	return Status;
}

/****************************************************************************************/

void CRC_Init(void){
	hcrc.Instance = CRC;
	hcrc.Init.CRCLength = CRC_POLYLENGTH_8B; // Do not change this since it is used for message CRC8
	hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_ENABLE;
	hcrc.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_ENABLE;
	hcrc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_NONE;
	hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;
	hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_WORDS;
	HAL_CRC_Init(&hcrc);
}

/****************************************************************************************/

void HAL_CRC_MspInit(CRC_HandleTypeDef *hcrc){
	/* Enable peripheral clock */
	__HAL_RCC_CRC_CLK_ENABLE();
}

/****************************************************************************************/

void HAL_CRC_MspDeInit(CRC_HandleTypeDef *hcrc){
	/* Disable peripheral clock */
	__HAL_RCC_CRC_CLK_DISABLE();
}

/****************************************************************************************/

/*-----------------------------------------------------------*/
/*
 * calculate CRC8 byte for a data buffer
 */
uint8_t  CalculateCRC8(uint8_t pBuffer[], uint16_t size)
{
  uint8_t pTemp;
  uint8_t temp_index;
  uint8_t temp_buffer[4] = {0};

  /* check if the passed variables are null */
  if (NULL!=pBuffer && 0!=size)
  {
	if(size < 4)
	{
		temp_index = 0;
		for(int i=0; i<4; i++)
		{
			temp_buffer[i] = pBuffer[temp_index++];
			if(--size == 0) break;
		}
		pTemp=HAL_CRC_Calculate(&hcrc, (uint32_t*)temp_buffer, 1);

	}

	else
	{
		pTemp=HAL_CRC_Calculate(&hcrc, (uint32_t*)pBuffer, size/4);
		if ((size%4)!=0)
		{
			temp_index = size - (size%4);
			size %= 4;
			for(int i=0; i<4; i++)
			{
				temp_buffer[i] = pBuffer[temp_index++];
				if(--size == 0) break;
			}
		  	pTemp=HAL_CRC_Accumulate(&hcrc, (uint32_t*)temp_buffer, 1);

		}
	}

	return pTemp;
  }

else
	return 0;
}

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
