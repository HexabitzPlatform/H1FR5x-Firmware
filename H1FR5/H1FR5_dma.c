/*
 BitzOS (BOS) V0.3.5 - Copyright (C) 2017-2024 Hexabitz
 All rights reserved
 
 File Name     : H1FR5_dma.c
 Description   : source file Contains Peripheral DMA setup .

 */

/* Includes ------------------------------------------------------------------*/
#include "BOS.h"

/*----------------------------------------------------------------------------*/
/* Configure DMA                                                              */
/*----------------------------------------------------------------------------*/

/* Variables ---------------------------------------------------------*/

/* DMA structs. Number of structs depends on available DMA channels and array ports where some channels might be reconfigured. 
 - Update for non-standard MCUs 
 */
DMA_HandleTypeDef msgRxDMA[6] ={0};
DMA_HandleTypeDef msgTxDMA[3] ={0};
DMA_HandleTypeDef streamDMA[6] ={0};
DMA_HandleTypeDef frontendDMA[3] ={0};
DMAMUX_Channel_TypeDef DMAMUXRx[6]={0};
DMAMUX_Channel_TypeDef DMAMUXTx[6]={0};
CRC_HandleTypeDef hcrc;

extern uint8_t UARTRxBuf[NumOfPorts][MSG_RX_BUF_SIZE];
//extern uint8_t UARTTxBuf[3][MSG_TX_BUF_SIZE];

/* Private function prototypes -----------------------------------------------*/
void SetupDMAInterrupts(DMA_HandleTypeDef *hDMA,uint8_t priority);
void UnSetupDMAInterrupts(DMA_HandleTypeDef *hDMA);
void RemapAndLinkDMAtoUARTRx(UART_HandleTypeDef *huart,DMA_HandleTypeDef *hDMA);
void RemapAndLinkDMAtoUARTTx(UART_HandleTypeDef *huart,DMA_HandleTypeDef *hDMA);

/*-----------------------------------------------------------*/

/** 
 * Initialize the DMAs
 */
void DMA_Init(void){
	/* DMA controller clock enable */
	__DMA1_CLK_ENABLE();
	__DMA2_CLK_ENABLE();

	/* Initialize messaging RX DMAs x 6 - Update for non-standard MCUs */
#ifdef _P1
	DMA_MSG_RX_CH_Init(&msgRxDMA[0],DMA1_Channel4);
//	DMA_MSG_TX_CH_Init(&msgTxDMA[0],DMA2_Channel1);
#endif
#ifdef _P2
	DMA_MSG_RX_CH_Init(&msgRxDMA[1],DMA1_Channel2);
//	DMA_MSG_TX_CH_Init(&msgTxDMA[1],DMA1_Channel4);
#endif
#ifdef _P3
	DMA_MSG_RX_CH_Init(&msgRxDMA[2],DMA1_Channel3);
//	DMA_MSG_TX_CH_Init(&msgTxDMA[2],DMA1_Channel6);
#endif
#ifdef _P4
	DMA_MSG_RX_CH_Init(&msgRxDMA[3],DMA1_Channel1);
//	DMA_MSG_TX_CH_Init(&msgTxDMA[3],DMA1_Channel2);
#endif
#ifdef _P5
	DMA_MSG_RX_CH_Init(&msgRxDMA[4],DMA1_Channel5);
//	DMA_MSG_TX_CH_Init(&msgTxDMA[4],DMA2_Channel3);
#endif
#ifdef _P6
	DMA_MSG_RX_CH_Init(&msgRxDMA[5],DMA1_Channel6);
//	DMA_MSG_TX_CH_Init(&msgTxDMA[5],DMA2_Channel5);
#endif



	/* Initialize streaming RX DMAs x 0 */
	// No more channels. Dynamically reconfigure from messaging RX DMAs.
	/* Initialize frontend DMAs x 3 - Update for each module */
	//DMA_FRONTEND_CH_Init(&frontendDMA[0], DMA2_Channel5);
}

/*-----------------------------------------------------------*/
/* Initialization functions ---------------------------------*/
/*-----------------------------------------------------------*/

/* Initialize a messaging RX DMA channel 
 */
void DMA_MSG_RX_CH_Init(DMA_HandleTypeDef *hDMA,DMA_Channel_TypeDef *ch){
	hDMA->Instance =ch;
	hDMA->Init.Direction = DMA_PERIPH_TO_MEMORY;
	hDMA->Init.PeriphInc = DMA_PINC_DISABLE;
	hDMA->Init.MemInc = DMA_MINC_ENABLE;
	hDMA->Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
	hDMA->Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
	hDMA->Init.Mode = DMA_CIRCULAR;
	hDMA->Init.Priority = MSG_DMA_PRIORITY;
	
	HAL_DMA_Init(hDMA);

}

/*-----------------------------------------------------------*/

/* Initialize a messaging TX DMA channel 
 */
void DMA_MSG_TX_CH_Init(DMA_HandleTypeDef *hDMA,DMA_Channel_TypeDef *ch){
	hDMA->Instance =ch;
	hDMA->Init.Direction = DMA_MEMORY_TO_PERIPH;
	hDMA->Init.PeriphInc = DMA_PINC_DISABLE;
	hDMA->Init.MemInc = DMA_MINC_ENABLE;
	hDMA->Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
	hDMA->Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
	hDMA->Init.Mode = DMA_NORMAL;
	hDMA->Init.Priority = MSG_DMA_PRIORITY;
	
	HAL_DMA_Init(hDMA);
}

/*-----------------------------------------------------------*/

/* Initialize a streaming DMA channel (RX only) 
 */
void DMA_STREAM_CH_Init(DMA_HandleTypeDef *hDMA,DMA_Channel_TypeDef *ch){
	hDMA->Instance =ch;
	hDMA->Init.Direction = DMA_PERIPH_TO_MEMORY;
	hDMA->Init.PeriphInc = DMA_PINC_DISABLE;
	hDMA->Init.MemInc = DMA_MINC_DISABLE;
	hDMA->Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
	hDMA->Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
	hDMA->Init.Mode = DMA_CIRCULAR;
	hDMA->Init.Priority = STREAM_DMA_PRIORITY;
	
	HAL_DMA_Init(hDMA);
}

/*-----------------------------------------------------------*/

/* Initialize a frontend DMA channel - modify based on frontend needs 
 */
void DMA_FRONTEND_CH_Init(DMA_HandleTypeDef *hDMA,DMA_Channel_TypeDef *ch){
	hDMA->Instance =ch;
	hDMA->Init.Direction = DMA_PERIPH_TO_MEMORY;
	hDMA->Init.PeriphInc = DMA_PINC_DISABLE;
	hDMA->Init.MemInc = DMA_MINC_DISABLE;
	hDMA->Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
	hDMA->Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
	hDMA->Init.Mode = DMA_CIRCULAR;
	hDMA->Init.Priority = FRONTEND_DMA_PRIORITY;
	
	HAL_DMA_Init(hDMA);
}

/*-----------------------------------------------------------*/
/* Setup and control functions ------------------------------*/
/*-----------------------------------------------------------*/

/* Setup and start Messaging DMAs 
 */
void SetupMessagingRxDMAs(void){
#ifdef _P1
	if(portStatus[P1] == FREE)
		DMA_MSG_RX_Setup(P1uart,&msgRxDMA[0]);
#endif
#ifdef _P2
	if(portStatus[P2] == FREE)
		DMA_MSG_RX_Setup(P2uart,&msgRxDMA[1]);
#endif
#ifdef _P3	
	if(portStatus[P3] == FREE)
		DMA_MSG_RX_Setup(P3uart,&msgRxDMA[2]);
#endif
#ifdef _P4		
	if(portStatus[P4] == FREE)
		DMA_MSG_RX_Setup(P4uart,&msgRxDMA[3]);
#endif
#ifdef _P5		
	if(portStatus[P5] == FREE)
		DMA_MSG_RX_Setup(P5uart,&msgRxDMA[4]);
#endif
#ifdef _P6
	if(portStatus[P6] == FREE)
		DMA_MSG_RX_Setup(P6uart,&msgRxDMA[5]);
#endif

}

/*-----------------------------------------------------------*/

/* Messaging DMA RX setup (port-to-memory) 
 */
void DMA_MSG_RX_Setup(UART_HandleTypeDef *huart,DMA_HandleTypeDef *hDMA){
	/* Remap and link to UART Rx */
	//RemapAndLinkDMAtoUARTRx(huart,hDMA);
	//TOBECHECKED

	/* Setup DMA interrupts */
//	SetupDMAInterrupts(hDMA,MSG_DMA_INT_PRIORITY);
	//TOBECHECKED

	/* Start DMA stream	*/
	//HAL_UART_Receive_DMA(huart,(uint8_t* )&Rx_Data[GetPort(huart) - 1] , 1);
//	HAL_UART_Receive_IT(huart,(uint8_t* )&Rx_Data[GetPort(huart) - 1] , 1);
	HAL_UART_Receive_DMA(huart,(uint8_t* )&UARTRxBuf[GetPort(huart) - 1],MSG_RX_BUF_SIZE);
}

/*-----------------------------------------------------------*/

/* Messaging DMA TX setup (memory-to-port) 
 */
void DMA_MSG_TX_Setup(UART_HandleTypeDef *huart){
	DMA_HandleTypeDef *hDMA;
	
	/* Assign the first free TX DMA */
	if(msgTxDMA[0].Parent == NULL)
		hDMA =&msgTxDMA[0];
	else if(msgTxDMA[1].Parent == NULL)
		hDMA =&msgTxDMA[1];
	else if(msgTxDMA[2].Parent == NULL)
		hDMA =&msgTxDMA[2];
	// TODO return no enough TX DMAs
	
	/* Remap and link to UART Tx */
	RemapAndLinkDMAtoUARTTx(huart,hDMA);
	
	/* Setup DMA interrupts */
	SetupDMAInterrupts(hDMA,MSG_DMA_INT_PRIORITY);
	
	/* Start DMA stream	when needed */
}

/*-----------------------------------------------------------*/

/* Unsetup messaging DMA TX (memory-to-port) since TX DMAs are shared
 */
void DMA_MSG_TX_UnSetup(UART_HandleTypeDef *huart){
	/* Setup DMA interrupts */
	UnSetupDMAInterrupts(huart->hdmatx);
	
	/* Unlink the TX DMA and UART */
	huart->hdmatx->Parent = NULL;
	huart->hdmatx = NULL;
}

/*-----------------------------------------------------------*/

/* Streaming DMA setup (port-to-port) 
 */
void DMA_STREAM_Setup(UART_HandleTypeDef *huartSrc,UART_HandleTypeDef *huartDst,uint16_t num){
	DMA_HandleTypeDef *hDMA;
	uint8_t port =GetPort(huartSrc);
	
	/* Select DMA struct */
	hDMA =&streamDMA[port - 1];
	
	/* Remap and link to UART RX */
	RemapAndLinkDMAtoUARTRx(huartSrc,hDMA);

	/* Setup DMA interrupts */
	//SetupDMAInterrupts(hDMA,STREAM_DMA_INT_PRIORITY);
	
	/* Start DMA stream	*/
	huartSrc->gState =HAL_UART_STATE_READY;
	HAL_UART_Receive_DMA(huartSrc,(uint8_t* )(&(huartDst->Instance->TDR)),num);

}

/*-----------------------------------------------------------*/
/* Private functions ----------------------------------------*/
/*-----------------------------------------------------------*/

/* --- Stop a streaming DMA ---
 */
void StopStreamDMA(uint8_t port) {
	DMA_HandleTypeDef *hDMA;
	UART_HandleTypeDef *huartSrc;
	/* Select DMA struct */
	hDMA = &streamDMA[port - 1];
	huartSrc=GetUart(port);
	HAL_UART_DMAStop(huartSrc);
	hDMA->Instance->CNDTR = 0;
	dmaStreamCount[port - 1] = 0;
	dmaStreamTotal[port - 1] = 0;

}


/* --- Stop a messaging DMA ---
 */
void StopMsgDMA(uint8_t port){
	DMA_HandleTypeDef *hDMA;
	UART_HandleTypeDef *huartSrc;
	huartSrc=GetUart(port);
	/* Select DMA struct */
	hDMA =&msgRxDMA[port - 1];
	HAL_UART_DMAStop(huartSrc);
	hDMA->Instance->CNDTR =0;
}


/*-----------------------------------------------------------*/

/* Switch messaging DMA channels to streaming
 */
void SwitchMsgDMAToStream(uint8_t port) {
	// TODO - Make sure all messages in the RX buffer have been parsed?

	// Stop the messaging DMA
	StopMsgDMA(port);
	// Initialize a streaming DMA using same channel
	DMA_STREAM_CH_Init(&streamDMA[port - 1], msgRxDMA[port - 1].Instance);
}

/*-----------------------------------------------------------*/

/* Switch streaming DMA channel to messaging
 */
void SwitchStreamDMAToMsg(uint8_t port) {
	// Stop the streaming DMA
	UART_HandleTypeDef *huartSrc;
	StopStreamDMA(port);
	huartSrc=GetUart(port);
		/* Select DMA struct */
	// Initialize a messaging DMA using same channels
	DMA_MSG_RX_CH_Init(&msgRxDMA[port - 1], streamDMA[port - 1].Instance);
	HAL_UART_MspInit(huartSrc);
	// Remove stream DMA and change port status
	portStatus[GetPort(streamDMA[port - 1].Parent)] = FREE;
	streamDMA[port - 1].Instance = 0;
	dmaStreamDst[port - 1] = 0;

	// Read this port again in messaging mode
	DMA_MSG_RX_Setup(GetUart(port), &msgRxDMA[port - 1]);

}


/* Setup DMA interrupts  
 */
void SetupDMAInterrupts(DMA_HandleTypeDef *hDMA,uint8_t priority){
	switch((uint32_t )hDMA->Instance){
		case (uint32_t ) DMA1_Channel1:
			HAL_NVIC_SetPriority(DMA1_Channel1_IRQn,priority,0);
			HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
			break;
			
		case (uint32_t ) DMA1_Channel2:
		case (uint32_t ) DMA1_Channel3:
		case (uint32_t ) DMA2_Channel1:
		case (uint32_t ) DMA2_Channel2:
			HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn,priority,0);
			HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);
			break;
			
		case (uint32_t ) DMA1_Channel4:
		case (uint32_t ) DMA1_Channel5:
		case (uint32_t ) DMA1_Channel6:
		case (uint32_t ) DMA1_Channel7:
		case (uint32_t ) DMA2_Channel3:
		case (uint32_t ) DMA2_Channel4:
		case (uint32_t ) DMA2_Channel5:
			HAL_NVIC_SetPriority(DMA1_Ch4_7_DMA2_Ch1_5_DMAMUX1_OVR_IRQn,priority,0);
			HAL_NVIC_EnableIRQ(DMA1_Ch4_7_DMA2_Ch1_5_DMAMUX1_OVR_IRQn);
			break;
			
		default:
			break;
	}
}
//TOBECHECKED
/*-----------------------------------------------------------*/

/* UnSetup DMA interrupts  
 */
void UnSetupDMAInterrupts(DMA_HandleTypeDef *hDMA){
	switch((uint32_t )hDMA->Instance){
		case (uint32_t ) DMA1_Channel1:
			//HAL_NVIC_DisableIRQ(DMA1_Ch1_IRQn);
			HAL_NVIC_DisableIRQ(DMA1_Channel1_IRQn);
			break;
			
		case (uint32_t ) DMA1_Channel2:
		case (uint32_t ) DMA1_Channel3:
		case (uint32_t ) DMA2_Channel1:
		case (uint32_t ) DMA2_Channel2:
			HAL_NVIC_DisableIRQ(DMA1_Channel2_3_IRQn);
			break;
			
		case (uint32_t ) DMA1_Channel4:
		case (uint32_t ) DMA1_Channel5:
		case (uint32_t ) DMA1_Channel6:
		case (uint32_t ) DMA1_Channel7:
		case (uint32_t ) DMA2_Channel3:
		case (uint32_t ) DMA2_Channel4:
		case (uint32_t ) DMA2_Channel5:
			HAL_NVIC_DisableIRQ(DMA1_Ch4_7_DMA2_Ch1_5_DMAMUX1_OVR_IRQn);
			break;
			
		default:
			break;
	}
}
//TOBECHECKED
/*-----------------------------------------------------------*/

/* Remap and link the UART RX and DMA structs 
 */
void RemapAndLinkDMAtoUARTRx(UART_HandleTypeDef *huart,DMA_HandleTypeDef *hDMA){
	// USART 1
	if(huart->Instance == USART1 && hDMA->Instance == DMA1_Channel1){
		//__HAL_DMA1_REMAP(HAL_DMA1_CH1_USART1_RX);
		LL_DMAMUX_SetRequestID(&DMAMUXRx[0],LL_DMAMUX_CHANNEL_0, LL_DMAMUX_REQ_USART1_RX);
	}

		// USART 2

	else if(huart->Instance == USART2 && hDMA->Instance == DMA1_Channel2){
		//__HAL_DMA1_REMAP(HAL_DMA1_CH1_USART2_RX);
		LL_DMAMUX_SetRequestID(&DMAMUXRx[1],LL_DMAMUX_CHANNEL_1, LL_DMAMUX_REQ_USART2_RX);
	}

	// USART 3
	else if(huart->Instance == USART3 && hDMA->Instance == DMA1_Channel3){
		//__HAL_DMA1_REMAP(HAL_DMA1_CH1_USART3_RX);
		LL_DMAMUX_SetRequestID(&DMAMUXRx[2],LL_DMAMUX_CHANNEL_2, LL_DMAMUX_REQ_USART3_RX);
	}

	// USART 4
	else if(huart->Instance == USART4 && hDMA->Instance == DMA1_Channel4){
		//__HAL_DMA1_REMAP(HAL_DMA1_CH1_USART4_RX);
		LL_DMAMUX_SetRequestID(&DMAMUXRx[3],LL_DMAMUX_CHANNEL_3, LL_DMAMUX_REQ_USART4_RX);
	}

	// USART 5
	else if(huart->Instance == USART5 && hDMA->Instance == DMA1_Channel5){
		//__HAL_DMA1_REMAP(HAL_DMA1_CH1_USART5_RX);
		LL_DMAMUX_SetRequestID(&DMAMUXRx[4],LL_DMAMUX_CHANNEL_4, LL_DMAMUX_REQ_USART5_RX);
	}

	// USART 6
	else if(huart->Instance == USART6 && hDMA->Instance == DMA1_Channel6){
		//__HAL_DMA1_REMAP(HAL_DMA1_CH1_USART6_RX);
		LL_DMAMUX_SetRequestID(&DMAMUXRx[5],LL_DMAMUX_CHANNEL_5, LL_DMAMUX_REQ_USART6_RX);
	}


	__HAL_LINKDMA(huart,hdmarx,*hDMA);
}

/*-----------------------------------------------------------*/

/* Remap and link the UART TX and DMA structs 
 */
void RemapAndLinkDMAtoUARTTx(UART_HandleTypeDef *huart,DMA_HandleTypeDef *hDMA){
	// USART 1
	if(huart->Instance == USART1 && hDMA->Instance == DMA1_Channel2){
	//	__HAL_DMA1_REMAP(HAL_DMA1_CH2_USART1_TX);
		LL_DMAMUX_SetRequestID(&DMAMUXTx[0],LL_DMAMUX_CHANNEL_1, LL_DMAMUX_REQ_USART1_TX);
	}

	// USART 2
	else if(huart->Instance == USART2 && hDMA->Instance == DMA1_Channel4){
		//__HAL_DMA1_REMAP(HAL_DMA1_CH2_USART2_TX);
		LL_DMAMUX_SetRequestID(&DMAMUXTx[1],LL_DMAMUX_CHANNEL_3, LL_DMAMUX_REQ_USART2_TX);
	}

	// USART 3
	else if(huart->Instance == USART3 && hDMA->Instance == DMA1_Channel6){
		//__HAL_DMA1_REMAP(HAL_DMA1_CH2_USART3_TX);
		LL_DMAMUX_SetRequestID(&DMAMUXTx[2],LL_DMAMUX_CHANNEL_5, LL_DMAMUX_REQ_USART3_TX);
	}

	// USART 4
	else if(huart->Instance == USART4 && hDMA->Instance == DMA2_Channel1){
		//__HAL_DMA1_REMAP(HAL_DMA1_CH2_USART4_TX);
		LL_DMAMUX_SetRequestID(&DMAMUXTx[3],LL_DMAMUX_CHANNEL_7, LL_DMAMUX_REQ_USART4_TX);
	}

	// USART 5
	else if(huart->Instance == USART5 && hDMA->Instance == DMA2_Channel3){
		//__HAL_DMA1_REMAP(HAL_DMA1_CH2_USART5_TX);
		LL_DMAMUX_SetRequestID(&DMAMUXTx[4],LL_DMAMUX_CHANNEL_9, LL_DMAMUX_REQ_USART5_TX);
	}
    // USART 6
	else if(huart->Instance == USART6 && hDMA->Instance == DMA2_Channel5){
		//__HAL_DMA1_REMAP(HAL_DMA1_CH2_USART6_TX);
		LL_DMAMUX_SetRequestID(&DMAMUXTx[5],LL_DMAMUX_CHANNEL_11, LL_DMAMUX_REQ_USART6_TX);
	}


	__HAL_LINKDMA(huart,hdmatx,*hDMA);
}
//TOBECHECKED
/*-----------------------------------------------------------*/
/* Hardware CRC ---------------------------------------------*/
/*-----------------------------------------------------------*/

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

void HAL_CRC_MspInit(CRC_HandleTypeDef *hcrc){
	/* Enable peripheral clock */
	__HAL_RCC_CRC_CLK_ENABLE();
}

void HAL_CRC_MspDeInit(CRC_HandleTypeDef *hcrc){
	/* Disable peripheral clock */
	__HAL_RCC_CRC_CLK_DISABLE();
}

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
