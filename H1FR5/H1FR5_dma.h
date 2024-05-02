/*
 BitzOS (BOS) V0.3.3 - Copyright (C) 2017-2024 Hexabitz
 All rights reserved
 
 File Name     : H1FR5_dma.h
 Description   : Header file contains Peripheral DMA setup.

 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef H1FR5_dma_H
#define H1FR5_dma_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g0xx_hal.h"

/* Check which DMA interrupt occured */
#define HAL_DMA_GET_IT_SOURCE(__HANDLE__, __INTERRUPT__)  ((((__HANDLE__)->ISR & (__INTERRUPT__)) == (__INTERRUPT__)) ? SET : RESET)

/* External variables --------------------------------------------------------*/

/* Export DMA structs */
extern DMA_HandleTypeDef msgRxDMA[6];
extern DMA_HandleTypeDef msgTxDMA[3];
extern DMA_HandleTypeDef streamDMA[6];
extern DMA_HandleTypeDef frontendDMA[3];
extern CRC_HandleTypeDef hcrc;

/* External function prototypes ----------------------------------------------*/
extern void DMA_Init(void);
extern void DMA_MSG_RX_CH_Init(DMA_HandleTypeDef *hDMA,DMA_Channel_TypeDef *ch);
extern void DMA_MSG_TX_CH_Init(DMA_HandleTypeDef *hDMA,DMA_Channel_TypeDef *ch);
extern void DMA_STREAM_CH_Init(DMA_HandleTypeDef *hDMA,DMA_Channel_TypeDef *ch);
extern void SetupMessagingRxDMAs(void);
extern void DMA_MSG_RX_Setup(UART_HandleTypeDef *huart,DMA_HandleTypeDef *hDMA);
extern void DMA_MSG_TX_Setup(UART_HandleTypeDef *huart);
extern void DMA_MSG_TX_UnSetup(UART_HandleTypeDef *huart);
extern void CRC_Init(void);
extern uint8_t  CalculateCRC8(uint8_t pBuffer[], uint16_t size);
extern void StopMsgDMA(uint8_t port);
extern void StopStreamDMA(uint8_t port);
extern void SwitchMsgDMAToStream(uint8_t port);
extern void SwitchStreamDMAToMsg(uint8_t port);

#ifdef __cplusplus
}
#endif

#endif /* H1FR5_dma_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
