/*
 BitzOS (BOS) V0.4.0 - Copyright (C) 2017-2025 Hexabitz
 All rights reserved
 
 File Name     : H1FR5_dma.h
 Description   : Header file contains Peripheral DMA setup.

 */

/* Define to prevent recursive inclusion ***********************************/
#ifndef H0BR4_dma_H
#define H0BR4_dma_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ****************************************************************/
#include "stm32g0xx_hal.h"

/* Check which DMA interrupt occurred */
#define HAL_DMA_GET_IT_SOURCE(__HANDLE__, __INTERRUPT__)  ((((__HANDLE__)->ISR & (__INTERRUPT__)) == (__INTERRUPT__)) ? SET : RESET)

/* Exported Variables ******************************************************/
extern CRC_HandleTypeDef hcrc;
extern DMA_HandleTypeDef *UARTDMAHandler[6];

/* External function *******************************************************/
extern void DMA_Init(void);
extern BOS_Status StopDMA(uint8_t port);
extern BOS_Status SetupMessagingRxDMAs(void);
extern BOS_Status SwitchMsgDMAToStream(uint8_t port);
extern BOS_Status SwitchStreamDMAToMsg(uint8_t port);
extern BOS_Status DMA_MSG_RX_Setup(UART_HandleTypeDef *huart,DMA_HandleTypeDef *hDMA);
extern BOS_Status DMA_STREAM_Setup(UART_HandleTypeDef *huartSrc,UART_HandleTypeDef *huartDst,uint16_t num);

extern void CRC_Init(void);
extern uint8_t  CalculateCRC8(uint8_t pBuffer[], uint16_t size);

#ifdef __cplusplus
}
#endif

#endif /* H0BR4_dma_H */

 /***************** (C) COPYRIGHT HEXABITZ ***** END OF FILE ****************/
