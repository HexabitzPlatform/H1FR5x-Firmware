/*
 BitzOS (BOS) V0.3.6 - Copyright (C) 2017-2024 Hexabitz
 All rights reserved
 
 File Name     : H1FR5.h
 Description   : Header file for module H1FR5.
 	 	 	 	 (Description_of_module)

(Description of Special module peripheral configuration):
>>
>>
>>

 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef H1FR5_H
#define H1FR5_H

/* Includes ------------------------------------------------------------------*/
#include "BOS.h"
#include "H1FR5_MemoryMap.h"
#include "H1FR5_uart.h"
#include "H1FR5_gpio.h"
#include "H1FR5_dma.h"
#include "H1FR5_inputs.h"
#include "H1FR5_eeprom.h"
/* Exported definitions -------------------------------------------------------*/

#define	modulePN		_H1FR5


/* Port-related definitions */
#define	NumOfPorts			5

#define P_PROG 				P2						/* ST factory bootloader UART */

/* Define available ports */
#define _P1 
#define _P2 
#define _P3 
#define _P4 
#define _P5 


/* Define available USARTs */
#define _Usart1 1
#define _Usart2 1
#define _Usart3 1
#define _Usart4 1
#define _Usart5	1
#define _Usart6	1


/* Port-UART mapping */

#define P1uart &huart4
#define P2uart &huart2
#define P3uart &huart3
#define P4uart &huart1
#define P5uart &huart6

/* Port Definitions */
#define	USART1_TX_PIN		GPIO_PIN_9
#define	USART1_RX_PIN		GPIO_PIN_10
#define	USART1_TX_PORT		GPIOA
#define	USART1_RX_PORT		GPIOA
#define	USART1_AF			GPIO_AF1_USART1

#define	USART2_TX_PIN		GPIO_PIN_2
#define	USART2_RX_PIN		GPIO_PIN_3
#define	USART2_TX_PORT		GPIOA
#define	USART2_RX_PORT		GPIOA
#define	USART2_AF			GPIO_AF1_USART2

#define	USART3_TX_PIN		GPIO_PIN_10
#define	USART3_RX_PIN		GPIO_PIN_11
#define	USART3_TX_PORT		GPIOB
#define	USART3_RX_PORT		GPIOB
#define	USART3_AF			GPIO_AF4_USART3

#define	USART4_TX_PIN		GPIO_PIN_0
#define	USART4_RX_PIN		GPIO_PIN_1
#define	USART4_TX_PORT		GPIOA
#define	USART4_RX_PORT		GPIOA
#define	USART4_AF			GPIO_AF4_USART4

#define	USART4_TX_PIN		GPIO_PIN_0
#define	USART4_RX_PIN		GPIO_PIN_1
#define	USART4_TX_PORT		GPIOA
#define	USART4_RX_PORT		GPIOA
#define	USART4_AF			GPIO_AF4_USART4

#define	USART5_TX_PIN		GPIO_PIN_0
#define	USART5_RX_PIN		GPIO_PIN_1
#define	USART5_TX_PORT		GPIOB
#define	USART5_RX_PORT		GPIOB
#define	USART5_AF			GPIO_AF8_USART5

#define	USART6_TX_PIN		GPIO_PIN_8
#define	USART6_RX_PIN		GPIO_PIN_9
#define	USART6_TX_PORT		GPIOB
#define	USART6_RX_PORT		GPIOB
#define	USART6_AF			GPIO_AF8_USART6

/* Module-specific Definitions */

#define NUM_MODULE_PARAMS						1

#define SAMPLE_TEM              0
#define SAMPLE_TO_PORT          1
#define STREAM_TO_PORT          2
#define STREAM_TO_Terminal      3
#define DEFAULT                 4
/* Module EEPROM Variables */

// Module Addressing Space 500 - 599
#define _EE_MODULE							500		


typedef enum {
	Heigh=0,
	Speed,
	UTC,
	Position,

}All_Data;
typedef enum {
	Heigh_buf=0,
	Speed_buf,
	longdegree_buf,
	latdegree_buf,

}buffer_Data;
/* Module_Status Type Definition */
typedef enum {
	H1FR5_OK =0,
	H1FR5_ERR_UnknownMessage,
	H1FR5_ERR_WrongParams,
	H1FR5_ERR_TERMINATED,
	H1FR5_ERR_BUSY,
	H1FR5_ERROR =255
} Module_Status;

/* Indicator LED */
#define _IND_LED_PORT			GPIOB
#define _IND_LED_PIN			GPIO_PIN_14

/* Export UART variables */
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart6;

/* Define UART Init prototypes */
extern void MX_USART1_UART_Init(void);
extern void MX_USART2_UART_Init(void);
extern void MX_USART3_UART_Init(void);
extern void MX_USART4_UART_Init(void);
extern void MX_USART5_UART_Init(void);
extern void MX_USART6_UART_Init(void);
extern void SystemClock_Config(void);
extern void ExecuteMonitor(void);



/* -----------------------------------------------------------------------
 |								  APIs							          |  																 	|
/* -----------------------------------------------------------------------
 */

void GPSHandel(void);
void SetupPortForRemoteBootloaderUpdate(uint8_t port);
void remoteBootloaderUpdate(uint8_t src,uint8_t dst,uint8_t inport,uint8_t outport);
Module_Status StreamToPort(uint8_t module,uint8_t port,All_Data function,  uint32_t Numofsamples, uint32_t timeout);
Module_Status StreamToTerminal(uint8_t port,All_Data function,uint32_t Numofsamples, uint32_t timeout);
Module_Status GetPosition(float * longdegree, float * latdegree, char *longindicator,char *latindicator);
Module_Status GetUTC(uint8_t *hours, uint8_t *min, uint8_t *sec);
Module_Status GetSpeed(float *speedinch, float *speedkm);
Module_Status GetHeight(float *height);
Module_Status StreamToBuffer(float *buffer,buffer_Data function, uint32_t Numofsamples, uint32_t timeout);

/* -----------------------------------------------------------------------
 |								Commands							      |															 	|
/* -----------------------------------------------------------------------
 */


#endif /* H1FR5_H */

/************************ (C) COPYRIGHT HEXABITZ *****END OF FILE****/
