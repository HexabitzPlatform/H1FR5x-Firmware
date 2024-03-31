/*
 BitzOS (BOS) V0.3.2 - Copyright (C) 2017-2024 Hexabitz
 All rights reserved

 File Name     : H1FR5_gpio.h
 Description   : Header file contains all the functions prototypes for
 the GPIO .

 */


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __gpio_H
#define __gpio_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g0xx_hal.h"

extern void GPIO_Init(void);
extern void IND_LED_Init(void);

#ifdef __cplusplus
}
#endif
#endif /*__gpio_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
