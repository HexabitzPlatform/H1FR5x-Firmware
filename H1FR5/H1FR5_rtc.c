/*
 BitzOS (BOS) V0.3.6 - Copyright (C) 2017-2025 Hexabitz
 All rights reserved

 File Name     : H1FR5_rtc.c
 Description   : Peripheral RTC setup source file.

 */

/* Includes ------------------------------------------------------------------*/
#include "BOS.h"

/* Variables ---------------------------------------------------------*/
RTC_HandleTypeDef RtcHandle;
uint8_t bootStatus =POWER_ON_BOOT;
extern const char *monthStringAbreviated[];

/* Function prototypes ----------------------------------------------*/
BOS_Status RTC_Init(void);
BOS_Status RTC_CalendarConfig(void);
void HAL_RTC_MspInit(RTC_HandleTypeDef* hrtc);
void HAL_RTC_MspDeInit(RTC_HandleTypeDef* hrtc);

/* Functions ---------------------------------------------------------*/

/*
 * @brief: Initialize and configure the internal real-time clock (RTC) and boot status..
 * @retval: BOS_Status.
 */

BOS_Status RTC_Init(void){
	RtcHandle.Instance = RTC;
	RtcHandle.Init.HourFormat = RTC_HOURFORMAT_24;
	RtcHandle.Init.AsynchPrediv = 127;
	RtcHandle.Init.SynchPrediv = 255;
	RtcHandle.Init.OutPut = RTC_OUTPUT_DISABLE;
	RtcHandle.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
	RtcHandle.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
	RtcHandle.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
	RtcHandle.Init.OutPutPullUp = RTC_OUTPUT_PULLUP_NONE;

	/* read time format(12/24) from specified RTC Backup */
	RtcHandle.TampOffset = (TAMP_BASE - RTC_BASE);
	/* Enable RTC clock in order to read from RTC Backup */
	HAL_RTC_MspInit(&RtcHandle);

	if(HAL_RTCEx_BKUPRead(&RtcHandle,RTC_BKP_DR0) == 1)
	{
		RtcHandle.Init.HourFormat = RTC_HOURFORMAT_12;
	}

	if (HAL_RTC_Init(&RtcHandle) != HAL_OK)
		return BOS_ERROR;


	/* Check if Data stored in BackUp register1: No Need to reconfigure RTC */
	/* Read the Back Up Register 1 Data */
	if(HAL_RTCEx_BKUPRead(&RtcHandle,RTC_BKP_DR1) != 0x32F2){
		/* Configure RTC Calendar */
		RTC_CalendarConfig();
	}
	else{
		/* Check if the Power On Reset flag is set */
		if(__HAL_RCC_GET_FLAG(RCC_FLAG_PWRRST) != RESET){
			bootStatus =POWER_ON_BOOT;
		}
		/* Check if Pin Reset flag is set */
		if(__HAL_RCC_GET_FLAG(RCC_FLAG_PINRST) != RESET){
			bootStatus =RESET_BOOT;
		}
	}
	/* Clear source Reset Flag */
	__HAL_RCC_CLEAR_RESET_FLAGS();

	return BOS_OK;
}

/**********************************************************************************/

/*
 * @brief: First time-configuration of the internal real-time clock.
 * @retval: BOS_Status.
 */

BOS_Status RTC_CalendarConfig(void){
	RTC_DateTypeDef sdatestructure;
	RTC_TimeTypeDef stimestructure;
	uint8_t month, day, year, seconds, minutes, hours;
	char comDate[] = __DATE__, comTime[] = __TIME__;
	
	/* Get compile date */
	year =atoi(comDate + 9);		// only last 2 digits
	*(comDate + 6) =0;
	day =atoi(comDate + 4);
	*(comDate + 3) =0;
	for(uint8_t i =0; i < 12; i++){
		if(!strcmp(comDate,monthStringAbreviated[i]))
			month =i + 1;
	}
	
	/* Get compile time */
	seconds =atoi(comTime + 6);
	*(comDate + 5) =0;
	minutes =atoi(comTime + 3);
	*(comDate + 2) =0;
	hours =atoi(comTime);
	
	/* Set Date */
	sdatestructure.Year =year;
	sdatestructure.Month =month;
	sdatestructure.Date =day;
	sdatestructure.WeekDay = RTC_WEEKDAY_MONDAY;		// Todo - Calculate weekday later
	
	if(HAL_RTC_SetDate(&RtcHandle,&sdatestructure,RTC_FORMAT_BIN) != HAL_OK)
		return BOS_ERROR;
	
	/* Set Time */
	stimestructure.Hours =hours;
	stimestructure.Minutes =minutes;
	stimestructure.Seconds =seconds;
	stimestructure.TimeFormat = RTC_HOURFORMAT12_AM;
	BOS.hourformat =24;
	stimestructure.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
	stimestructure.StoreOperation = RTC_STOREOPERATION_RESET;
	
	if(HAL_RTC_SetTime(&RtcHandle,&stimestructure,RTC_FORMAT_BIN) != HAL_OK)
		return BOS_ERROR;
	
	/* Writes a data in a RTC Backup data Register1 */
	HAL_RTCEx_BKUPWrite(&RtcHandle,RTC_BKP_DR1,0x32F2);
	
	return BOS_OK;
}

/**********************************************************************************/

/*
 * @brief: BOS internal real-time clock and calendar configuration.
 * param1: month
 * param2: month day
 * param3: year
 * param4: week day
 * param5: seconds
 * param6: minutes
 * param7: hours
 * param8: AMPM
 * @retval: BOS_Status.
 */

BOS_Status BOS_CalendarConfig(Months_e month, uint8_t monthDay, uint16_t year, Weekdays_e weekDay, uint8_t seconds, uint8_t minutes, uint8_t hours, TimePeriod_e AMPM){
	RTC_DateTypeDef sdatestructure;
	RTC_TimeTypeDef stimestructure;
	
	/* Set Date */
	sdatestructure.Year =year - 2000;
	sdatestructure.Month =month;
	sdatestructure.Date =monthDay;
	sdatestructure.WeekDay =weekDay;		// Todo - Calculate weekday later
	
	/* Set Time */
	stimestructure.Hours =hours;
	stimestructure.Minutes =minutes;
	stimestructure.Seconds =seconds;
	stimestructure.StoreOperation = RTC_STOREOPERATION_RESET;		// Todo - Use this to make sure user does not change daylight settings again
	
	if(AMPM == RTC_AM && hours <= 12){
		HAL_RTCEx_BKUPWrite(&RtcHandle, RTC_BKP_DR0, 1);
		stimestructure.TimeFormat = RTC_HOURFORMAT12_AM;
		BOS.hourformat =12;
		HAL_RTC_DeInit(&RtcHandle);
		RtcHandle.Init.HourFormat = RTC_HOURFORMAT_12;
		HAL_RTC_Init(&RtcHandle);
	}
	else if(AMPM == RTC_PM && hours <= 12){
		HAL_RTCEx_BKUPWrite(&RtcHandle, RTC_BKP_DR0, 1);
		stimestructure.TimeFormat = RTC_HOURFORMAT12_PM;
		BOS.hourformat =12;
		HAL_RTC_DeInit(&RtcHandle);
		RtcHandle.Init.HourFormat = RTC_HOURFORMAT_12;
		HAL_RTC_Init(&RtcHandle);
	}
	else
	{
		HAL_RTCEx_BKUPWrite(&RtcHandle, RTC_BKP_DR0, 0);
		BOS.hourformat =24;
		HAL_RTC_DeInit(&RtcHandle);
		RtcHandle.Init.HourFormat = RTC_HOURFORMAT_24;
		HAL_RTC_Init(&RtcHandle);
	}
	
	if(HAL_RTC_SetTime(&RtcHandle,&stimestructure,RTC_FORMAT_BIN) != HAL_OK)
		return BOS_ERROR;
	
	if(HAL_RTC_SetDate(&RtcHandle,&sdatestructure,RTC_FORMAT_BIN) != HAL_OK)
		return BOS_ERROR;
	/* Save RTC hourformat and daylightsaving to EEPROM */
	EE_WriteVariable(_EE_PARAMS_RTC,((uint16_t )BOS.hourformat << 8) | (uint16_t )BOS.buttons.minInterClickTime);
	
	/* Writes a data in a RTC Backup data Register1 */
	HAL_RTCEx_BKUPWrite(&RtcHandle,RTC_BKP_DR1,0x32F2);
	
	return BOS_OK;
}

/**********************************************************************************/

/*
 * @brief: Get current RTC time and date.
 * @retval: None.
 */

void GetTimeDate(void){
	RTC_DateTypeDef sdatestructureget;
	RTC_TimeTypeDef stimestructureget;
	
	HAL_RTC_GetTime(&RtcHandle,&stimestructureget,RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&RtcHandle,&sdatestructureget,RTC_FORMAT_BIN);
	
	BOS.time.ampm =(stimestructureget.TimeFormat >> 7) + 1;
	BOS.time.msec =stimestructureget.SubSeconds / 2;
	BOS.time.seconds =stimestructureget.Seconds;
	BOS.time.minutes =stimestructureget.Minutes;
	BOS.time.hours =stimestructureget.Hours;
	BOS.date.day =sdatestructureget.Date;
	BOS.date.month =sdatestructureget.Month;
	BOS.date.weekday =sdatestructureget.WeekDay;
	BOS.date.year =sdatestructureget.Year + 2000;
}

/**********************************************************************************/

/**
* @brief RTC MSP Initialization
* This function configures the hardware resources used in this example
* @param: RTC handle pointer
* @retval None
*/

void HAL_RTC_MspInit(RTC_HandleTypeDef* hrtc)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
  if(hrtc->Instance==RTC)
  {
  /* USER CODE BEGIN RTC_MspInit 0 */

  /* USER CODE END RTC_MspInit 0 */

  /** Initializes the peripherals clocks
  */
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
    PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
      /*Error_Handler();*/
    }

    /* Peripheral clock enable */
    __HAL_RCC_RTC_ENABLE();
    __HAL_RCC_RTCAPB_CLK_ENABLE();
  /* USER CODE BEGIN RTC_MspInit 1 */

  /* USER CODE END RTC_MspInit 1 */
  }

}

/**********************************************************************************/

/**
* @brief RTC MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param: RTC handle pointer
* @retval None
*/
void HAL_RTC_MspDeInit(RTC_HandleTypeDef* hrtc)
{
  if(hrtc->Instance==RTC)
  {
  /* USER CODE BEGIN RTC_MspDeInit 0 */

  /* USER CODE END RTC_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_RTC_DISABLE();
    __HAL_RCC_RTCAPB_CLK_DISABLE();
  /* USER CODE BEGIN RTC_MspDeInit 1 */

  /* USER CODE END RTC_MspDeInit 1 */
  }

}

/*-----------------------------------------------------------*/

/************************ (C) COPYRIGHT HEXABITZ *****END OF FILE****/
