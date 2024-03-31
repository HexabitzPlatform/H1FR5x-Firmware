/*
 BitzOS (BOS) V0.3.2 - Copyright (C) 2017-2024 Hexabitz
 All rights reserved

 File Name     : H1FR5_rtc.c
 Description   : Peripheral RTC setup source file.

 */
/* Includes ------------------------------------------------------------------*/
#include "BOS.h"

/*----------------------------------------------------------------------------*/
/* Configure RTC                                                              */
/*----------------------------------------------------------------------------*/

/* RTC */
RTC_HandleTypeDef RtcHandle;
uint8_t bootStatus =POWER_ON_BOOT;

extern const char *monthStringAbreviated[];

BOS_Status RTC_Init(void);
BOS_Status RTC_CalendarConfig(void);
/*-----------------------------------------------------------*/

/* --- Initialize and config the internal real-time clock (RTC) and boot status.
 */
BOS_Status RTC_Init(void){
	/* RTC clock enable */
	__HAL_RCC_RTC_ENABLE();
	
	/* Configure the RTC 
	 f_ckspre = f_rtcclk / ((PREDIV_S+1) * (PREDIV_A+1))
	 - f_rtcclk is HSE 8 MHz / 32 = 250 kHz
	 - f_ckspre should be 1 Hz 
	 - PREDIV_A should be as high as possible to minimize power consumption
	 >> Choose PREDIV_A = 124 and PREDIV_S = 1999
	 */
	RtcHandle.Instance = RTC;
	RtcHandle.Init.HourFormat = RTC_HOURFORMAT_24;
	RtcHandle.Init.AsynchPrediv = 124;
	RtcHandle.Init.SynchPrediv = 1999;
	RtcHandle.Init.OutPut = RTC_OUTPUT_DISABLE;
	RtcHandle.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
	RtcHandle.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
	RtcHandle.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
	RtcHandle.Init.OutPutPullUp = RTC_OUTPUT_PULLUP_NONE;
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

/*-----------------------------------------------------------*/

/* --- First time-configuration of the internal real-time clock.
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

/*-----------------------------------------------------------*/

/* --- BOS internal real-time clock and calendar configuration.
 */
BOS_Status BOS_CalendarConfig(uint8_t month,uint8_t day,uint16_t year,uint8_t weekday,uint8_t seconds,uint8_t minutes,uint8_t hours,uint8_t AMPM,int8_t daylightsaving){
	RTC_DateTypeDef sdatestructure;
	RTC_TimeTypeDef stimestructure;
	
	/* Set Date */
	sdatestructure.Year =year - 2000;
	sdatestructure.Month =month;
	sdatestructure.Date =day;
	sdatestructure.WeekDay =weekday;		// Todo - Calculate weekday later
	
	if(HAL_RTC_SetDate(&RtcHandle,&sdatestructure,RTC_FORMAT_BIN) != HAL_OK)
		return BOS_ERROR;
	
	/* Set Time */
	stimestructure.Hours =hours;
	stimestructure.Minutes =minutes;
	stimestructure.Seconds =seconds;
	stimestructure.StoreOperation = RTC_STOREOPERATION_RESET;		// Todo - Use this to make sure user does not change daylight settings again
	
//	if (daylightsaving == DAYLIGHT_NONE) 											// Todo
//		stimestructure.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
//	else if (daylightsaving == DAYLIGHT_ADD1H)
//		stimestructure.DayLightSaving = RTC_DAYLIGHTSAVING_ADD1H;
//	else if (daylightsaving == DAYLIGHT_SUB1H)
//		stimestructure.DayLightSaving = RTC_DAYLIGHTSAVING_SUB1H;
	
	if(hours > 12)
		BOS.hourformat =24;
	
	if(AMPM == RTC_AM){
		stimestructure.TimeFormat = RTC_HOURFORMAT12_AM;
		BOS.hourformat =12;
	}
	else if(AMPM == RTC_PM){
		stimestructure.TimeFormat = RTC_HOURFORMAT12_PM;
		BOS.hourformat =12;
	}
	else
		BOS.hourformat =24;
	
	if(HAL_RTC_SetTime(&RtcHandle,&stimestructure,RTC_FORMAT_BIN) != HAL_OK)
		return BOS_ERROR;
	
	/* Save RTC hourformat and daylightsaving to EEPROM */
	EE_WriteVariable(_EE_PARAMS_RTC,((uint16_t )BOS.hourformat << 8) | (uint16_t )BOS.buttons.minInterClickTime);
	
	/* Writes a data in a RTC Backup data Register1 */
	HAL_RTCEx_BKUPWrite(&RtcHandle,RTC_BKP_DR1,0x32F2);
	
	return BOS_OK;
}

/*-----------------------------------------------------------*/

/* --- Get current RTC time and date.
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

/*-----------------------------------------------------------*/

/************************ (C) COPYRIGHT HEXABITZ *****END OF FILE****/
