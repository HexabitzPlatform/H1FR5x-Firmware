/*
 BitzOS (BOS) V0.3.1 - Copyright (C) 2017-2024 Hexabitz
 All rights reserved

 File Name     : SAM_M10Q.c

 */


#ifndef INC_SAM_M10Q_H_
#define INC_SAM_M10Q_H_

#include "stm32g0xx_hal.h"

#define NMEA_Message_SIZE	   100
#define DMA_Message_SIZE	   1000
#define NSV_IN_VIEW		       10		//Max Number of Satellites in view


typedef enum
{
	invalid = 0,
	valid = 1
}GPS_STATUS_NUMBER;

typedef enum
{
	GP = 1,GL = 2,GA = 3,GB = 4,GN = 5,EI = 6,PUBX = 7
}GPS_TALKER_ID;

typedef enum
{
	DMT = 1,
	GBQ,GBS,GGA,GLL,GLQ,GNQ,GNS,GPQ,GRS,GSA,GST,GSV,RMC,TXT,VLW,VTG,ZDA
}GPS_MESSAGE_ID;

typedef enum {
	Reject  = 0,
	Accept  = 1,
	CRC_error = 2
}Filter_Statues;

typedef struct{
	char Message_Buff[DMA_Message_SIZE];
	uint8_t Message_INDEX;
	uint8_t Message_ID;
} Message_Prosses_Buffer;

typedef struct{
	uint16_t INDEX;
	uint16_t LAST_INDEX;
} DMA_INDEXER;

typedef struct
{
	float Degree;
	char indicator;
	char status;
}Latitude_Longitude_TYPE;

typedef struct
{
	uint8_t hrs;
	uint8_t mint;
	uint8_t sec;
	uint8_t day;
	uint8_t month;
	uint16_t year;
}UTC_TYPE;

typedef struct
{
	float height;
}HEIGHT_TYPE;

typedef struct
{
	float speedKnot;
	float speedKm;
}SPEED_TYPE;

typedef struct
{
	Latitude_Longitude_TYPE Latitude;
	Latitude_Longitude_TYPE Longitude;
	UTC_TYPE UTC_TIME;
	HEIGHT_TYPE HEIGHT;
	SPEED_TYPE SPEED;
	char posMode;
}GPS_TYPE;

typedef struct
{
	uint8_t Satellite_ID;
	uint8_t Elevation;
	uint16_t Azimuth;
	uint8_t Signal_strength;
}Satellites_Info;

typedef uint8_t Number_of_Satellites_t ;

typedef struct
{
	GPS_STATUS_NUMBER POS_Status;
	GPS_STATUS_NUMBER TIME_Status;
	GPS_STATUS_NUMBER Height_Status;
	GPS_STATUS_NUMBER Speed_Status;
}GPS_STATUS;

extern GPS_TYPE GPS_INFO;
extern GPS_STATUS GPS_STATUS_INFO;
extern Satellites_Info Satellites[NSV_IN_VIEW];
extern Message_Prosses_Buffer Message_Incoming;
extern DMA_INDEXER DMA_INDEX;
extern Filter_Statues _FILTER;

//extern char NMEA_Message_TDR[NMEA_Message_SIZE];
//extern char NMEA_Message_RDR[NMEA_Message_SIZE];
//extern uint8_t GP_Number_Of_Satellites;
//extern uint8_t GSV_Satellites_STATUS;
/* -----------------------------------------------------------------------
 |								  APIs							          |  																 	|
/* -----------------------------------------------------------------------
 */
extern void Build_NMEA_message(char NMEA_Talker_IDs[],char _CMD[],char _DATA[]);
extern void Build_PUBX_message(char _DATA[]);
extern void Send_NMEA_Message (USART_TypeDef * _USARTx);
extern void NMEA_FILL_Incoming_Message();
extern GPS_STATUS Get_GPS_GLL_Information(GPS_TYPE* Temp_info);
extern GPS_STATUS Get_GPS_ZDA_Information(GPS_TYPE* Temp_info);
extern GPS_STATUS Get_GPS_RMC_Information(GPS_TYPE* Temp_info);
extern Filter_Statues NMEA_Filter_Incoming_Messages(char _Message_RDR[]);
extern void GPS_IN_Action();
extern void Incoming_Message_Handel();

#endif /* INC_SAM_M8Q_H_ */

/************************ (C) COPYRIGHT HEXABITZ *****END OF FILE****/
