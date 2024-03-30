/*
 BitzOS (BOS) V0.3.1 - Copyright (C) 2017-2024 Hexabitz
 All rights reserved

 File Name     : SAM_M10Q.c

 */


#include "SAM_M10Q.h"
#include <stdlib.h>
#include <string.h>

extern DMA_HandleTypeDef hdma_usart5_rx;
extern UART_HandleTypeDef huart5;

char NMEA_Message_TDR[NMEA_Message_SIZE];		//Receive NMEA GPS Protocol Message Buffer
char NMEA_Message_RDR[NMEA_Message_SIZE];		//Out NMEA GPS Protocol Message Buffer
GPS_TYPE GPS_INFO;
GPS_STATUS GPS_STATUS_INFO;
Satellites_Info Satellites[NSV_IN_VIEW];
Message_Prosses_Buffer Message_Incoming;
DMA_INDEXER DMA_INDEX;
Filter_Statues _FILTER;

uint8_t GP_Number_Of_Satellites;				//Number of Active Satellites
uint8_t GSV_Satellites_STATUS;					//Get All Satellites information 0x00 ERROR || 0x01 Complete
uint8_t speedKm,speedKnot;
uint8_t _flag = 1;
uint16_t _GLL,_RMC,_ZDA,_GGA,_GNS,_VTG;


/*-----------------------------------------------------------*/
void Incoming_Message_Handel()
{

	if(_flag == 1)
	{
		_flag = 0;
	 HAL_UART_Receive_DMA(&huart5, (uint8_t *) &Message_Incoming.Message_Buff[0], 1000);
	}
	//New Char received
	DMA_INDEX.INDEX =   DMA_Message_SIZE - hdma_usart5_rx.Instance->CNDTR;
	if (DMA_INDEX.INDEX != DMA_INDEX.LAST_INDEX)
			{GPS_IN_Action();}
	DMA_INDEX.LAST_INDEX =  DMA_INDEX.INDEX;

}

/*-----------------------------------------------------------*/
/*return the Size of NMEA Message
 * @ _data[] NMEA Buffer*/
uint8_t Size_of_NMEA_Message(char _data[])
{
	for (uint8_t i =0; i<= NMEA_Message_SIZE;i++)
	{
		if (_data[i-1] == '\r' && _data[i] == '\n')
			{return i;}
	}
	return 0xFF;
}

/*-----------------------------------------------------------*/
/*Build CRC NMEA For Message Start after Header '$' And end Before CRC '*'
 * @ _data[] NMEA Buffer
 * @ data_length length of NMEA DATA
 * return CRC*/
uint16_t CRC_XOR(char _data[], size_t data_length)
{
	uint8_t temp_XOR=0x00;
	uint8_t HL_temp_XOR[2];
	uint16_t Return_XOR;

	for(size_t _index=1;_index <= data_length;_index++)
	{
		temp_XOR ^= _data[_index];
	}
	HL_temp_XOR[0] = temp_XOR & 0x0F;
	HL_temp_XOR[1] = temp_XOR & 0xF0;
	HL_temp_XOR[1] = (HL_temp_XOR[1]>>4);
	//Convert HEX to Char
	(HL_temp_XOR[0] <= 9) ? (HL_temp_XOR[0] += 0x30) : (HL_temp_XOR[0] += 0x37);
	(HL_temp_XOR[1] <= 9) ? (HL_temp_XOR[1] += 0x30) : (HL_temp_XOR[1] += 0x37);
	Return_XOR = (HL_temp_XOR[0]<<8);
	Return_XOR += HL_temp_XOR[1];
	return Return_XOR;
}

/*-----------------------------------------------------------*/
/*Build NMEA protocol message on NMEA_Message_TDR[]
 * @NMEA_Talker_IDs[2]		GPS, SBAS, QZSS 	 GP
 *							GLONASS 			 GL
 *							Galileo 			 GA
 *							BeiDou 				 GB
 *			Any combination of GNSS 			 GN
 *
 * @_CMD[3]					Section 30.1.9 Messages Overview	\\page 108
 * @_DATA[]					(char) Data of GPS must start with , and end with *
 * */
void Build_NMEA_message(char NMEA_Talker_IDs[],char _CMD[],char _DATA[])
{
	uint8_t _DATA_Size = 0;
	uint16_t _CRC_WORD;
	NMEA_Message_TDR[0] = '$';
	NMEA_Message_TDR[1] = NMEA_Talker_IDs[0],NMEA_Message_TDR[2] = NMEA_Talker_IDs[1];
	NMEA_Message_TDR[3] = _CMD[0],NMEA_Message_TDR[4] = _CMD[1],NMEA_Message_TDR[5] = _CMD[2];

	//FILL_DATA
	while (_DATA[_DATA_Size] !='*')
	{
		NMEA_Message_TDR[_DATA_Size+6] = _DATA[_DATA_Size];
		_DATA_Size++;
	}
	_CRC_WORD = CRC_XOR(NMEA_Message_TDR,_DATA_Size+5);
	//Message Bulid CRC
	NMEA_Message_TDR[_DATA_Size+6] = '*';
	NMEA_Message_TDR[_DATA_Size+7] = (char) _CRC_WORD;
	NMEA_Message_TDR[_DATA_Size+8] = (char) (_CRC_WORD>>8);
	//message Footer
	NMEA_Message_TDR[_DATA_Size+9] ='\r';
	NMEA_Message_TDR[_DATA_Size+10] = '\n';

}

/*-----------------------------------------------------------*/
/*Build PUBX Message On NMEA_Message_TDR[]*/
void Build_PUBX_message(char _DATA[])
{
	uint8_t _DATA_Size = 0;
	uint16_t _CRC_WORD;
	NMEA_Message_TDR[0] = '$';
	NMEA_Message_TDR[1] = 'P',NMEA_Message_TDR[2] ='U',NMEA_Message_TDR[3] = 'B',NMEA_Message_TDR[4] = 'X';

	//FILL_DATA
	while (_DATA[_DATA_Size] != '*')
	{
		NMEA_Message_TDR[_DATA_Size+5] = _DATA[_DATA_Size];
		_DATA_Size++;
	}
	_CRC_WORD = CRC_XOR(NMEA_Message_TDR,_DATA_Size+4);
	//Message Bulid CRC
	NMEA_Message_TDR[_DATA_Size+5] = '*';
	NMEA_Message_TDR[_DATA_Size+6] = (char) _CRC_WORD;
	NMEA_Message_TDR[_DATA_Size+7] = (char) (_CRC_WORD>>8);
	//message Footer
	NMEA_Message_TDR[_DATA_Size+8] ='\r';
	NMEA_Message_TDR[_DATA_Size+9] = '\n';
}

/*-----------------------------------------------------------*/
/*Fill From DMA TO NMEA_Message_RDR[] Buffer
 * @ N DMA for GPS INDEX Buffer*/
//uint8_t Indexer,k;
//extern UART_HandleTypeDef huart5;
void NMEA_FILL_Incoming_Message()
{
	uint16_t Indexer,k;

	k =  DMA_INDEX.INDEX ;
	Indexer = DMA_INDEX.LAST_INDEX;
	while(k != DMA_INDEX.LAST_INDEX)
	{
		if(k == 0) {k = DMA_Message_SIZE;}
		NMEA_Message_RDR[Message_Incoming.Message_INDEX] = Message_Incoming.Message_Buff[Indexer];
		Indexer++;
		if(Indexer >= DMA_Message_SIZE) {Indexer = 0;}
		if (NMEA_Message_RDR[Message_Incoming.Message_INDEX-1] == '\r' && NMEA_Message_RDR[Message_Incoming.Message_INDEX] == '\n')
			{Message_Incoming.Message_INDEX = 0;Message_Incoming.Message_ID = 1;}
		else
			{Message_Incoming.Message_INDEX++;}
		k--;
	}
}

/*-----------------------------------------------------------*/
/*Check Form NMEA Incoming Messages
 * @ _Message_RDR[] NMEA Incoming Messages Buffer
 * return Filter_Statues (Reject | CRC_error | Accept)*/
Filter_Statues NMEA_Filter_Incoming_Messages(char _Message_RDR[])
{
	//check Header
	if (_Message_RDR[0] != '$')
		{return Reject;}
	uint8_t Message_len = Size_of_NMEA_Message(_Message_RDR);
	uint16_t _CRC_WORD = CRC_XOR(_Message_RDR,Message_len-5);

	//if CRC ERROR
	if (_Message_RDR[Message_len-4] != '*')
		{return Reject;}
	else if (_Message_RDR[Message_len-3] != (uint8_t) _CRC_WORD)
		{return CRC_error;}
	else if (_Message_RDR[Message_len-2] != (uint8_t) (_CRC_WORD>>8))
		{return CRC_error;}
	//else no Error
	return Accept;
}

/*-----------------------------------------------------------*/
/*Get Index Number For NMEA_Message_RDR[] From NMEA_Field_No*/
uint8_t NMEA_Field_No_INDEX(uint8_t _Field_No)
{
	uint8_t _Indexing = 0;
	uint8_t Temp_Field_No = 0;
	while (NMEA_Message_RDR[_Indexing] !='*')
	{
		if (NMEA_Message_RDR[_Indexing] == ',')
			{Temp_Field_No++;}
		if (Temp_Field_No == _Field_No)
			{return _Indexing+1;}
		_Indexing++;
	}
	return 0XFF;
}

/*-----------------------------------------------------------*/
/*Convert From ASCII char to uint8_t DEC*/
uint8_t CHAR_TO_DEC(char _CAHR)
{
	(_CAHR <= 0x39) ? (_CAHR -= 0x30) : (_CAHR =0x00);
	return _CAHR;
}

/*-----------------------------------------------------------*/
/*Convert From ASCII HEX to uint8_t HEX*/
uint8_t HEX_TO_DEC(char _CAHR[2])
{
	(_CAHR[0] <= 0x39) ? (_CAHR[0] -= 0x30) : (_CAHR[0] -= 0x37);
	(_CAHR[1] <= 0x39) ? (_CAHR[1] -= 0x30) : (_CAHR[1] -= 0x37);
	return (uint8_t) ((_CAHR[0]<<4) +  _CAHR[1]);
}

/*-----------------------------------------------------------*/
/*Convert From ASCII String to uint16_t DEC
 * Use only For NMEA Message*/
uint16_t Sting_TO_DEC(char _STRING[])
{
	uint8_t Indexing = 0;
	float _Power = 1;
	uint8_t _HEX = 0;
	uint16_t _DEC = 0;
	if(_STRING[Indexing] == ',' || _STRING[Indexing] == '*')
		{return 0;}
	while(1)
	{
		_DEC *= _Power;
		_HEX = CHAR_TO_DEC(_STRING[Indexing]);
		_DEC += (uint8_t) _HEX;
		_Power *= 10;
		Indexing++;
		//end of String
		if(_STRING[Indexing] == ',' || _STRING[Indexing] == '*')
			{break;}
	}
	return _DEC;
}

/*-----------------------------------------------------------*/
/*Convert From ASCII String to float
 * Use only For NMEA Message*/
uint8_t Sting_TO_Float(char _STRING[])
{
	uint8_t Indexing = 0;
	float _Power = 1;
	uint8_t _HEX;
	float _FLOAT;

	if(_STRING[Indexing] == ',' || _STRING[Indexing] == '*')
		{return 0;}

	while(_STRING[Indexing] != '.')
	{
		_FLOAT *= (float) _Power;
		_HEX = CHAR_TO_DEC(_STRING[Indexing]);
		_FLOAT += (float) _HEX;
		_Power *=10;
		Indexing++;
		//end of String
		if(_STRING[Indexing] == '.' || _STRING[Indexing] == '*')
			{break;}
	}
	_Power = 0.1;
	while(1)
	{
		_HEX = CHAR_TO_DEC(_STRING[Indexing]);
		_FLOAT += (float) _HEX * _Power;
		_Power /=(float) 10;
		Indexing++;
		//end of String
		if(_STRING[Indexing] == ',' || _STRING[Indexing] == 0x00)
			{break;}
	}
	return _FLOAT;
}

/*-----------------------------------------------------------*/
float stringToFloat(char* str)
{
    float result = 0.0;
    int sign = 1;
    int decimalFound = 0;
    float decimalPlace = 0.1;

    // Check for negative sign
    if (*str == '-') {
        sign = -1;
        str++;
    }

    // Iterate through each character of the string
    while (*str != ',') {
        // Check for decimal point
        if (*str == '.') {
            decimalFound = 1;
            str++;
            continue;
        }

        // Convert character to digit
        int digit = *str - '0';

        // If decimal point is found, adjust decimal place
        if (decimalFound) {
            result += digit * decimalPlace;
            decimalPlace *= 0.1;
        }
        else {
            result = result * 10 + digit;
        }

        str++;
    }

    return result * sign;
}

/*-----------------------------------------------------------*/
/*Get GPS Information From NMEA_Message_RDR[]
 * Latitude and longitude, with time of position fix and status
 * from GNGLL GPS Message
 * return GPS_TYPE Parameter
 * */
GPS_STATUS Get_GPS_GLL_Information(GPS_TYPE* Temp_info)
{
	uint8_t _Field_Index;
	GPS_STATUS _Status;
			/* Get Status
			 * V = Data invalid or receiver warning, A = Data valid*/
		_Field_Index = NMEA_Field_No_INDEX(6);
		Temp_info->Latitude.status = NMEA_Message_RDR[_Field_Index];
		Temp_info->Longitude.status = NMEA_Message_RDR[_Field_Index];
			/*if Status Data valid
			 * Get Latitude and longitude Information*/
		if (NMEA_Message_RDR[_Field_Index] == 'A')
			_Status.POS_Status = valid;
		else
			_Status.POS_Status = invalid;

		if (_Status.POS_Status == valid)
		{
			_Field_Index = NMEA_Field_No_INDEX(1);
			Temp_info->Latitude.Degree = (float) ( CHAR_TO_DEC(NMEA_Message_RDR[_Field_Index])*10 + CHAR_TO_DEC(NMEA_Message_RDR[_Field_Index+1]) );
			Temp_info->Latitude.Degree += Sting_TO_Float(&NMEA_Message_RDR[_Field_Index+2])/(float) 60;
			_Field_Index = NMEA_Field_No_INDEX(2);
			Temp_info->Latitude.indicator = NMEA_Message_RDR[_Field_Index];

			_Field_Index = NMEA_Field_No_INDEX(3);
			Temp_info->Longitude.Degree = (float) ( CHAR_TO_DEC(NMEA_Message_RDR[_Field_Index])*100 + CHAR_TO_DEC(NMEA_Message_RDR[_Field_Index+1])*10 + CHAR_TO_DEC(NMEA_Message_RDR[_Field_Index+2]) );
			Temp_info->Longitude.Degree += Sting_TO_Float(&NMEA_Message_RDR[_Field_Index+3])/(float) 60;
			_Field_Index = NMEA_Field_No_INDEX(4);
			Temp_info->Longitude.indicator = NMEA_Message_RDR[_Field_Index];
		}

		/*Get time Information*/
		_Field_Index = NMEA_Field_No_INDEX(5);
		if (NMEA_Message_RDR[_Field_Index] != ',')
			_Status.TIME_Status = valid;
		else
			_Status.TIME_Status = invalid;

		if (_Status.TIME_Status == valid)
		{
			Temp_info->UTC_TIME.hrs = ( CHAR_TO_DEC(NMEA_Message_RDR[_Field_Index])*10 + CHAR_TO_DEC(NMEA_Message_RDR[_Field_Index+1]) );
			_Field_Index +=2;
			Temp_info->UTC_TIME.mint = ( CHAR_TO_DEC(NMEA_Message_RDR[_Field_Index])*10 + CHAR_TO_DEC(NMEA_Message_RDR[_Field_Index+1]) );
			_Field_Index +=2;
			Temp_info->UTC_TIME.sec = ( CHAR_TO_DEC(NMEA_Message_RDR[_Field_Index])*10 + CHAR_TO_DEC(NMEA_Message_RDR[_Field_Index+1]) );
		}
		/*Get Positioning mode Information
		 * Possible values for posMode: N = No fix, E = Estimated/Dead reckoning fix, A = Autonomous GNSS fix,
		 * D = Differential GNSS fix, F = RTK float, R = RTK fixed*/
		_Field_Index = NMEA_Field_No_INDEX(7);
		Temp_info->posMode = NMEA_Message_RDR[_Field_Index];

	return _Status;
}

/*-----------------------------------------------------------*/
/*Get GPS Information From NMEA_Message_RDR[]
 * Time and Date
 * from GNZDA GPS Message
 * return on GPS_TYPE* Parameter
 * */
GPS_STATUS Get_GPS_ZDA_Information(GPS_TYPE* Temp_info)
{
	uint8_t _Field_Index;
	GPS_STATUS _Status;
	_Field_Index = NMEA_Field_No_INDEX(1);

	if (NMEA_Message_RDR[_Field_Index] != ',')
		_Status.TIME_Status = valid;
	else
		_Status.TIME_Status = invalid;

	if (_Status.TIME_Status == valid)
	{
		Temp_info->UTC_TIME.hrs = ( CHAR_TO_DEC(NMEA_Message_RDR[_Field_Index])*10 + CHAR_TO_DEC(NMEA_Message_RDR[_Field_Index+1]) );
		_Field_Index +=2;
		Temp_info->UTC_TIME.mint = ( CHAR_TO_DEC(NMEA_Message_RDR[_Field_Index])*10 + CHAR_TO_DEC(NMEA_Message_RDR[_Field_Index+1]) );
		_Field_Index +=2;
		Temp_info->UTC_TIME.sec = ( CHAR_TO_DEC(NMEA_Message_RDR[_Field_Index])*10 + CHAR_TO_DEC(NMEA_Message_RDR[_Field_Index+1]) );

		_Field_Index = NMEA_Field_No_INDEX(2);
		Temp_info->UTC_TIME.day = ( CHAR_TO_DEC(NMEA_Message_RDR[_Field_Index])*10 + CHAR_TO_DEC(NMEA_Message_RDR[_Field_Index+1]) );

		_Field_Index = NMEA_Field_No_INDEX(3);
		Temp_info->UTC_TIME.month = ( CHAR_TO_DEC(NMEA_Message_RDR[_Field_Index])*10 + CHAR_TO_DEC(NMEA_Message_RDR[_Field_Index+1]) );

		_Field_Index = NMEA_Field_No_INDEX(4);
		Temp_info->UTC_TIME.year = ( CHAR_TO_DEC(NMEA_Message_RDR[_Field_Index])*1000 + CHAR_TO_DEC(NMEA_Message_RDR[_Field_Index+1])*100  \
									 + CHAR_TO_DEC(NMEA_Message_RDR[_Field_Index+2])*10 + CHAR_TO_DEC(NMEA_Message_RDR[_Field_Index+3]) );
	}
	return _Status;
}

/*-----------------------------------------------------------*/
/*Get GPS Information From NMEA_Message_RDR[]
 * Recommended Minimum data ,Time & Latitude
 * from GNZDA GPS Message
 * return on GPS_TYPE* Parameter
 * */
GPS_STATUS Get_GPS_RMC_Information(GPS_TYPE* Temp_info)
{
	uint8_t _Field_Index;
	GPS_STATUS _Status;

	/*Get time Information*/
	_Field_Index = NMEA_Field_No_INDEX(1);
	if (NMEA_Message_RDR[_Field_Index] != ',')
		_Status.TIME_Status = valid;
	else
		_Status.TIME_Status = invalid;

	if (_Status.TIME_Status == valid)
	{
		Temp_info->UTC_TIME.hrs = ( CHAR_TO_DEC(NMEA_Message_RDR[_Field_Index])*10 + CHAR_TO_DEC(NMEA_Message_RDR[_Field_Index+1]) );
		_Field_Index +=2;
		Temp_info->UTC_TIME.mint = ( CHAR_TO_DEC(NMEA_Message_RDR[_Field_Index])*10 + CHAR_TO_DEC(NMEA_Message_RDR[_Field_Index+1]) );
		_Field_Index +=2;
		Temp_info->UTC_TIME.sec = ( CHAR_TO_DEC(NMEA_Message_RDR[_Field_Index])*10 + CHAR_TO_DEC(NMEA_Message_RDR[_Field_Index+1]) );
	}
	/* Get Status
	 * V = Data invalid or receiver warning, A = Data valid*/
	_Field_Index = NMEA_Field_No_INDEX(2);
	Temp_info->Latitude.status = NMEA_Message_RDR[_Field_Index];

	if (NMEA_Message_RDR[_Field_Index] == 'A')
		_Status.POS_Status = valid;
	else
		_Status.POS_Status = invalid;

	if (_Status.POS_Status == valid)
	{
		_Field_Index = NMEA_Field_No_INDEX(3);
		Temp_info->Latitude.Degree = (float) ( CHAR_TO_DEC(NMEA_Message_RDR[_Field_Index])*10 + CHAR_TO_DEC(NMEA_Message_RDR[_Field_Index+1]) );
		Temp_info->Latitude.Degree += Sting_TO_Float(&NMEA_Message_RDR[_Field_Index+2])/(float) 60;
		_Field_Index = NMEA_Field_No_INDEX(4);
		Temp_info->Latitude.indicator = NMEA_Message_RDR[_Field_Index];

		_Field_Index = NMEA_Field_No_INDEX(5);
		Temp_info->Longitude.Degree = (float) ( CHAR_TO_DEC(NMEA_Message_RDR[_Field_Index])*100 + CHAR_TO_DEC(NMEA_Message_RDR[_Field_Index+1])*10 + CHAR_TO_DEC(NMEA_Message_RDR[_Field_Index+2]) );
		Temp_info->Longitude.Degree += Sting_TO_Float(&NMEA_Message_RDR[_Field_Index+3])/(float) 60;
		_Field_Index = NMEA_Field_No_INDEX(6);
		Temp_info->Longitude.indicator = NMEA_Message_RDR[_Field_Index];
	}
	//add
	_Field_Index = NMEA_Field_No_INDEX(7);
	if (NMEA_Message_RDR[_Field_Index] != ',')
			_Status.Speed_Status = valid;
		else
			_Status.Speed_Status = invalid;
	if (_Status.Speed_Status == valid)
	{
		speedKnot++;
		Temp_info->SPEED.speedKnot = stringToFloat(&NMEA_Message_RDR[_Field_Index]);/*Sting_TO_Float(&NMEA_Message_RDR[_Field_Index+7]);*/
	}
	//end add
	return _Status;
}

/*-----------------------------------------------------------*/
/*Get GPS Satellites in View Information From NMEA_Message_RDR[]
 * GNSS Satellites in View
 * return Satellites_Info* Parameter
 * */

Number_of_Satellites_t Get_GPS_GSV_Information(Satellites_Info* Temp_info)
{
	uint8_t _Field_Index;
	uint8_t numMsg;	//Number of messages, total number of GSV messages being output
	uint8_t msgNum;	//Number of this message
	uint8_t numSV;	//Number of satellites in view
	uint8_t _N=0;

	GSV_Satellites_STATUS = 0;

	_Field_Index = NMEA_Field_No_INDEX(1);
	numMsg = CHAR_TO_DEC(NMEA_Message_RDR[_Field_Index]);
	_Field_Index = NMEA_Field_No_INDEX(2);
	msgNum = CHAR_TO_DEC(NMEA_Message_RDR[_Field_Index]);
	_Field_Index = NMEA_Field_No_INDEX(3);
	numSV = CHAR_TO_DEC(NMEA_Message_RDR[_Field_Index])*10+CHAR_TO_DEC(NMEA_Message_RDR[_Field_Index+1]);

	if (numSV == 0) {return numSV;}

	_N = (msgNum-1)*4;
	for (uint8_t NSV= (msgNum-1)*4;NSV <= (msgNum-1)*4+4;NSV++)
	{
		if (_N >= NSV_IN_VIEW){break;}
		_Field_Index = NMEA_Field_No_INDEX(4+4*_N);
		Satellites[NSV].Satellite_ID = Sting_TO_DEC(&NMEA_Message_RDR[_Field_Index]);
		_Field_Index = NMEA_Field_No_INDEX(5+4*_N);
		Satellites[NSV].Elevation = Sting_TO_DEC(&NMEA_Message_RDR[_Field_Index]);
		_Field_Index = NMEA_Field_No_INDEX(6+4*_N);
		Satellites[NSV].Azimuth = Sting_TO_DEC(&NMEA_Message_RDR[_Field_Index]);
		_Field_Index = NMEA_Field_No_INDEX(7+4*_N);
		Satellites[NSV].Signal_strength = Sting_TO_DEC(&NMEA_Message_RDR[_Field_Index]);
		_N++;
		if (_N >= numSV ){break;}
	}

	if (_N == numSV && numMsg == msgNum)
	{GSV_Satellites_STATUS = 0x01;}
	return numSV;
}

/*-----------------------------------------------------------*/
/*Get GPS Satellites in View Information From NMEA_Message_RDR[]
 * GNSS Satellites in View
 * return Satellites_Info* Parameter
 * */
GPS_STATUS Get_GPS_GGA_GNS_Information(GPS_TYPE* Temp_info)
{
	uint8_t _Field_Index;
	GPS_STATUS _Status;


	/*Get time Information*/
	_Field_Index = NMEA_Field_No_INDEX(1);
	if (NMEA_Message_RDR[_Field_Index] != ',')
		_Status.TIME_Status = valid;
	else
		_Status.TIME_Status = invalid;

	if (_Status.TIME_Status == valid)
	{
		Temp_info->UTC_TIME.hrs = ( CHAR_TO_DEC(NMEA_Message_RDR[_Field_Index])*10 + CHAR_TO_DEC(NMEA_Message_RDR[_Field_Index+1]) );
		_Field_Index +=2;
		Temp_info->UTC_TIME.mint = ( CHAR_TO_DEC(NMEA_Message_RDR[_Field_Index])*10 + CHAR_TO_DEC(NMEA_Message_RDR[_Field_Index+1]) );
		_Field_Index +=2;
		Temp_info->UTC_TIME.sec = ( CHAR_TO_DEC(NMEA_Message_RDR[_Field_Index])*10 + CHAR_TO_DEC(NMEA_Message_RDR[_Field_Index+1]) );
	}


	_Field_Index = NMEA_Field_No_INDEX(2);
		/*if Status Data valid
		 * Get Latitude and longitude Information*/
	if (NMEA_Message_RDR[_Field_Index] != ',' && NMEA_Message_RDR[_Field_Index + 1] != ',' && NMEA_Message_RDR[_Field_Index + 2] != ',' && NMEA_Message_RDR[_Field_Index + 3] != ',')
		_Status.POS_Status = valid;
	else
		_Status.POS_Status = invalid;

	if (_Status.POS_Status == valid)
	{
		_Field_Index = NMEA_Field_No_INDEX(2);
		Temp_info->Latitude.Degree = (float) ( CHAR_TO_DEC(NMEA_Message_RDR[_Field_Index])*10 + CHAR_TO_DEC(NMEA_Message_RDR[_Field_Index+1]) );
		Temp_info->Latitude.Degree += Sting_TO_Float(&NMEA_Message_RDR[_Field_Index+2])/(float) 60;
		_Field_Index = NMEA_Field_No_INDEX(3);
		Temp_info->Latitude.indicator = NMEA_Message_RDR[_Field_Index];

		_Field_Index = NMEA_Field_No_INDEX(4);
		Temp_info->Longitude.Degree = (float) ( CHAR_TO_DEC(NMEA_Message_RDR[_Field_Index])*100 + CHAR_TO_DEC(NMEA_Message_RDR[_Field_Index+1])*10 + CHAR_TO_DEC(NMEA_Message_RDR[_Field_Index+2]) );
		Temp_info->Longitude.Degree += Sting_TO_Float(&NMEA_Message_RDR[_Field_Index+3])/(float) 60;
		_Field_Index = NMEA_Field_No_INDEX(5);
		Temp_info->Longitude.indicator = NMEA_Message_RDR[_Field_Index];
	}


	/*Get Height Information*/
	_Field_Index = NMEA_Field_No_INDEX(9);
	if (NMEA_Message_RDR[_Field_Index] != ',')
		_Status.Height_Status = valid;
	else
		_Status.Height_Status = invalid;

	if (_Status.Height_Status == valid)
	{
		Temp_info->HEIGHT.height = stringToFloat(&NMEA_Message_RDR[_Field_Index]);/*roundf(stringToFloat(&NMEA_Message_RDR[_Field_Index])* 10)/10;*/

	return _Status;
}
}

/*-----------------------------------------------------------*/
/*Get GPS Satellites in View Information From NMEA_Message_RDR[]
 * GNSS Satellites in View
 * return Satellites_Info* Parameter
 * */
GPS_STATUS Get_GPS_VTG_Information(GPS_TYPE* Temp_info)
{
	uint8_t _Field_Index;
	GPS_STATUS _Status;
	/*Get time Information*/
	_Field_Index = NMEA_Field_No_INDEX(5);
	if (NMEA_Message_RDR[_Field_Index] != ',')
		_Status.Speed_Status = valid;
	else
		_Status.Speed_Status = invalid;

	if (_Status.Speed_Status == valid)
	{
		Temp_info->SPEED.speedKnot = stringToFloat(&NMEA_Message_RDR[_Field_Index]);/*Sting_TO_Float(&NMEA_Message_RDR[_Field_Index+5]);*/
	}
	_Field_Index = NMEA_Field_No_INDEX(7);
	if (NMEA_Message_RDR[_Field_Index] != ',')
		_Status.Speed_Status = valid;
	else
		_Status.Speed_Status = invalid;

	if (_Status.Speed_Status == valid)
	{
		speedKm++;
		Temp_info->SPEED.speedKm = stringToFloat(&NMEA_Message_RDR[_Field_Index]);/*Sting_TO_Float(&NMEA_Message_RDR[_Field_Index]);*/
	}

	return _Status;
}

/*-----------------------------------------------------------*/
/*Get Talker_ID From NMEA Protocol
 * @ _NMEA_Message_Buffer[]
 * return Talker ID*/
GPS_TALKER_ID GPS_Get_Talker_ID ( char _NMEA_Message_Buffer[])
{
	if (_NMEA_Message_Buffer[1] == 'G')
	{
		switch (_NMEA_Message_Buffer[2])
		{
		case 'P' : return GP;break;
		case 'L' : return GL;break;
		case 'A' : return GA;break;
		case 'B' : return GB;break;
		case 'N' : return GN;break;
		}
	}
	else if (_NMEA_Message_Buffer[1] == 'P' && _NMEA_Message_Buffer[2] == 'U' && _NMEA_Message_Buffer[3] == 'B' && _NMEA_Message_Buffer[4] == 'X')
		return PUBX;
	else if (_NMEA_Message_Buffer[1] == 'E' && _NMEA_Message_Buffer[2] == 'I')
		return EI;
	return 0XFF;
}

/*-----------------------------------------------------------*/
/*Get Command From NMEA Protocol
 * @ _NMEA_Message_Buffer[]
 * return Message ID*/
GPS_MESSAGE_ID GPS_Get_Message_ID ( char _NMEA_Message_Buffer[])
{
	/*NMEA Standard Messages*/
	if (_NMEA_Message_Buffer[1] == 'G')
	{
		if (_NMEA_Message_Buffer[3] == 'D' && _NMEA_Message_Buffer[4] == 'M' && _NMEA_Message_Buffer[5] == 'T')
			return DMT;
   else if (_NMEA_Message_Buffer[3] == 'G' && _NMEA_Message_Buffer[4] == 'B' && _NMEA_Message_Buffer[5] == 'Q')
			return GBQ;
   else if (_NMEA_Message_Buffer[3] == 'G' && _NMEA_Message_Buffer[4] == 'B' && _NMEA_Message_Buffer[5] == 'S')
			return GBS;
   else if (_NMEA_Message_Buffer[3] == 'G' && _NMEA_Message_Buffer[4] == 'G' && _NMEA_Message_Buffer[5] == 'A')
			return GGA;
   else if (_NMEA_Message_Buffer[3] == 'G' && _NMEA_Message_Buffer[4] == 'L' && _NMEA_Message_Buffer[5] == 'L')
			return GLL;
   else if (_NMEA_Message_Buffer[3] == 'G' && _NMEA_Message_Buffer[4] == 'L' && _NMEA_Message_Buffer[5] == 'Q')
			return GLQ;
   else if (_NMEA_Message_Buffer[3] == 'G' && _NMEA_Message_Buffer[4] == 'N' && _NMEA_Message_Buffer[5] == 'Q')
			return GNQ;
   else if (_NMEA_Message_Buffer[3] == 'G' && _NMEA_Message_Buffer[4] == 'N' && _NMEA_Message_Buffer[5] == 'S')
			return GNS;
   else if (_NMEA_Message_Buffer[3] == 'G' && _NMEA_Message_Buffer[4] == 'P' && _NMEA_Message_Buffer[5] == 'Q')
			return GPQ;
   else if (_NMEA_Message_Buffer[3] == 'G' && _NMEA_Message_Buffer[4] == 'R' && _NMEA_Message_Buffer[5] == 'S')
			return GRS;
   else if (_NMEA_Message_Buffer[3] == 'G' && _NMEA_Message_Buffer[4] == 'S' && _NMEA_Message_Buffer[5] == 'A')
			return GSA;
   else if (_NMEA_Message_Buffer[3] == 'G' && _NMEA_Message_Buffer[4] == 'S' && _NMEA_Message_Buffer[5] == 'T')
			return GST;
   else if (_NMEA_Message_Buffer[3] == 'G' && _NMEA_Message_Buffer[4] == 'S' && _NMEA_Message_Buffer[5] == 'V')
			return GSV;
   else if (_NMEA_Message_Buffer[3] == 'R' && _NMEA_Message_Buffer[4] == 'M' && _NMEA_Message_Buffer[5] == 'C')
			return RMC;
   else if (_NMEA_Message_Buffer[3] == 'T' && _NMEA_Message_Buffer[4] == 'X' && _NMEA_Message_Buffer[5] == 'T')
			return TXT;
   else if (_NMEA_Message_Buffer[3] == 'V' && _NMEA_Message_Buffer[4] == 'L' && _NMEA_Message_Buffer[5] == 'W')
			return VLW;
   else if (_NMEA_Message_Buffer[3] == 'V' && _NMEA_Message_Buffer[4] == 'T' && _NMEA_Message_Buffer[5] == 'G')
			return VTG;
   else if (_NMEA_Message_Buffer[3] == 'Z' && _NMEA_Message_Buffer[4] == 'D' && _NMEA_Message_Buffer[5] == 'A')
			return ZDA;
   else return 0XFF;
	}
	/*NMEA PUBX Message*/
	else if (_NMEA_Message_Buffer[1] == 'P')
	{
		if (_NMEA_Message_Buffer[6] == '4' && _NMEA_Message_Buffer[7] == '1')
			return 0x41;
   else if (_NMEA_Message_Buffer[6] == '0' && _NMEA_Message_Buffer[7] == '0')
			return 0x00;
   else if (_NMEA_Message_Buffer[6] == '4' && _NMEA_Message_Buffer[7] == '0')
			return 0x40;
   else	if (_NMEA_Message_Buffer[6] == '0' && _NMEA_Message_Buffer[7] == '3')
			return 0x03;
   else	if (_NMEA_Message_Buffer[6] == '0' && _NMEA_Message_Buffer[7] == '4')
			return 0x04;
	}
	return 0XFF;
}

/*-----------------------------------------------------------*/
/*GPS SAM_M8Q Message Parser */

void GPS_IN_Action()
{
	NMEA_FILL_Incoming_Message();
	/*New Message Received*/
	if (Message_Incoming.Message_ID == 1)
	{
		/*GPS message received Complete
		* Ready to Start Terminate GPS Message
		* Filter incoming Message*/
		_FILTER = NMEA_Filter_Incoming_Messages(NMEA_Message_RDR);
		/*Message received Error*/
		if (_FILTER != Accept) {return (void)0;}

		/*Get Talker_ID & Message Command*/
//		GPS_TALKER_ID talker_ID = GPS_Get_Talker_ID(NMEA_Message_RDR);
		GPS_MESSAGE_ID message_ID = GPS_Get_Message_ID(NMEA_Message_RDR);
		switch(message_ID)
		{
			case GSV:
				GP_Number_Of_Satellites = Get_GPS_GSV_Information(&Satellites[0]);
				break;
			case GLL:
				_GLL=1;
				GPS_STATUS_INFO = Get_GPS_GLL_Information(&GPS_INFO);
				break;
			case RMC:
				_RMC=1;
				GPS_STATUS_INFO = Get_GPS_RMC_Information(&GPS_INFO);
				break;
			case ZDA:
				_ZDA=1;
				GPS_STATUS_INFO = Get_GPS_ZDA_Information(&GPS_INFO);
				break;
			case GGA:
			case GNS:
				if(message_ID == GGA)
					_GGA=1;
				else
					_GNS=1;
				GPS_STATUS_INFO = Get_GPS_GGA_GNS_Information(&GPS_INFO);
				break;
			case VTG:
				_VTG=1;
				GPS_STATUS_INFO = Get_GPS_VTG_Information(&GPS_INFO);
				break;
		}
		Message_Incoming.Message_ID = 0;
	}
}
/*-----------------------------------------------------------*/
/************************ (C) COPYRIGHT HEXABITZ *****END OF FILE****/
