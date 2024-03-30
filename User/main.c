/*
 BitzOS (BOS) V0.3.1 - Copyright (C) 2017-2024 Hexabitz
 All rights reserved

 File Name     : main.c
 Description   : Main program body.
 */
/* Includes ------------------------------------------------------------------*/
#include "BOS.h"

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Main function ------------------------------------------------------------*/
float longdegree, latdegree, speedknot, speedkm;
char longindicator,latindicator;
uint8_t hours, min, sec;

float height;
int main(void){

	Module_Init();		//Initialize Module &  BitzOS

	//Don't place your code here.
	for(;;){

		}
}

/*-----------------------------------------------------------*/

/* User Task */
void UserTask(void *argument){


	// put your code here, to run repeatedly.
	while(1){

		GPSHandel();
		GetPosition(&longdegree, &latdegree, &longindicator, &latindicator);
		GetUTC(&hours, &min, &sec);
		GetSpeed(&speedknot, &speedkm);
		GetHeight(&height);

	}
}

/*-----------------------------------------------------------*/
