/*
 BitzOS (BOS) V0.3.1 - Copyright (C) 2017-2024 Hexabitz
 All rights reserved

 File Name     : H1FR5.c
 Description   : Source code for module H1FR5.
 	 	 	 	 (Description_of_module)

(Description of Special module peripheral configuration):
>>
>>
>>

 */

/* Includes ------------------------------------------------------------------*/
#include "BOS.h"
#include "H1FR5_inputs.h"
#include "SAM_M10Q.h"

/* Define UART variables */
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;
UART_HandleTypeDef huart6;

/* Exported variables */
extern FLASH_ProcessTypeDef pFlash;
extern uint8_t numOfRecordedSnippets;

/* Module exported parameters ------------------------------------------------*/
module_param_t modParam[NUM_MODULE_PARAMS] ={{.paramPtr = NULL, .paramFormat =FMT_FLOAT, .paramName =""}};
#define MIN_MEMS_PERIOD_MS				100
#define MAX_MEMS_TIMEOUT_MS				0xFFFFFFFF
/* Private variables ---------------------------------------------------------*/
typedef Module_Status (*SampleMemsToPort)(uint8_t, uint8_t);
typedef void (*SampleMemsToString)(char *, size_t);
static bool stopStream = false;
/* Private function prototypes -----------------------------------------------*/
void ExecuteMonitor(void);
static portBASE_TYPE SampleGPSCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString);
static portBASE_TYPE StreamGPSCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString);
//static Module_Status StreamMemsToCLI(uint32_t period, uint32_t timeout, SampleMemsToString function);
static Module_Status StreamMemsToPort(uint8_t port, uint8_t module, uint32_t period, uint32_t timeout, SampleMemsToPort function);
static portBASE_TYPE StopStreamCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString);
/* Create CLI commands --------------------------------------------------------*/

/*-----------------------------------------------------------*/
/* CLI command structure : sample */
const CLI_Command_Definition_t SampleCommandDefinition = {
	(const int8_t *) "sample",
	(const int8_t *) "sample:\r\n Syntax: sample [position]/[utc]/[speed]/[height].\r\n\r\n",
	SampleGPSCommand,
	1
};
/*-----------------------------------------------------------*/
/* CLI command structure : stream */
const CLI_Command_Definition_t StreamCommandDefinition = {
	(const int8_t *) "stream",
	(const int8_t *) "stream:\r\n Syntax: stream [position]/[utc]/[speed]/[height] (period in ms) (time in ms) [port] [module].\r\n\r\n",
	StreamGPSCommand,
	-1
};
/*-----------------------------------------------------------*/
/* CLI command structure : stop */
const CLI_Command_Definition_t StopCommandDefinition = {
	(const int8_t *) "stop",
	(const int8_t *) "stop:\r\n Syntax: stop\r\n \
\tStop the current streaming of MEMS values. r\n\r\n",
	StopStreamCommand,
	0
};
/* ---------------------------------------------------------------------
 |								 Private Functions	                	|
 ----------------------------------------------------------------------- 
 */

/**
 * @brief  System Clock Configuration
 *         The system Clock is configured as follow : 
 *            System Clock source            = PLL (HSE)
 *            SYSCLK(Hz)                     = 48000000
 *            HCLK(Hz)                       = 48000000
 *            AHB Prescaler                  = 1
 *            APB1 Prescaler                 = 1
 *            HSE Frequency(Hz)              = 8000000
 *            PREDIV                         = 1
 *            PLLMUL                         = 6
 *            Flash Latency(WS)              = 1
 * @param  None
 * @retval None
 */
void SystemClock_Config(void){
	  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
	  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

	  /** Configure the main internal regulator output voltage
	  */
	  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);
	  /** Initializes the RCC Oscillators according to the specified parameters
	  * in the RCC_OscInitTypeDef structure.
	  */
	  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
	  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
	  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
	  RCC_OscInitStruct.PLL.PLLN = 12;
	  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
	  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
	 HAL_RCC_OscConfig(&RCC_OscInitStruct);

	  /** Initializes the CPU, AHB and APB buses clocks
	  */
	  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
	                              |RCC_CLOCKTYPE_PCLK1;
	  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

	 HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1);

	  /** Initializes the peripherals clocks
	  */
	  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_USART2;
	  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
	  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
	  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);


	  HAL_NVIC_SetPriority(SysTick_IRQn,0,0);
	
}

/*-----------------------------------------------------------*/


/* --- Save array topology and Command Snippets in Flash RO --- 
 */
uint8_t SaveToRO(void){
	BOS_Status result =BOS_OK;
	HAL_StatusTypeDef FlashStatus =HAL_OK;
	uint16_t add =8;
    uint16_t temp =0;
	uint8_t snipBuffer[sizeof(snippet_t) + 1] ={0};
	
	HAL_FLASH_Unlock();
	/* Erase RO area */
	FLASH_PageErase(FLASH_BANK_1,RO_START_ADDRESS);
	FlashStatus =FLASH_WaitForLastOperation((uint32_t ) HAL_FLASH_TIMEOUT_VALUE);
	FLASH_PageErase(FLASH_BANK_1,RO_MID_ADDRESS);
	//TOBECHECKED
	FlashStatus =FLASH_WaitForLastOperation((uint32_t ) HAL_FLASH_TIMEOUT_VALUE);
	if(FlashStatus != HAL_OK){
		return pFlash.ErrorCode;
	}
	else{
		/* Operation is completed, disable the PER Bit */
		CLEAR_BIT(FLASH->CR,FLASH_CR_PER);
	}
	
	/* Save number of modules and myID */
	if(myID){
		temp =(uint16_t )(N << 8) + myID;
		//HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,RO_START_ADDRESS,temp);
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,RO_START_ADDRESS,temp);
		//TOBECHECKED
		FlashStatus =FLASH_WaitForLastOperation((uint32_t ) HAL_FLASH_TIMEOUT_VALUE);
		if(FlashStatus != HAL_OK){
			return pFlash.ErrorCode;
		}
		else{
			/* If the program operation is completed, disable the PG Bit */
			CLEAR_BIT(FLASH->CR,FLASH_CR_PG);
		}
		
		/* Save topology */
		for(uint8_t i =1; i <= N; i++){
			for(uint8_t j =0; j <= MaxNumOfPorts; j++){
				if(array[i - 1][0]){

          	HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,RO_START_ADDRESS + add,array[i - 1][j]);
				 //HALFWORD 	//TOBECHECKED
					FlashStatus =FLASH_WaitForLastOperation((uint32_t ) HAL_FLASH_TIMEOUT_VALUE);
					if(FlashStatus != HAL_OK){
						return pFlash.ErrorCode;
					}
					else{
						/* If the program operation is completed, disable the PG Bit */
						CLEAR_BIT(FLASH->CR,FLASH_CR_PG);
						add +=8;
					}
				}
			}
		}
	}
	
	// Save Command Snippets
	int currentAdd = RO_MID_ADDRESS;
	for(uint8_t s =0; s < numOfRecordedSnippets; s++){
		if(snippets[s].cond.conditionType){
			snipBuffer[0] =0xFE;		// A marker to separate Snippets
			memcpy((uint32_t* )&snipBuffer[1],(uint8_t* )&snippets[s],sizeof(snippet_t));
			// Copy the snippet struct buffer (20 x numOfRecordedSnippets). Note this is assuming sizeof(snippet_t) is even.
			for(uint8_t j =0; j < (sizeof(snippet_t)/4); j++){
				HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,currentAdd,*(uint64_t* )&snipBuffer[j*8]);
				//HALFWORD
				//TOBECHECKED
				FlashStatus =FLASH_WaitForLastOperation((uint32_t ) HAL_FLASH_TIMEOUT_VALUE);
				if(FlashStatus != HAL_OK){
					return pFlash.ErrorCode;
				}
				else{
					/* If the program operation is completed, disable the PG Bit */
					CLEAR_BIT(FLASH->CR,FLASH_CR_PG);
					currentAdd +=8;
				}
			}
			// Copy the snippet commands buffer. Always an even number. Note the string termination char might be skipped
			for(uint8_t j =0; j < ((strlen(snippets[s].cmd) + 1)/4); j++){
				HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,currentAdd,*(uint64_t* )(snippets[s].cmd + j*4 ));
				//HALFWORD
				//TOBECHECKED
				FlashStatus =FLASH_WaitForLastOperation((uint32_t ) HAL_FLASH_TIMEOUT_VALUE);
				if(FlashStatus != HAL_OK){
					return pFlash.ErrorCode;
				}
				else{
					/* If the program operation is completed, disable the PG Bit */
					CLEAR_BIT(FLASH->CR,FLASH_CR_PG);
					currentAdd +=8;
				}
			}
		}
	}
	
	HAL_FLASH_Lock();
	
	return result;
}


/* --- Clear array topology in SRAM and Flash RO --- 
 */
uint8_t ClearROtopology(void){
	// Clear the array 
	memset(array,0,sizeof(array));
	N =1;
	myID =0;
	
	return SaveToRO();
}
/*-----------------------------------------------------------*/

/* --- Trigger ST factory bootloader update for a remote module.
 */
void remoteBootloaderUpdate(uint8_t src,uint8_t dst,uint8_t inport,uint8_t outport){

	uint8_t myOutport =0, lastModule =0;
	int8_t *pcOutputString;

	/* 1. Get route to destination module */
	myOutport =FindRoute(myID,dst);
	if(outport && dst == myID){ /* This is a 'via port' update and I'm the last module */
		myOutport =outport;
		lastModule =myID;
	}
	else if(outport == 0){ /* This is a remote update */
		if(NumberOfHops(dst)== 1)
		lastModule = myID;
		else
		lastModule = route[NumberOfHops(dst)-1]; /* previous module = route[Number of hops - 1] */
	}

	/* 2. If this is the source of the message, show status on the CLI */
	if(src == myID){
		/* Obtain the address of the output buffer.  Note there is no mutual
		 exclusion on this buffer as it is assumed only one command console
		 interface will be used at any one time. */
		pcOutputString =FreeRTOS_CLIGetOutputBuffer();

		if(outport == 0)		// This is a remote module update
			sprintf((char* )pcOutputString,pcRemoteBootloaderUpdateMessage,dst);
		else
			// This is a 'via port' remote update
			sprintf((char* )pcOutputString,pcRemoteBootloaderUpdateViaPortMessage,dst,outport);

		strcat((char* )pcOutputString,pcRemoteBootloaderUpdateWarningMessage);
		writePxITMutex(inport,(char* )pcOutputString,strlen((char* )pcOutputString),cmd50ms);
		Delay_ms(100);
	}

	/* 3. Setup my inport and outport for bootloader update */
	SetupPortForRemoteBootloaderUpdate(inport);
	SetupPortForRemoteBootloaderUpdate(myOutport);


	/* 5. Build a DMA stream between my inport and outport */
	StartScastDMAStream(inport,myID,myOutport,myID,BIDIRECTIONAL,0xFFFFFFFF,0xFFFFFFFF,false);
}

/*-----------------------------------------------------------*/

/* --- Setup a port for remote ST factory bootloader update:
 - Set baudrate to 57600
 - Enable even parity
 - Set datasize to 9 bits
 */
void SetupPortForRemoteBootloaderUpdate(uint8_t port){
	UART_HandleTypeDef *huart =GetUart(port);

	huart->Init.BaudRate =57600;
	huart->Init.Parity = UART_PARITY_EVEN;
	huart->Init.WordLength = UART_WORDLENGTH_9B;
	HAL_UART_Init(huart);

	/* The CLI port RXNE interrupt might be disabled so enable here again to be sure */
	__HAL_UART_ENABLE_IT(huart,UART_IT_RXNE);
}

/*-----------------------------------------------------------*/
/* --- H1FR5 module initialization.
 */
void Module_Peripheral_Init(void){

	/* Array ports */
	MX_USART1_UART_Init();
	MX_USART2_UART_Init();
	MX_USART3_UART_Init();
	MX_USART4_UART_Init();
	MX_USART5_UART_Init();
	MX_USART6_UART_Init();

	 //Circulating DMA Channels ON All Module
	 for(int i=1;i<=NumOfPorts;i++)
		{
		  if(GetUart(i)==&huart1)
				   { index_dma[i-1]=&(DMA1_Channel1->CNDTR); }
		  else if(GetUart(i)==&huart2)
				   { index_dma[i-1]=&(DMA1_Channel2->CNDTR); }
		  else if(GetUart(i)==&huart3)
				   { index_dma[i-1]=&(DMA1_Channel3->CNDTR); }
		  else if(GetUart(i)==&huart4)
				   { index_dma[i-1]=&(DMA1_Channel4->CNDTR); }
		  else if(GetUart(i)==&huart5)
				   { index_dma[i-1]=&(DMA1_Channel5->CNDTR); }
		  else if(GetUart(i)==&huart6)
				   { index_dma[i-1]=&(DMA1_Channel6->CNDTR); }
		}

}

/*-----------------------------------------------------------*/
/* --- H1FR5 message processing task.
 */

/* --- Get the port for a given UART. 
 */
uint8_t GetPort(UART_HandleTypeDef *huart){

	if(huart->Instance == USART4)
		return P1;
	else if(huart->Instance == USART2)
		return P2;
	else if(huart->Instance == USART3)
		return P3;
	else if(huart->Instance == USART1)
		return P4;
	else if(huart->Instance == USART6)
		return P5;
	
	return 0;
}

/*-----------------------------------------------------------*/

/* --- Register this module CLI Commands
 */
void RegisterModuleCLICommands(void){

	FreeRTOS_CLIRegisterCommand( &SampleCommandDefinition );
	FreeRTOS_CLIRegisterCommand( &StreamCommandDefinition );
	FreeRTOS_CLIRegisterCommand( &StopCommandDefinition);
}

/*-----------------------------------------------------------*/

static Module_Status PollingSleepCLISafe(uint32_t period)
{
	const unsigned DELTA_SLEEP_MS = 100; // milliseconds
	long numDeltaDelay =  period / DELTA_SLEEP_MS;
	unsigned lastDelayMS = period % DELTA_SLEEP_MS;

	while (numDeltaDelay-- > 0) {
		vTaskDelay(pdMS_TO_TICKS(DELTA_SLEEP_MS));

		// Look for ENTER key to stop the stream
		for (uint8_t chr=1 ; chr<MSG_RX_BUF_SIZE ; chr++)
		{
			if (UARTRxBuf[PcPort-1][chr] == '\r') {
				UARTRxBuf[PcPort-1][chr] = 0;
				return H1FR5_ERR_TERMINATED;
			}
		}

		if (stopStream)
			return H1FR5_ERR_TERMINATED;
	}

	vTaskDelay(pdMS_TO_TICKS(lastDelayMS));
	return H1FR5_OK;
}


/*-----------------------------------------------------------*/

//static Module_Status StreamMemsToCLI(uint32_t period, uint32_t timeout, SampleMemsToString function)
//{
//	Module_Status status = H1FR5_OK;
//	int8_t *pcOutputString = NULL;
//
//	if (period < MIN_MEMS_PERIOD_MS)
//		return H1FR5_ERR_WrongParams;
//
//	// TODO: Check if CLI is enable or not
//
//	if (period > timeout)
//		timeout = period;
//
//	long numTimes = timeout / period;
//	stopStream = false;
//
//	while ((numTimes-- > 0) || (timeout >= MAX_MEMS_TIMEOUT_MS)) {
//		pcOutputString = FreeRTOS_CLIGetOutputBuffer();
//		function((char *)pcOutputString, 100);
//
//
//		writePxMutex(PcPort, (char *)pcOutputString, strlen((char *)pcOutputString), cmd500ms, HAL_MAX_DELAY);
////		if (PollingSleepCLISafe(period) != H1FR5_OK)
////			break;
//		vTaskDelay(pdMS_TO_TICKS(period));
//		if (stopStream) {
//			status = H1FR5_ERR_TERMINATED;
//			break;
//		}
//	}
//
//	memset((char *) pcOutputString, 0, configCOMMAND_INT_MAX_OUTPUT_SIZE);
//  sprintf((char *)pcOutputString, "\r\n");
//	return status;
//}

/*-----------------------------------------------------------*/

static Module_Status StreamMemsToPort(uint8_t port, uint8_t module, uint32_t period, uint32_t timeout, SampleMemsToPort function)
{
	Module_Status status = H1FR5_OK;


	if (period < MIN_MEMS_PERIOD_MS)
		return H1FR5_ERR_WrongParams;
	if (port == 0)
		return H1FR5_ERR_WrongParams;
//	if (port == PcPort) // Check if CLI is not enabled at that port!
//		return H1FR5_ERR_BUSY;

	if (period > timeout)
		timeout = period;

	long numTimes = timeout / period;
	stopStream = false;

	while ((numTimes-- > 0) || (timeout >= MAX_MEMS_TIMEOUT_MS)) {
		function(port, module);

		vTaskDelay(pdMS_TO_TICKS(period));
		if (stopStream) {
			status = H1FR5_ERR_TERMINATED;
			break;
		}
	}
	return status;
}

/*-----------------------------------------------------------*/

/* -----------------------------------------------------------------------
 |								  APIs							          | 																 	|
/* -----------------------------------------------------------------------
 */

void GPSHandel(void)
{
	Incoming_Message_Handel();
}

/*-----------------------------------------------------------*/

Module_Status GetPosition(float * longdegree, float * latdegree, char *longindicator,char *latindicator)
{
	Module_Status status = H1FR5_OK;
	*longdegree = GPS_INFO.Longitude.Degree;
	*longindicator = GPS_INFO.Longitude.indicator;
	*latdegree = GPS_INFO.Latitude.Degree;
	*latindicator = GPS_INFO.Latitude.indicator;
	return status;
}

/*-----------------------------------------------------------*/

Module_Status GetUTC(uint8_t *hours, uint8_t *min, uint8_t *sec)
{
	Module_Status status = H1FR5_OK;
	*hours = GPS_INFO.UTC_TIME.hrs;
    *min = GPS_INFO.UTC_TIME.mint;
	*sec = GPS_INFO.UTC_TIME.sec;
	return status;
}

/*-----------------------------------------------------------*/

Module_Status GetSpeed(float *speedinch, float *speedkm)
{
	Module_Status status = H1FR5_OK;
	*speedinch = GPS_INFO.SPEED.speedKnot;
	*speedkm = GPS_INFO.SPEED.speedKm;
	return status;
}

/*-----------------------------------------------------------*/

Module_Status GetHeight(float *height)
{
	Module_Status status = H1FR5_OK;
	*height = GPS_INFO.HEIGHT.height;
	return status;
}

/*-----------------------------------------------------------*/

Module_Status SamplePositionToPort(uint8_t port,uint8_t module)
{
	Module_Status status = H1FR5_OK;
	float longdegree,latdegree;
	char longindicator,latindicator;
	static uint8_t temp[10];

	GetPosition(&longdegree,&latdegree,&longindicator,&latindicator);

	if(module == myID){
			temp[0] =*((__IO uint8_t* )(&longdegree) + 3);
			temp[1] =*((__IO uint8_t* )(&longdegree) + 2);
			temp[2] =*((__IO uint8_t* )(&longdegree) + 1);
			temp[3] =*((__IO uint8_t* )(&longdegree) + 0);
			temp[4] =*((__IO uint8_t* )(&latdegree) + 3);
			temp[5] =*((__IO uint8_t* )(&latdegree) + 2);
			temp[6] =*((__IO uint8_t* )(&latdegree) + 1);
			temp[7] =*((__IO uint8_t* )(&latdegree) + 0);
			temp[8] =*((__IO uint8_t* )(&longindicator));
			temp[9] =*((__IO uint8_t* )(&latindicator));

			writePxITMutex(port,(char* )&temp[0],10 * sizeof(uint8_t),10);
		}
		else{
			messageParams[0] =port;
			messageParams[1] =*((__IO uint8_t* )(&longdegree) + 3);
			messageParams[2] =*((__IO uint8_t* )(&longdegree) + 2);
			messageParams[3] =*((__IO uint8_t* )(&longdegree) + 1);
			messageParams[4] =*((__IO uint8_t* )(&longdegree) + 0);
			messageParams[5] =*((__IO uint8_t* )(&latdegree) + 3);
			messageParams[6] =*((__IO uint8_t* )(&latdegree) + 2);
			messageParams[7] =*((__IO uint8_t* )(&latdegree) + 1);
			messageParams[8] =*((__IO uint8_t* )(&latdegree) + 0);
			messageParams[9] =*((__IO uint8_t* )(&longindicator) + 0);
			messageParams[10] =*((__IO uint8_t* )(&latindicator) + 0);

			SendMessageToModule(module,CODE_PORT_FORWARD,11);
		}
	return status;
}

/*-----------------------------------------------------------*/

Module_Status SampleUTCToPort(uint8_t port,uint8_t module)
{
	Module_Status status = H1FR5_OK;
	uint8_t hours,min,sec;
	static uint8_t temp[3];

	GetUTC(&hours, &min, &sec);

	if(module == myID){
			temp[0] = hours;
			temp[1] = min;
			temp[2] = sec;

			writePxITMutex(port,(char* )&temp[0],3 * sizeof(uint8_t),10);
		}
		else{
			messageParams[0] = port;
			messageParams[1] = hours;
			messageParams[2] = min;
			messageParams[3] = sec;

			SendMessageToModule(module,CODE_PORT_FORWARD,4);
		}
	return status;
}

/*-----------------------------------------------------------*/

Module_Status SampleSpeedToPort(uint8_t port,uint8_t module)
{
	Module_Status status = H1FR5_OK;
	float speedinch,speedkm;
	static uint8_t temp[8];

	GetSpeed(&speedinch, &speedkm);

	if(module == myID){
			temp[0] =*((__IO uint8_t* )(&speedinch) + 3);
			temp[1] =*((__IO uint8_t* )(&speedinch) + 2);
			temp[2] =*((__IO uint8_t* )(&speedinch) + 1);
			temp[3] =*((__IO uint8_t* )(&speedinch) + 0);
			temp[4] =*((__IO uint8_t* )(&speedkm) + 3);
			temp[5] =*((__IO uint8_t* )(&speedkm) + 2);
			temp[6] =*((__IO uint8_t* )(&speedkm) + 1);
			temp[7] =*((__IO uint8_t* )(&speedkm) + 0);

			writePxITMutex(port,(char* )&temp[0],8 * sizeof(uint8_t),10);
		}
		else{
			messageParams[0] =port;
			messageParams[1] =*((__IO uint8_t* )(&speedinch) + 3);
			messageParams[2] =*((__IO uint8_t* )(&speedinch) + 2);
			messageParams[3] =*((__IO uint8_t* )(&speedinch) + 1);
			messageParams[4] =*((__IO uint8_t* )(&speedinch) + 0);
			messageParams[5] =*((__IO uint8_t* )(&speedkm) + 3);
			messageParams[6] =*((__IO uint8_t* )(&speedkm) + 2);
			messageParams[7] =*((__IO uint8_t* )(&speedkm) + 1);
			messageParams[8] =*((__IO uint8_t* )(&speedkm) + 0);

			SendMessageToModule(module,CODE_PORT_FORWARD,9);
		}
	return status;
}

/*-----------------------------------------------------------*/

Module_Status SampleHeightToPort(uint8_t port,uint8_t module)
{
	Module_Status status = H1FR5_OK;
	float height;
	char longindicator,latindicator;
	static uint8_t temp[4];

	GetHeight(&height);

	if(module == myID){
			temp[0] =*((__IO uint8_t* )(&height) + 3);
			temp[1] =*((__IO uint8_t* )(&height) + 2);
			temp[2] =*((__IO uint8_t* )(&height) + 1);
			temp[3] =*((__IO uint8_t* )(&height) + 0);

			writePxITMutex(port,(char* )&temp[0],4 * sizeof(uint8_t),10);
		}
		else{
			messageParams[0] =port;
			messageParams[1] =*((__IO uint8_t* )(&height) + 3);
			messageParams[2] =*((__IO uint8_t* )(&height) + 2);
			messageParams[3] =*((__IO uint8_t* )(&height) + 1);
			messageParams[4] =*((__IO uint8_t* )(&height) + 0);

			SendMessageToModule(module,CODE_PORT_FORWARD,sizeof(float)+1);
		}
	return status;
}
/*-----------------------------------------------------------*/
void SamplePositionToString(char *cstring, size_t maxLen)
{
	float longdegree,latdegree;
	char longindicator,latindicator;
	GetPosition(&longdegree,&latdegree,&longindicator,&latindicator);
	snprintf(cstring, maxLen, "Longitude_Degree: %.3f (deg)\r\nLongitude_indicator: %c\r\nLatitude_Degree: %.3f (deg)\r\nLatitude_indicator: %c\r\n", longdegree, longindicator, latdegree, latindicator);
}
/*-----------------------------------------------------------*/
void SampleUTCToString(char *cstring, size_t maxLen)
{
	uint8_t hours,min,sec;
	GetUTC(&hours, &min, &sec);
	snprintf(cstring, maxLen, "hours: %d min: %d sec: %d\r\n", hours, min, sec);
}
/*-----------------------------------------------------------*/
void SampleSpeedToString(char *cstring, size_t maxLen)
{
	float speedknot,speedkm;
	GetSpeed(&speedknot, &speedkm);
	snprintf(cstring, maxLen, "speedkm: %.3f (km/h) speedinch: %.3f (knot)\r\n", speedkm, speedknot);
}
/*-----------------------------------------------------------*/
void SampleHeightToString(char *cstring, size_t maxLen)
{
	float height;
	GetHeight(&height);
	snprintf(cstring, maxLen, "height: %.1f (m)\r\n", height);
}
/*-----------------------------------------------------------*/

//Module_Status StreamPositionToCLI(uint32_t period, uint32_t timeout)
//{
//	return StreamMemsToCLI(period, timeout, SamplePositionToString);
//}
//
//Module_Status StreamUTCToCLI(uint32_t period, uint32_t timeout)
//{
//	return StreamMemsToCLI(period, timeout, SampleUTCToString);
//}
//
//Module_Status StreamSpeedToCLI(uint32_t period, uint32_t timeout)
//{
//	return StreamMemsToCLI(period, timeout, SampleSpeedToString);
//}
//
//Module_Status StreamHeightToCLI(uint32_t period, uint32_t timeout)
//{
//	return StreamMemsToCLI(period, timeout, SampleHeightToString);
//}

Module_Status StreamPositionToPort(uint8_t port, uint8_t module, uint32_t period, uint32_t timeout)
{
	return StreamMemsToPort(port, module, period, timeout, SamplePositionToPort);
}
/*-----------------------------------------------------------*/

Module_Status StreamUTCToPort(uint8_t port, uint8_t module, uint32_t period, uint32_t timeout)
{
	return StreamMemsToPort(port, module, period, timeout, SampleUTCToPort);
}
/*-----------------------------------------------------------*/

Module_Status StreamSpeedToPort(uint8_t port, uint8_t module, uint32_t period, uint32_t timeout)
{
	return StreamMemsToPort(port, module, period, timeout, SampleSpeedToPort);
}
/*-----------------------------------------------------------*/

Module_Status StreamHeightToPort(uint8_t port, uint8_t module, uint32_t period, uint32_t timeout)
{
	return StreamMemsToPort(port, module, period, timeout, SampleHeightToPort);
}
/*-----------------------------------------------------------*/
void stopStreamMems(void)
{
	stopStream = true;
}

/*-----------------------------------------------------------*/

Module_Status Module_MessagingTask(uint16_t code, uint8_t port, uint8_t src, uint8_t dst, uint8_t shift)
{
	Module_Status status = H1FR5_OK;

	switch (code)
	{
		case CODE_H1FR5_GET_POSITION:
		{
			status = SamplePositionToPort(cMessage[port-1][1+shift] ,cMessage[port-1][shift]);
			break;
		}
		case CODE_H1FR5_GET_UTC:
		{
			status = SampleUTCToPort(cMessage[port-1][1+shift] ,cMessage[port-1][shift]);
			break;
		}
		case CODE_H1FR5_GET_SPEED:
		{
			status = SampleSpeedToPort(cMessage[port-1][1+shift] ,cMessage[port-1][shift]);
			break;
		}
		case CODE_H1FR5_GET_HIEGHT:
		{
			status = SampleHeightToPort(cMessage[port-1][1+shift] ,cMessage[port-1][shift]);
			break;
		}

		default:
			status = H1FR5_ERR_UnknownMessage;
			break;
	}

	return status;
}

/* -----------------------------------------------------------------------
 |								Commands							      |
   -----------------------------------------------------------------------
 */

static portBASE_TYPE SampleGPSCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString)
{
	const char *const positionCmdName = "position";
	const char *const utcCmdName = "utc";
	const char *const speedCmdName = "speed";
	const char *const heightCmdName = "height";

	const char *pfuncName = NULL;
	portBASE_TYPE funcNameLen = 0;

	// Make sure we return something
	*pcWriteBuffer = '\0';

	pfuncName = (const char *)FreeRTOS_CLIGetParameter(pcCommandString, 1, &funcNameLen);

	if (pfuncName == NULL) {
		snprintf((char *)pcWriteBuffer, xWriteBufferLen, "Invalid Arguments\r\n");
		return pdFALSE;
	}

	do {
		if (!strncmp(pfuncName, positionCmdName, strlen(positionCmdName))) {
			SamplePositionToString((char *)pcWriteBuffer, xWriteBufferLen);

		} else if (!strncmp(pfuncName, utcCmdName, strlen(utcCmdName))) {
			SampleUTCToString((char *)pcWriteBuffer, xWriteBufferLen);


		} else if (!strncmp(pfuncName, speedCmdName, strlen(speedCmdName))) {
			SampleSpeedToString((char *)pcWriteBuffer, xWriteBufferLen);


		} else if (!strncmp(pfuncName, heightCmdName, strlen(heightCmdName))) {
			SampleHeightToString((char *)pcWriteBuffer, xWriteBufferLen);


		} else {
			snprintf((char *)pcWriteBuffer, xWriteBufferLen, "Invalid Arguments\r\n");
		}

		return pdFALSE;
	} while (0);

	snprintf((char *)pcWriteBuffer, xWriteBufferLen, "Error reading Sensor\r\n");
	return pdFALSE;
}

/*-----------------------------------------------------------*/

static bool StreamCommandParser(const int8_t *pcCommandString, const char **ppSensName, portBASE_TYPE *pSensNameLen,
														bool *pPortOrCLI, uint32_t *pPeriod, uint32_t *pTimeout, uint8_t *pPort, uint8_t *pModule)
{
	const char *pPeriodMSStr = NULL;
	const char *pTimeoutMSStr = NULL;

	portBASE_TYPE periodStrLen = 0;
	portBASE_TYPE timeoutStrLen = 0;

	const char *pPortStr = NULL;
	const char *pModStr = NULL;

	portBASE_TYPE portStrLen = 0;
	portBASE_TYPE modStrLen = 0;

	*ppSensName = (const char *)FreeRTOS_CLIGetParameter(pcCommandString, 1, pSensNameLen);
	pPeriodMSStr = (const char *)FreeRTOS_CLIGetParameter(pcCommandString, 2, &periodStrLen);
	pTimeoutMSStr = (const char *)FreeRTOS_CLIGetParameter(pcCommandString, 3, &timeoutStrLen);

	// At least 3 Parameters are required!
	if ((*ppSensName == NULL) || (pPeriodMSStr == NULL) || (pTimeoutMSStr == NULL))
		return false;

	// TODO: Check if Period and Timeout are integers or not!
	*pPeriod = atoi(pPeriodMSStr);
	*pTimeout = atoi(pTimeoutMSStr);
	*pPortOrCLI = true;

	pPortStr = (const char *)FreeRTOS_CLIGetParameter(pcCommandString, 4, &portStrLen);
	pModStr = (const char *)FreeRTOS_CLIGetParameter(pcCommandString, 5, &modStrLen);

	if ((pModStr == NULL) && (pPortStr == NULL))
		return true;
	if ((pModStr == NULL) || (pPortStr == NULL))	// If user has provided 4 Arguments.
		return false;

	*pPort = atoi(pPortStr);
	*pModule = atoi(pModStr);
	*pPortOrCLI = false;

	return true;
}


static portBASE_TYPE StreamGPSCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString)
{
	const char *const positionCmdName = "position";
	const char *const utcCmdName = "utc";
	const char *const speedCmdName = "speed";
	const char *const heightCmdName = "height";

	uint32_t period = 0;
	uint32_t timeout = 0;
	uint8_t port = 0;
	uint8_t module = 0;

	bool portOrCLI = true; // Port Mode => false and CLI Mode => true

	const char *pSensName = NULL;
	portBASE_TYPE sensNameLen = 0;

	// Make sure we return something
	*pcWriteBuffer = '\0';

	if (!StreamCommandParser(pcCommandString, &pSensName, &sensNameLen, &portOrCLI, &period, &timeout, &port, &module)) {
		snprintf((char *)pcWriteBuffer, xWriteBufferLen, "Invalid Arguments\r\n");
		return pdFALSE;
	}

	do {
		if (!strncmp(pSensName, positionCmdName, strlen(positionCmdName))) {
			if (portOrCLI) {
//				StreamPositionToCLI(period, timeout);

			} else {
				StreamPositionToPort(port, module, period, timeout);

			}

		} else if (!strncmp(pSensName, utcCmdName, strlen(utcCmdName))) {
			if (portOrCLI) {
//				StreamUTCToCLI(period, timeout);

			} else {
				StreamUTCToPort(port, module, period, timeout);

			}

		}
		else if (!strncmp(pSensName, speedCmdName, strlen(speedCmdName))) {
			if (portOrCLI) {
//				StreamSpeedToCLI(period, timeout);

			} else {
				StreamSpeedToPort(port, module, period, timeout);

			}

		} else if (!strncmp(pSensName, heightCmdName, strlen(heightCmdName))) {
			if (portOrCLI) {
//				StreamHeightToCLI(period, timeout);

			} else {
				StreamHeightToPort(port, module, period, timeout);

			}

		} else {
			snprintf((char *)pcWriteBuffer, xWriteBufferLen, "Invalid Arguments\r\n");
		}

		snprintf((char *)pcWriteBuffer, xWriteBufferLen, "\r\n");
		return pdFALSE;
	} while (0);

	snprintf((char *)pcWriteBuffer, xWriteBufferLen, "Error reading Sensor\r\n");
	return pdFALSE;
}

/*-----------------------------------------------------------*/
static portBASE_TYPE StopStreamCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString)
{
	// Make sure we return something
	pcWriteBuffer[0] = '\0';
	snprintf((char *)pcWriteBuffer, xWriteBufferLen, "Stopping Streaming MEMS...\r\n");

	stopStreamMems();
	return pdFALSE;
}


/*-----------------------------------------------------------*/
/************************ (C) COPYRIGHT HEXABITZ *****END OF FILE****/
