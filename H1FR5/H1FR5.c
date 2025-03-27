/*
 BitzOS (BOS) V0.3.6 - Copyright (C) 2017-2024 Hexabitz
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
uint8_t port1, module1;
uint8_t port2 ,module2,mode2,mode1;
uint32_t Numofsamples1 ,timeout1;
uint8_t port3 ,module3,mode3;
uint32_t Numofsamples2 ,timeout2;
uint32_t Numofsamples3 ,timeout3;
uint8_t flag ;
uint8_t cont ;
uint8_t tofMode ;
/* Module exported parameters ------------------------------------------------*/
#define MIN_MEMS_PERIOD_MS				100
#define MAX_MEMS_TIMEOUT_MS				0xFFFFFFFF
/* Private variables ---------------------------------------------------------*/
typedef Module_Status (*SampleMemsToPort)(uint8_t, uint8_t);
typedef void (*SampleMemsToString)(char *, size_t);
static bool stopStream = false;
TaskHandle_t GPSTaskHandle = NULL;
uint8_t tofMode ;

float H1FR5_longitude = 0.0f;
float H1FR5_latitude = 0.0f;
char H1FR5_longIndicator = 'E';  // Default: East
char H1FR5_latIndicator = 'N';   // Default: North
uint8_t H1FR5_hours = 0;
uint8_t H1FR5_minutes = 0;
uint8_t H1FR5_seconds = 0;
float H1FR5_speedInch = 0.0f;
float H1FR5_speedKm = 0.0f;
float H1FR5_height = 0.0f;

/* Exported Typedef */
module_param_t modParam[NUM_MODULE_PARAMS] = {
    {.paramPtr = &H1FR5_longitude, .paramFormat = FMT_FLOAT, .paramName = "longitude"},
    {.paramPtr = &H1FR5_latitude, .paramFormat = FMT_FLOAT, .paramName = "latitude"},
    {.paramPtr = &H1FR5_longIndicator, .paramFormat = FMT_INT8, .paramName = "longindicator"},
    {.paramPtr = &H1FR5_latIndicator, .paramFormat = FMT_INT8, .paramName = "latindicator"},
    {.paramPtr = &H1FR5_hours, .paramFormat = FMT_UINT8, .paramName = "utc_hours"},
    {.paramPtr = &H1FR5_minutes, .paramFormat = FMT_UINT8, .paramName = "utc_minutes"},
    {.paramPtr = &H1FR5_seconds, .paramFormat = FMT_UINT8, .paramName = "utc_seconds"},
    {.paramPtr = &H1FR5_speedInch, .paramFormat = FMT_FLOAT, .paramName = "speedinch"},
    {.paramPtr = &H1FR5_speedKm, .paramFormat = FMT_FLOAT, .paramName = "speedkm"},
    {.paramPtr = &H1FR5_height, .paramFormat = FMT_FLOAT, .paramName = "height"}
};


/* Private function prototypes -----------------------------------------------*/
static Module_Status StreamMemsToTerminal(uint32_t Numofsamples, uint32_t timeout,uint8_t Port, SampleMemsToString function);
static Module_Status StreamToBuf( float *buffer, uint32_t Numofsamples, uint32_t timeout,buffer_Data function);
void SampleToPositionString(char *cstring, size_t maxLen);
void SampleToUTCString(char *cstring, size_t maxLen);
void SampleToHeighString(char *cstring, size_t maxLen);
void SampleToSpeedString(char *cstring, size_t maxLen);
void GPS(void *argument);
void ExecuteMonitor(void);
void GPSHandel(void);
Module_Status SampleHeightToPort(uint8_t port,uint8_t module);
Module_Status SampleSpeedToPort(uint8_t port,uint8_t module);
Module_Status SampleUTCToPort(uint8_t port,uint8_t module);
Module_Status SamplePositionToPort(uint8_t port,uint8_t module);
static Module_Status StreamMemsToPort(uint8_t port, uint8_t module, uint32_t Numofsamples, uint32_t timeout, SampleMemsToPort function);
static portBASE_TYPE SampleGPSCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString);
static portBASE_TYPE StreamGPSCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString);
static Module_Status StreamMemsToCLI(uint32_t period, uint32_t timeout, SampleMemsToString function);
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
\tStop the current streaming of values. r\n\r\n",
	StopStreamCommand,
	0
};
/* ---------------------------------------------------------------------
 |								 Private Functions	                	|
 ----------------------------------------------------------------------- 
 */

/**
* @brief  System Clock Configuration
*         This function configures the system clock as follows:
*            - System Clock source            = PLL (HSE)
*            - SYSCLK(Hz)                     = 64000000
*            - HCLK(Hz)                       = 64000000
*            - AHB Prescaler                  = 1
*            - APB1 Prescaler                 = 1
*            - HSE Frequency(Hz)              = 8000000
*            - PLLM                           = 1
*            - PLLN                           = 16
*            - PLLP                           = 2
*            - Flash Latency(WS)              = 2
*            - Clock Source for UART1,UART2,UART3 = 16MHz (HSI)
* @param  None
* @retval None
*/
void SystemClock_Config(void){
   RCC_OscInitTypeDef RCC_OscInitStruct = {0};
   RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

   /** Configure the main internal regulator output voltage */
   HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

   /** Initializes the RCC Oscillators according to the specified parameters
    * in the RCC_OscInitTypeDef structure.
    */
   RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_HSE; // Enable both HSI and HSE oscillators
   RCC_OscInitStruct.HSEState = RCC_HSE_ON; // Enable HSE (External High-Speed Oscillator)
   RCC_OscInitStruct.HSIState = RCC_HSI_ON; // Enable HSI (Internal High-Speed Oscillator)
   RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1; // No division on HSI
   RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT; // Default calibration value for HSI
   RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON; // Enable PLL
   RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE; // Set PLL source to HSE
   RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1; // Prescaler for PLL input
   RCC_OscInitStruct.PLL.PLLN = 16; // Multiplication factor for PLL
   RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2; // PLLP division factor
   RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2; // PLLQ division factor
   RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2; // PLLR division factor
   HAL_RCC_OscConfig(&RCC_OscInitStruct);

   /** Initializes the CPU, AHB and APB buses clocks */
   RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1;
   RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK; // Select PLL as the system clock source
   RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1; // AHB Prescaler set to 1
   RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1; // APB1 Prescaler set to 1

   HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2); // Configure system clocks with flash latency of 2 WS
}

/*-----------------------------------------------------------*/

/* --- Save Command Topology in Flash RO --- */

uint8_t SaveTopologyToRO(void)
{
	HAL_StatusTypeDef flashStatus =HAL_OK;
	/* flashAdd is initialized with 8 because the first memory room in topology page
	 * is reserved for module's ID */
	uint16_t flashAdd = 8;
    uint16_t temp =0;

    /* Unlock the FLASH control register access */
	HAL_FLASH_Unlock();

	/* Erase Topology page */
	FLASH_PageErase(FLASH_BANK_2,TOPOLOGY_PAGE_NUM);

	/* Wait for an Erase operation to complete */
	flashStatus =FLASH_WaitForLastOperation((uint32_t ) HAL_FLASH_TIMEOUT_VALUE);

	if(flashStatus != HAL_OK){
		/* return FLASH error code */
		return pFlash.ErrorCode;
	}

	else{
		/* Operation is completed, disable the PER Bit */
		CLEAR_BIT(FLASH->CR,FLASH_CR_PER);
	}

	/* Save module's ID and topology */
	if(myID){

		/* Save module's ID */
		temp =(uint16_t )(N << 8) + myID;

		/* Save module's ID in Flash memory */
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,TOPOLOGY_START_ADDRESS,temp);

		/* Wait for a Write operation to complete */
		flashStatus =FLASH_WaitForLastOperation((uint32_t ) HAL_FLASH_TIMEOUT_VALUE);

		if(flashStatus != HAL_OK){
			/* return FLASH error code */
			return pFlash.ErrorCode;
		}

		else{
			/* If the program operation is completed, disable the PG Bit */
			CLEAR_BIT(FLASH->CR,FLASH_CR_PG);
		}

		/* Save topology */
		for(uint8_t row =1; row <= N; row++){
			for(uint8_t column =0; column <= MaxNumOfPorts; column++){
				/* Check the module serial number
				 * Note: there isn't a module has serial number 0
				 */
				if(array[row - 1][0]){
					/* Save each element in topology array in Flash memory */
					HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,TOPOLOGY_START_ADDRESS + flashAdd,array[row - 1][column]);
					/* Wait for a Write operation to complete */
					flashStatus =FLASH_WaitForLastOperation((uint32_t ) HAL_FLASH_TIMEOUT_VALUE);
					if(flashStatus != HAL_OK){
						/* return FLASH error code */
						return pFlash.ErrorCode;
					}
					else{
						/* If the program operation is completed, disable the PG Bit */
						CLEAR_BIT(FLASH->CR,FLASH_CR_PG);
						/* update new flash memory address */
						flashAdd += 8;
					}
				}
			}
		}
	}
	/* Lock the FLASH control register access */
	HAL_FLASH_Lock();
}

/*-----------------------------------------------------------*/

/* --- Save Command Snippets in Flash RO --- */

uint8_t SaveSnippetsToRO(void)
{
	HAL_StatusTypeDef FlashStatus =HAL_OK;
    uint8_t snipBuffer[sizeof(snippet_t) + 1] ={0};

    /* Unlock the FLASH control register access */
	HAL_FLASH_Unlock();
    /* Erase Snippets page */
	FLASH_PageErase(FLASH_BANK_2,SNIPPETS_PAGE_NUM);
	/* Wait for an Erase operation to complete */
	FlashStatus =FLASH_WaitForLastOperation((uint32_t ) HAL_FLASH_TIMEOUT_VALUE);

	if(FlashStatus != HAL_OK){
		/* return FLASH error code */
		return pFlash.ErrorCode;
	}
	else{
		/* Operation is completed, disable the PER Bit */
		CLEAR_BIT(FLASH->CR,FLASH_CR_PER);
	}

	/* Save Command Snippets */
	int currentAdd = SNIPPETS_START_ADDRESS;
	for(uint8_t index = 0; index < numOfRecordedSnippets; index++){
		/* Check if Snippet condition is true or false */
		if(snippets[index].cond.conditionType){
			/* A marker to separate Snippets */
			snipBuffer[0] =0xFE;
			memcpy((uint32_t* )&snipBuffer[1],(uint8_t* )&snippets[index],sizeof(snippet_t));
			/* Copy the snippet struct buffer (20 x numOfRecordedSnippets). Note this is assuming sizeof(snippet_t) is even */
			for(uint8_t j =0; j < (sizeof(snippet_t)/4); j++){
				HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,currentAdd,*(uint64_t* )&snipBuffer[j*8]);
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
			/* Copy the snippet commands buffer. Always an even number. Note the string termination char might be skipped */
			for(uint8_t j = 0; j < ((strlen(snippets[index].cmd) + 1)/4); j++){
				HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,currentAdd,*(uint64_t* )(snippets[index].cmd + j*4 ));
				FlashStatus =FLASH_WaitForLastOperation((uint32_t ) HAL_FLASH_TIMEOUT_VALUE);
				if(FlashStatus != HAL_OK){
					return pFlash.ErrorCode;
				}
				else{
					/* If the program operation is completed, disable the PG Bit */
					CLEAR_BIT(FLASH->CR,FLASH_CR_PG);
					currentAdd += 8;
				}
			}
		}
	}
	/* Lock the FLASH control register access */
	HAL_FLASH_Lock();
}

/*-----------------------------------------------------------*/

/* --- Clear array topology in SRAM and Flash RO --- */

uint8_t ClearROtopology(void){
	// Clear the array 
	memset(array,0,sizeof(array));
	N =1;
	myID =0;
	
	return SaveTopologyToRO();
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

	 xTaskCreate(GPS,(const char* ) "GPS",configMINIMAL_STACK_SIZE,NULL,osPriorityNormal - osPriorityIdle,&GPSTaskHandle);

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
/***************************************************************************/
/* This function is useful only for input (sensor) modules.
 * @brief: Samples a module parameter value based on parameter index.
 * @param paramIndex: Index of the parameter (1-based index).
 * @param value: Pointer to store the sampled float value.
 * @retval: Module_Status indicating success or failure.
 */
Module_Status GetModuleParameter(uint8_t paramIndex, float *value) {
    Module_Status status = BOS_OK;

    switch (paramIndex) {
        /* Sample Longitude */
        case 1:
            status = GetPosition(value, NULL, NULL, NULL);
            break;

        /* Sample Latitude */
        case 2:
            status = GetPosition(NULL, value, NULL, NULL);
            break;

        /* Sample Longitude Indicator */
        case 3: {
            char temp;
            status = GetPosition(NULL, NULL, &temp, NULL);
            if (status == BOS_OK)
                *value = (float)temp;
            break;
        }

        /* Sample Latitude Indicator */
        case 4: {
            char temp;
            status = GetPosition(NULL, NULL, NULL, &temp);
            if (status == BOS_OK)
                *value = (float)temp;
            break;
        }

        /* Sample UTC Hours */
        case 5: {
            uint8_t temp;
            status = GetUTC(&temp, NULL, NULL);
            if (status == BOS_OK)
                *value = (float)temp;
            break;
        }

        /* Sample UTC Minutes */
        case 6: {
            uint8_t temp;
            status = GetUTC(NULL, &temp, NULL);
            if (status == BOS_OK)
                *value = (float)temp;
            break;
        }

        /* Sample UTC Seconds */
        case 7: {
            uint8_t temp;
            status = GetUTC(NULL, NULL, &temp);
            if (status == BOS_OK)
                *value = (float)temp;
            break;
        }

        /* Sample Speed in Inches */
        case 8:
            status = GetSpeed(value, NULL);
            break;

        /* Sample Speed in Km */
        case 9:
            status = GetSpeed(NULL, value);
            break;

        /* Sample Height */
        case 10:
            status = GetHeight(value);
            break;

        /* Invalid parameter index */
        default:
            status = BOS_ERR_WrongParam;
            break;
    }

    return status;
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

static Module_Status PollingSleepCLISafe(uint32_t period, long Numofsamples)
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
				flag=1;
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

static Module_Status StreamMemsToCLI(uint32_t Numofsamples, uint32_t timeout, SampleMemsToString function)
{
	Module_Status status = H1FR5_OK;
	int8_t *pcOutputString = NULL;
	uint32_t period = timeout / Numofsamples;
	if (period < MIN_MEMS_PERIOD_MS)
		return H1FR5_ERR_WrongParams;

	// TODO: Check if CLI is enable or not
	for (uint8_t chr = 0; chr < MSG_RX_BUF_SIZE; chr++) {
			if (UARTRxBuf[PcPort - 1][chr] == '\r' ) {
				UARTRxBuf[PcPort - 1][chr] = 0;
			}
		}
	if (1 == flag) {
		flag = 0;
		static char *pcOKMessage = (int8_t*) "Stop stream !\n\r";
		writePxITMutex(PcPort, pcOKMessage, strlen(pcOKMessage), 10);
		return status;
	}
	if (period > timeout)
		timeout = period;

	long numTimes = timeout / period;
	stopStream = false;

	while ((numTimes-- > 0) || (timeout >= MAX_MEMS_TIMEOUT_MS)) {
		pcOutputString = FreeRTOS_CLIGetOutputBuffer();
		function((char *)pcOutputString, 100);


		writePxMutex(PcPort, (char *)pcOutputString, strlen((char *)pcOutputString), cmd500ms, HAL_MAX_DELAY);
		if (PollingSleepCLISafe(period,Numofsamples) != H1FR5_OK)
			break;
	}

	memset((char *) pcOutputString, 0, configCOMMAND_INT_MAX_OUTPUT_SIZE);
  sprintf((char *)pcOutputString, "\r\n");
	return status;
}

void GPS(void *argument) {


	/* Infinite loop */
	for (;;) {
		GPSHandel();
		/*  */
		switch (tofMode) {
		case STREAM_TO_PORT:

			switch (mode1) {
			case Position:
				StreamMemsToPort(port2, module2, Numofsamples2, timeout2,
						SamplePositionToPort);
				break;
			case UTC:
				StreamMemsToPort(port2, module2, Numofsamples2, timeout2,
						SampleUTCToPort);
				break;
			case Speed:
				StreamMemsToPort(port2, module2, Numofsamples2, timeout2,
						SampleSpeedToPort);
				break;
			case Heigh:
				StreamMemsToPort(port2, module2, Numofsamples2, timeout2,
						SampleHeightToPort);
				break;
			default:
				break;
			}

			break;

		case STREAM_TO_Terminal:

			switch (mode1) {
					case Position:
						StreamMemsToTerminal(Numofsamples3, timeout3, port3,SampleToPositionString);
						break;
					case UTC:
						StreamMemsToTerminal(Numofsamples3, timeout3, port3,
								SampleToUTCString);
						break;
					case Speed:
						StreamMemsToTerminal(Numofsamples3, timeout3, port3,
								SampleToSpeedString);
						break;
					case Heigh:
						StreamMemsToTerminal(Numofsamples3, timeout3, port3,
								SampleToHeighString);
						break;
					default:
						break;
					}



			break;
		default:
			break;
}

		taskYIELD();
	}

}
/*-----------------------------------------------------------*/

static Module_Status StreamMemsToPort(uint8_t port, uint8_t module, uint32_t Numofsamples, uint32_t timeout, SampleMemsToPort function)
{
	Module_Status status = H1FR5_OK;
	uint32_t period = timeout / Numofsamples;

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
	*longdegree = GPS_INFO.Longitude.Degree;// tol
	*longindicator = GPS_INFO.Longitude.indicator;
	*latdegree = GPS_INFO.Latitude.Degree;  // ug
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


	status=GetPosition(&longdegree,&latdegree,&longindicator,&latindicator);

	if (module == myID || module == 0){
			temp[0] =*((__IO uint8_t* )(&longdegree) + 0);
			temp[1] =*((__IO uint8_t* )(&longdegree) + 1);
			temp[2] =*((__IO uint8_t* )(&longdegree) + 2);
			temp[3] =*((__IO uint8_t* )(&longdegree) + 3);
			temp[4] =*((__IO uint8_t* )(&latdegree) + 0);
			temp[5] =*((__IO uint8_t* )(&latdegree) + 1);
			temp[6] =*((__IO uint8_t* )(&latdegree) + 2);
			temp[7] =*((__IO uint8_t* )(&latdegree) + 3);
			temp[8] =*((__IO uint8_t* )(&longindicator));
			temp[9] =*((__IO uint8_t* )(&latindicator));

			writePxITMutex(port,(char* )&temp[0],10 * sizeof(uint8_t),10);
		}
		else{
			if (H1FR5_OK == status)
					messageParams[1] = BOS_OK;
				else
					messageParams[1] = BOS_ERROR;
			messageParams[0] =FMT_FLOAT;
			messageParams[2] =2;
			messageParams[3] =*((__IO uint8_t* )(&longdegree) + 0);
			messageParams[4] =*((__IO uint8_t* )(&longdegree) + 1);
			messageParams[5] =*((__IO uint8_t* )(&longdegree) + 2);
			messageParams[6] =*((__IO uint8_t* )(&longdegree) + 3);
			messageParams[7] =*((__IO uint8_t* )(&latdegree) + 0);
			messageParams[8] =*((__IO uint8_t* )(&latdegree) + 1);
			messageParams[9] =*((__IO uint8_t* )(&latdegree) + 2);
			messageParams[10] =*((__IO uint8_t* )(&latdegree) + 3);


			SendMessageToModule(module,CODE_READ_RESPONSE,13);
		}
	return status;
}

/*-----------------------------------------------------------*/

Module_Status SampleUTCToPort(uint8_t port,uint8_t module)
{
	Module_Status status = H1FR5_OK;
	uint8_t hours,min,sec;
	static uint8_t temp[3];

	status=GetUTC(&hours, &min, &sec);

	if (module == myID || module == 0){
			temp[0] = hours;
			temp[1] = min;
			temp[2] = sec;

			writePxITMutex(port,(char* )&temp[0],3 * sizeof(uint8_t),10);
		}
		else{
			if (H1FR5_OK == status)
					messageParams[1] = BOS_OK;
				else
					messageParams[1] = BOS_ERROR;
			messageParams[0] = FMT_UINT8;
			messageParams[2] = 3;
			messageParams[3] = hours;
			messageParams[4] = min;
			messageParams[5] = sec;

			SendMessageToModule(module,CODE_READ_RESPONSE,6);
		}
	return status;
}

/*-----------------------------------------------------------*/

Module_Status SampleSpeedToPort(uint8_t port,uint8_t module)
{
	Module_Status status = H1FR5_OK;
	float speedinch,speedkm;
	static uint8_t temp[8];

	status=GetSpeed(&speedinch, &speedkm);

	if (module == myID || module == 0){
			temp[0] =*((__IO uint8_t* )(&speedinch) + 0);
			temp[1] =*((__IO uint8_t* )(&speedinch) + 1);
			temp[2] =*((__IO uint8_t* )(&speedinch) + 2);
			temp[3] =*((__IO uint8_t* )(&speedinch) + 3);
			temp[4] =*((__IO uint8_t* )(&speedkm) + 0);
			temp[5] =*((__IO uint8_t* )(&speedkm) + 1);
			temp[6] =*((__IO uint8_t* )(&speedkm) + 2);
			temp[7] =*((__IO uint8_t* )(&speedkm) + 3);

			writePxITMutex(port,(char* )&temp[0],8 * sizeof(uint8_t),10);
		}
		else{
			if (H1FR5_OK == status)
					messageParams[1] = BOS_OK;
				else
					messageParams[1] = BOS_ERROR;
			messageParams[0] =FMT_FLOAT;
			messageParams[2] =2;
			messageParams[3] =*((__IO uint8_t* )(&speedinch) + 0);
			messageParams[4] =*((__IO uint8_t* )(&speedinch) + 1);
			messageParams[5] =*((__IO uint8_t* )(&speedinch) + 2);
			messageParams[6] =*((__IO uint8_t* )(&speedinch) + 3);
			messageParams[7] =*((__IO uint8_t* )(&speedkm) + 0);
			messageParams[8] =*((__IO uint8_t* )(&speedkm) + 1);
			messageParams[9] =*((__IO uint8_t* )(&speedkm) + 2);
			messageParams[10] =*((__IO uint8_t* )(&speedkm) + 3);

			SendMessageToModule(module,CODE_READ_RESPONSE,11);
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

	status=GetHeight(&height);

	if (module == myID || module == 0){
			temp[0] =*((__IO uint8_t* )(&height) + 0);
			temp[1] =*((__IO uint8_t* )(&height) + 1);
			temp[2] =*((__IO uint8_t* )(&height) + 2);
			temp[3] =*((__IO uint8_t* )(&height) + 3);

			writePxITMutex(port,(char* )&temp[0],4 * sizeof(uint8_t),10);
		}
		else{
			if (H1FR5_OK == status)
					messageParams[1] = BOS_OK;
				else
					messageParams[1] = BOS_ERROR;
			messageParams[0] =FMT_FLOAT;
			messageParams[2] =1;
			messageParams[3] =*((__IO uint8_t* )(&height) + 0);
			messageParams[4] =*((__IO uint8_t* )(&height) + 1);
			messageParams[5] =*((__IO uint8_t* )(&height) + 2);
			messageParams[6] =*((__IO uint8_t* )(&height) + 3);

			SendMessageToModule(module,CODE_READ_RESPONSE,sizeof(float)+3);
		}
	return status;
}
/*-----------------------------------------------------------*/
Module_Status StreamToBuffer(float *buffer,buffer_Data function, uint32_t Numofsamples, uint32_t timeout)
{
	return StreamToBuf(buffer, Numofsamples, timeout, function);
}

/*-----------------------------------------------------------*/

static Module_Status StreamToBuf( float *buffer, uint32_t Numofsamples, uint32_t timeout,buffer_Data function)
 {
	Module_Status status = H1FR5_OK;
	uint32_t period = timeout / Numofsamples;
	if (period < MIN_MEMS_PERIOD_MS)
		return H1FR5_ERR_WrongParams;

	// TODO: Check if CLI is enable or not

	if (period > timeout)
		timeout = period;

	long numTimes = timeout / period;
	stopStream = false;

	while ((numTimes-- > 0) || (timeout >= MAX_MEMS_TIMEOUT_MS)) {
			float sample,sample2;
		    char D,D1;
		switch (function) {
			case longdegree_buf:
				GetPosition(&sample, &sample2, &D1, &D);
				break;
			case Speed_buf:
				GetSpeed(&sample2, &sample);
						break;
			case latdegree_buf:
				GetPosition(&sample2, &sample, &D1, &D);
						break;
			case Heigh_buf:
				GetHeight(&sample);
						break;
			default:
				break;
		}


		buffer[cont] = sample;
		cont++;

		vTaskDelay(pdMS_TO_TICKS(period));
		if (stopStream) {
			status = H1FR5_ERR_TERMINATED;
			break;
		}
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

Module_Status StreamPositionToCLI(uint32_t Numofsamples, uint32_t timeout)
{
	return StreamMemsToCLI(Numofsamples, timeout, SamplePositionToString);
}
/*-----------------------------------------------------------*/
Module_Status StreamUTCToCLI(uint32_t Numofsamples, uint32_t timeout)
{
	return StreamMemsToCLI(Numofsamples, timeout, SampleUTCToString);
}
/*-----------------------------------------------------------*/
Module_Status StreamSpeedToCLI(uint32_t Numofsamples, uint32_t timeout)
{
	return StreamMemsToCLI(Numofsamples, timeout, SampleSpeedToString);
}
/*-----------------------------------------------------------*/
Module_Status StreamHeightToCLI(uint32_t Numofsamples, uint32_t timeout)
{
	return StreamMemsToCLI(Numofsamples, timeout, SampleHeightToString);
}
/*-----------------------------------------------------------*/
Module_Status StreamToPort(uint8_t module,uint8_t port,All_Data function,  uint32_t Numofsamples, uint32_t timeout)
 {
	Module_Status status = H1FR5_OK;
	tofMode = STREAM_TO_PORT;
	port2 = port;
	module2 = module;
	Numofsamples2 = Numofsamples;
	timeout2 = timeout;
	mode1 = function;
	return status;
}
/*-----------------------------------------------------------*/
Module_Status StreamToTerminal(uint8_t port,All_Data function,uint32_t Numofsamples, uint32_t timeout)
 {
	Module_Status status = H1FR5_OK;
	tofMode = STREAM_TO_Terminal;
	port3 = port;
	Numofsamples3 = Numofsamples;
	timeout3 = timeout;
	mode1 = function;
	return status;

}
/*-----------------------------------------------------------*/
static Module_Status StreamMemsToTerminal(uint32_t Numofsamples, uint32_t timeout,uint8_t Port, SampleMemsToString function)
{
	Module_Status status = H1FR5_OK;
	int8_t *pcOutputString = NULL;
	uint32_t period = timeout / Numofsamples;
	if (period < MIN_MEMS_PERIOD_MS)
		return H1FR5_ERR_WrongParams;

	// TODO: Check if CLI is enable or not

	if (period > timeout)
		timeout = period;

	long numTimes = timeout / period;
	stopStream = false;

	while ((numTimes-- > 0) || (timeout >= MAX_MEMS_TIMEOUT_MS)) {
		pcOutputString = FreeRTOS_CLIGetOutputBuffer();
		function((char *)pcOutputString, 100);


		writePxMutex(Port, (char *)pcOutputString, strlen((char *)pcOutputString), cmd500ms, HAL_MAX_DELAY);
		if (PollingSleepCLISafe(period,Numofsamples) != H1FR5_OK)
			break;
	}

	memset((char *) pcOutputString, 0, configCOMMAND_INT_MAX_OUTPUT_SIZE);
  sprintf((char *)pcOutputString, "\r\n");

	return status;
}
/*-----------------------------------------------------------*/
void SampleToPositionString(char *cstring, size_t maxLen)
 {
	float longdegree, latdegree;
	char latindicator, longindicator;
	GetPosition(&longdegree, &latdegree, &longindicator, &latindicator);
	snprintf(cstring, maxLen,
			"GPS: longdegree: %.2f , latdegree: %.2f , latdegree: %c  , latindicator: %c \r\n",
			longdegree, latdegree, longindicator, latindicator);
}
/*-----------------------------------------------------------*/
void SampleToUTCString(char *cstring, size_t maxLen)
 {
	uint8_t hours, min, sec;

	GetUTC(&hours, &min, &sec);

	snprintf(cstring, maxLen, "GPS: hours: %d , min: %d , sec: %d   \r\n",
			hours, min, sec);

}
/*-----------------------------------------------------------*/
void SampleToSpeedString(char *cstring, size_t maxLen)
 {
	float speedinch, speedkm;

	GetSpeed(&speedinch, &speedkm);

	snprintf(cstring, maxLen, "GPS: speedinch: %.2f , speedkm: %.2f \r\n",
			speedinch, speedkm);

}
/*-----------------------------------------------------------*/
void SampleToHeighString(char *cstring, size_t maxLen)
 {
	float height;

	GetHeight(&height);

	snprintf(cstring, maxLen, "GPS: height: %.2f \r\n", height);

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

	uint32_t Numofsamples = 0;
	uint32_t timeout = 0;
	uint8_t port = 0;
	uint8_t module = 0;

	bool portOrCLI = true; // Port Mode => false and CLI Mode => true

	const char *pSensName = NULL;
	portBASE_TYPE sensNameLen = 0;

	// Make sure we return something
	*pcWriteBuffer = '\0';

	if (!StreamCommandParser(pcCommandString, &pSensName, &sensNameLen, &portOrCLI, &Numofsamples, &timeout, &port, &module)) {
		snprintf((char *)pcWriteBuffer, xWriteBufferLen, "Invalid Arguments\r\n");
		return pdFALSE;
	}

	do {
		if (!strncmp(pSensName, positionCmdName, strlen(positionCmdName))) {
			if (portOrCLI) {
				StreamPositionToCLI(Numofsamples, timeout);

			} else {

				StreamToPort(module, port, Numofsamples, timeout, Position);
			}

		} else if (!strncmp(pSensName, utcCmdName, strlen(utcCmdName))) {
			if (portOrCLI) {
				StreamUTCToCLI(Numofsamples, timeout);

			} else {

				StreamToPort(module, port, Numofsamples, timeout, UTC);
			}

		} else if (!strncmp(pSensName, speedCmdName, strlen(speedCmdName))) {
			if (portOrCLI) {
				StreamSpeedToCLI(Numofsamples, timeout);

			} else {

				StreamToPort(module, port, Numofsamples, timeout, Speed);
			}

		} else if (!strncmp(pSensName, heightCmdName, strlen(heightCmdName))) {
			if (portOrCLI) {
				StreamHeightToCLI(Numofsamples, timeout);

			} else {

				StreamToPort(module, port, Numofsamples, timeout, Heigh);
			}

		} else {
			snprintf((char*) pcWriteBuffer, xWriteBufferLen,
					"Invalid Arguments\r\n");
		}

		snprintf((char*) pcWriteBuffer, xWriteBufferLen, "\r\n");
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
