/*
 BitzOS (BOS) V0.4.0 - Copyright (C) 2017-2025 Hexabitz
 All rights reserved

 File Name     : H1FR5.c
 Description   : Source code for module H1FR5.
 	 	 	 	 (Description_of_module)

(Description of Special module peripheral configuration):
>>
>>
>>
 */

/* Includes ****************************************************************/
#include "BOS.h"
#include "H1FR5_inputs.h"
#include "SAM_M10Q.h"

/* Exported Typedef ******************************************************/
/* Define UART variables */
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;
UART_HandleTypeDef huart6;

All_Data PortFunction;
All_Data TerminalFunction;

TimerHandle_t xTimerStream = NULL;
TaskHandle_t GPSTaskHandle = NULL;

/* Private Variables *******************************************************/
uint8_t flag;
uint8_t tofMode;
uint8_t port1, module1;
uint8_t port2 ,module2,mode2,mode1;
uint8_t port3 ,module3,mode3;
uint32_t Numofsamples1 ,timeout1;
uint32_t Numofsamples2 ,timeout2;
uint32_t Numofsamples3 ,timeout3;

/* Streaming variables */
static bool stopStream = false;         /* Flag to indicate whether to stop streaming process */
uint8_t PortModule = 0u;                /* Module ID for the destination port */
uint8_t PortNumber = 0u;                /* Physical port number used for streaming */
uint8_t StreamMode = 0u;                /* Current active streaming mode (to port, terminal, etc.) */
uint8_t TerminalPort = 0u;              /* Port number used to output data to a terminal */
uint8_t StopeCliStreamFlag = 0u;        /* Flag to request stopping a CLI stream operation */
uint32_t SampleCount = 0u;              /* Counter to track the number of samples streamed */
uint32_t PortNumOfSamples = 0u;         /* Total number of samples to be sent through the port */
uint32_t TerminalNumOfSamples = 0u;     /* Total number of samples to be streamed to the terminal */

/* Global variables for sensor data used in ModuleParam */
char H1FR5_longIndicator = 'E';  // Default: East
char H1FR5_latIndicator = 'N';   // Default: North
uint8_t H1FR5_hours = 0;
uint8_t H1FR5_minutes = 0;
uint8_t H1FR5_seconds = 0;
float H1FR5_speedInch = 0.0f;
float H1FR5_speedKm = 0.0f;
float H1FR5_height = 0.0f;
float H1FR5_longitude = 0.0f;
float H1FR5_latitude = 0.0f;

/* Module Parameters */
ModuleParam_t ModuleParam[NUM_MODULE_PARAMS] = {
    {.ParamPtr = &H1FR5_longitude    , .ParamFormat = FMT_FLOAT , .ParamName = "longitude"},
    {.ParamPtr = &H1FR5_latitude     , .ParamFormat = FMT_FLOAT , .ParamName = "latitude"},
    {.ParamPtr = &H1FR5_longIndicator, .ParamFormat = FMT_INT8  , .ParamName = "longindicator"},
    {.ParamPtr = &H1FR5_latIndicator , .ParamFormat = FMT_INT8  , .ParamName = "latindicator"},
    {.ParamPtr = &H1FR5_hours        , .ParamFormat = FMT_UINT8 , .ParamName = "utc_hours"},
    {.ParamPtr = &H1FR5_minutes      , .ParamFormat = FMT_UINT8 , .ParamName = "utc_minutes"},
    {.ParamPtr = &H1FR5_seconds      , .ParamFormat = FMT_UINT8 , .ParamName = "utc_seconds"},
    {.ParamPtr = &H1FR5_speedInch    , .ParamFormat = FMT_FLOAT , .ParamName = "speedinch"},
    {.ParamPtr = &H1FR5_speedKm      , .ParamFormat = FMT_FLOAT , .ParamName = "speedkm"},
    {.ParamPtr = &H1FR5_height       , .ParamFormat = FMT_FLOAT , .ParamName = "height"}
};

/* Local Typedef related to stream functions */
typedef void (*SampleMemsToString)(char *, size_t);
typedef Module_Status (*SampleMemsToPort)(uint8_t, uint8_t);

/* Local Typedef related to stream functions */
typedef void (*SampleToString)(char*,size_t);
typedef void (*SampleToBuffer)(float *buffer);
/* Private function prototypes *********************************************/
uint8_t ClearROtopology(void);
void Module_Peripheral_Init(void);
void SetupPortForRemoteBootloaderUpdate(uint8_t port);
void RemoteBootloaderUpdate(uint8_t src,uint8_t dst,uint8_t inport,uint8_t outport);
Module_Status Module_MessagingTask(uint16_t code,uint8_t port,uint8_t src,uint8_t dst,uint8_t shift);

/* Local function prototypes ***********************************************/
void GPSHandel(void);
void GPS(void *argument);

/* Stream Functions */
void SampleToUTCString(char *cstring, size_t maxLen);
void SampleToHeighString(char *cstring, size_t maxLen);
void SampleToSpeedString(char *cstring, size_t maxLen);
void SampleToPositionString(char *cstring, size_t maxLen);


static Module_Status StreamMemsToCLI(uint32_t period, uint32_t timeout, SampleMemsToString function);
static Module_Status StreamToBuf(float *buffer,uint32_t Numofsamples,uint32_t timeout,SampleToBuffer function);

/* Create CLI commands *****************************************************/
static portBASE_TYPE SampleGPSCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString);
static portBASE_TYPE StreamGPSCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString);
static portBASE_TYPE StopStreamCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString);

void SampleHeightBuf(float *buffer);
void SampleSpeedBuf(float *buffer);
void SampleUtcBuf(float *buffer);
void SamplePositionBuf(float *buffer);

/* CLI command structure ***************************************************/
/* CLI command structure : sample */
const CLI_Command_Definition_t SampleCommandDefinition = {
	(const int8_t *) "sample",
	(const int8_t *) "sample:\r\n Syntax: sample [position]/[utc]/[speed]/[height].\r\n\r\n",
	SampleGPSCommand,
	1 /* One parameter is expected. */
};

/***************************************************************************/
/* CLI command structure : stream */
const CLI_Command_Definition_t StreamCommandDefinition = {
	(const int8_t *) "stream",
	(const int8_t *) "stream:\r\n Syntax: stream [position]/[utc]/[speed]/[height] (period in ms) (time in ms) [port] [module].\r\n\r\n",
	StreamGPSCommand,
	-1
};

/***************************************************************************/
/* CLI command structure : stop */
const CLI_Command_Definition_t StopCommandDefinition = {
	(const int8_t *) "stop",
	(const int8_t *) "stop:\r\n Syntax: Stop the current streaming of values. r\n\r\n",
	StopStreamCommand,
	0 /* Zero parameter is expected. */
};

/***************************************************************************/
/************************ Private function Definitions *********************/
/***************************************************************************/
/* @brief  System Clock Configuration
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
 */
void SystemClock_Config(void){
	RCC_OscInitTypeDef RCC_OscInitStruct ={0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct ={0};

	/** Configure the main internal regulator output voltage */
	HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

	/* Initializes the RCC Oscillators according to the specified parameters
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
	RCC_OscInitStruct.PLL.PLLN =16; // Multiplication factor for PLL
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2; // PLLP division factor
	RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2; // PLLQ division factor
	RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2; // PLLR division factor
	HAL_RCC_OscConfig(&RCC_OscInitStruct);

	/** Initializes the CPU, AHB and APB buses clocks */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK; // Select PLL as the system clock source
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1; // AHB Prescaler set to 1
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1; // APB1 Prescaler set to 1

	HAL_RCC_ClockConfig(&RCC_ClkInitStruct,FLASH_LATENCY_2); // Configure system clocks with flash latency of 2 WS
}

/***************************************************************************/
/* enable stop mode regarding only UART1 , UART2 , and UART3 */
BOS_Status EnableStopModebyUARTx(uint8_t port){

	UART_WakeUpTypeDef WakeUpSelection;
	UART_HandleTypeDef *huart =GetUart(port);

	if((huart->Instance == USART1) || (huart->Instance == USART2) || (huart->Instance == USART3)){

		/* make sure that no UART transfer is on-going */
		while(__HAL_UART_GET_FLAG(huart, USART_ISR_BUSY) == SET);

		/* make sure that UART is ready to receive */
		while(__HAL_UART_GET_FLAG(huart, USART_ISR_REACK) == RESET);

		/* set the wake-up event:
		 * specify wake-up on start-bit detection */
		WakeUpSelection.WakeUpEvent = UART_WAKEUP_ON_STARTBIT;
		HAL_UARTEx_StopModeWakeUpSourceConfig(huart,WakeUpSelection);

		/* Enable the UART Wake UP from stop mode Interrupt */
		__HAL_UART_ENABLE_IT(huart,UART_IT_WUF);

		/* enable MCU wake-up by LPUART */
		HAL_UARTEx_EnableStopMode(huart);

		/* enter STOP mode */
		HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON,PWR_STOPENTRY_WFI);
	}
	else
		return BOS_ERROR;

}

/***************************************************************************/
/* Enable standby mode regarding wake-up pins:
 * WKUP1: PA0  pin
 * WKUP4: PA2  pin
 * WKUP6: PB5  pin
 * WKUP2: PC13 pin
 * NRST pin
 *  */
BOS_Status EnableStandbyModebyWakeupPinx(WakeupPins_t wakeupPins){

	/* Clear the WUF FLAG */
	__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WUF);

	/* Enable the WAKEUP PIN */
	switch(wakeupPins){

		case PA0_PIN:
			HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN1); /* PA0 */
			break;

		case PA2_PIN:
			HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN4); /* PA2 */
			break;

		case PB5_PIN:
			HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN6); /* PB5 */
			break;

		case PC13_PIN:
			HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN2); /* PC13 */
			break;

		case NRST_PIN:
			/* do no thing*/
			break;
	}

	/* Enable SRAM content retention in Standby mode */
	HAL_PWREx_EnableSRAMRetention();

	/* Finally enter the standby mode */
	HAL_PWR_EnterSTANDBYMode();

	return BOS_OK;
}

/***************************************************************************/
/* Disable standby mode regarding wake-up pins:
 * WKUP1: PA0  pin
 * WKUP4: PA2  pin
 * WKUP6: PB5  pin
 * WKUP2: PC13 pin
 * NRST pin
 *  */
BOS_Status DisableStandbyModeWakeupPinx(WakeupPins_t wakeupPins){

	/* The standby wake-up is same as a system RESET:
	 * The entire code runs from the beginning just as if it was a RESET.
	 * The only difference between a reset and a STANDBY wake-up is that, when the MCU wakes-up,
	 * The SBF status flag in the PWR power control/status register (PWR_CSR) is set */
	if(__HAL_PWR_GET_FLAG(PWR_FLAG_SB) != RESET){
		/* clear the flag */
		__HAL_PWR_CLEAR_FLAG(PWR_FLAG_SB);

		/* Disable  Wake-up Pinx */
		switch(wakeupPins){

			case PA0_PIN:
				HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN1); /* PA0 */
				break;

			case PA2_PIN:
				HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN4); /* PA2 */
				break;

			case PB5_PIN:
				HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN6); /* PB5 */
				break;

			case PC13_PIN:
				HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN2); /* PC13 */
				break;

			case NRST_PIN:
				/* do no thing*/
				break;
		}

		IND_blink(1000);

	}
	else
		return BOS_OK;

}

/***************************************************************************/
/* Save Command Topology in Flash RO */
uint8_t SaveTopologyToRO(void){

	HAL_StatusTypeDef flashStatus =HAL_OK;

	/* flashAdd is initialized with 8 because the first memory room in topology page
	 * is reserved for module's ID */
	uint16_t flashAdd =8;
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
			for(uint8_t column =0; column <= MAX_NUM_OF_PORTS; column++){
				/* Check the module serial number
				 * Note: there isn't a module has serial number 0
				 */
				if(Array[row - 1][0]){
					/* Save each element in topology Array in Flash memory */
					HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,TOPOLOGY_START_ADDRESS + flashAdd,Array[row - 1][column]);
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
						flashAdd +=8;
					}
				}
			}
		}
	}
	/* Lock the FLASH control register access */
	HAL_FLASH_Lock();
}

/***************************************************************************/
/* Save Command Snippets in Flash RO */
uint8_t SaveSnippetsToRO(void){
	HAL_StatusTypeDef FlashStatus =HAL_OK;
	uint8_t snipBuffer[sizeof(Snippet_t) + 1] ={0};

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
	for(uint8_t index =0; index < NumOfRecordedSnippets; index++){
		/* Check if Snippet condition is true or false */
		if(Snippets[index].Condition.ConditionType){
			/* A marker to separate Snippets */
			snipBuffer[0] =0xFE;
			memcpy((uint32_t* )&snipBuffer[1],(uint8_t* )&Snippets[index],sizeof(Snippet_t));
			/* Copy the snippet struct buffer (20 x NumOfRecordedSnippets). Note this is assuming sizeof(Snippet_t) is even */
			for(uint8_t j =0; j < (sizeof(Snippet_t) / 4); j++){
				HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,currentAdd,*(uint64_t* )&snipBuffer[j * 8]);
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
			for(uint8_t j =0; j < ((strlen(Snippets[index].CMD) + 1) / 4); j++){
				HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,currentAdd,*(uint64_t* )(Snippets[index].CMD + j * 4));
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
	/* Lock the FLASH control register access */
	HAL_FLASH_Lock();
}

/***************************************************************************/
/* Clear Array topology in SRAM and Flash RO */
uint8_t ClearROtopology(void){
	/* Clear the Array */
	memset(Array,0,sizeof(Array));
	N =1;
	myID =0;
	
	return SaveTopologyToRO();
}

/***************************************************************************/
/* Trigger ST factory bootloader update for a remote module */
void RemoteBootloaderUpdate(uint8_t src,uint8_t dst,uint8_t inport,uint8_t outport){

	uint8_t myOutport =0, lastModule =0;
	int8_t *pcOutputString;

	/* 1. Get Route to destination module */
	myOutport =FindRoute(myID,dst);
	if(outport && dst == myID){ /* This is a 'via port' update and I'm the last module */
		myOutport =outport;
		lastModule =myID;
	}
	else if(outport == 0){ /* This is a remote update */
		if(NumberOfHops(dst)== 1)
		lastModule = myID;
		else
		lastModule = Route[NumberOfHops(dst)-1]; /* previous module = Route[Number of hops - 1] */
	}

	/* 2. If this is the source of the message, show status on the CLI */
	if(src == myID){
		/* Obtain the address of the output buffer.  Note there is no mutual
		 * exclusion on this buffer as it is assumed only one command console
		 * interface will be used at any one time. */
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

/***************************************************************************/
/* Setup a port for remote ST factory bootloader update:
 * Set baudrate to 57600
 * Enable even parity
 * Set datasize to 9 bits
 */
void SetupPortForRemoteBootloaderUpdate(uint8_t port){

	UART_HandleTypeDef *huart =GetUart(port);
	HAL_UART_DeInit(huart);
	huart->Init.Parity = UART_PARITY_EVEN;
	huart->Init.WordLength = UART_WORDLENGTH_9B;
	HAL_UART_Init(huart);

	/* The CLI port RXNE interrupt might be disabled so enable here again to be sure */
	__HAL_UART_ENABLE_IT(huart,UART_IT_RXNE);

}

/***************************************************************************/
/* H1FR5 module initialization */
void Module_Peripheral_Init(void) {

	/* Array ports */
	MX_USART1_UART_Init();
	MX_USART2_UART_Init();
	MX_USART3_UART_Init();
	MX_USART4_UART_Init();
	MX_USART6_UART_Init();

	UARTInitGPS();

	/* Circulating DMA Channels ON All Module */
	for (int i = 1; i <= NUM_OF_PORTS; i++) {
		if (GetUart(i) == &huart1) {
			dmaIndex [i - 1] = &(DMA1_Channel1->CNDTR);
		} else if (GetUart(i) == &huart2) {
			dmaIndex [i - 1] = &(DMA1_Channel2->CNDTR);
		} else if (GetUart(i) == &huart3) {
			dmaIndex [i - 1] = &(DMA1_Channel3->CNDTR);
		} else if (GetUart(i) == &huart4) {
			dmaIndex [i - 1] = &(DMA1_Channel4->CNDTR);
		} else if (GetUart(i) == &huart5) {
			dmaIndex [i - 1] = &(DMA1_Channel5->CNDTR);
		} else if (GetUart(i) == &huart6) {
			dmaIndex [i - 1] = &(DMA1_Channel6->CNDTR);
		}
	}

	xTaskCreate(GPS, (const char*) "GPS", configMINIMAL_STACK_SIZE, NULL, osPriorityNormal - osPriorityIdle,
			&GPSTaskHandle);
	xTimerStream =xTimerCreate("StreamTimer",pdMS_TO_TICKS(1000),pdTRUE,(void* )1,StreamTimeCallback);

}

/***************************************************************************/
/* H1FR5 message processing task */
Module_Status Module_MessagingTask(uint16_t code, uint8_t port, uint8_t src, uint8_t dst, uint8_t shift) {
	Module_Status status = H1FR5_OK;

	switch (code) {
	case CODE_H1FR5_GET_POSITION:
		status = SampleToPort(cMessage[port - 1][shift],cMessage[port - 1][1 + shift],POSITION);
		break;

	case CODE_H1FR5_GET_UTC:
		status = SampleToPort(cMessage[port - 1][shift],cMessage[port - 1][1 + shift],UTC);
		break;

	case CODE_H1FR5_GET_SPEED:
		status = SampleToPort(cMessage[port - 1][shift],cMessage[port - 1][1 + shift],SPEED);
		break;

	case CODE_H1FR5_GET_HIEGHT:
		status = SampleToPort(cMessage[port - 1][shift],cMessage[port - 1][1 + shift],HEIGHT);
		break;

	default:
		status = H1FR5_ERR_UNKNOWNMESSAGE;
		break;
	}

	return status;
}

/***************************************************************************/
/* Get the port for a given UART */
uint8_t GetPort(UART_HandleTypeDef *huart) {

	if (huart->Instance == USART4)
		return P1;
	else if (huart->Instance == USART2)
		return P2;
	else if (huart->Instance == USART3)
		return P3;
	else if (huart->Instance == USART1)
		return P4;
	else if (huart->Instance == USART6)
		return P5;

	return 0;
}

/***************************************************************************/
/* Register this module CLI Commands */
void RegisterModuleCLICommands(void) {
	FreeRTOS_CLIRegisterCommand(&SampleCommandDefinition);
	FreeRTOS_CLIRegisterCommand(&StreamCommandDefinition);
	FreeRTOS_CLIRegisterCommand(&StopCommandDefinition);
}

/***************************************************************************/
/* This functions is useful only for input (sensors) modules.
 * Samples a module parameter value based on parameter index.
 * paramIndex: Index of the parameter (1-based index).
 * value: Pointer to store the sampled float value.
 */
Module_Status GetModuleParameter(uint8_t paramIndex, float *value) {
    Module_Status status = BOS_OK;
    char temp1;
    uint8_t temp;

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
        case 3:
            status = GetPosition(NULL, NULL, &temp1, NULL);
            if (status == BOS_OK)
                *value = (float)temp1;
            break;

        /* Sample Latitude Indicator */
        case 4:
            status = GetPosition(NULL, NULL, NULL, &temp1);
            if (status == BOS_OK)
                *value = (float)temp1;
            break;

        /* Sample UTC Hours */
        case 5:
            status = GetUTC(&temp, NULL, NULL);
            if (status == BOS_OK)
                *value = (float)temp;
            break;

        /* Sample UTC Minutes */
        case 6:
            status = GetUTC(NULL, &temp, NULL);
            if (status == BOS_OK)
                *value = (float)temp;
            break;

        /* Sample UTC Seconds */
        case 7:
            status = GetUTC(NULL, NULL, &temp);
            if (status == BOS_OK)
                *value = (float)temp;
            break;

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

/***************************************************************************/
/****************************** Local Functions ****************************/
/***************************************************************************/
void GPS(void *argument) {

	/* Infinite loop */
	for (;;) {
		GPSHandel();
		/*  */
		switch (tofMode) {
		case STREAM_TO_PORT:

			switch (mode1) {
			case POSITION:
				StreamtoPort(module2, port2, POSITION, Numofsamples2, timeout2);
				break;
			case UTC:
				StreamtoPort(module2, port2, UTC, Numofsamples2, timeout2);
				break;
			case SPEED:
				StreamtoPort(module2, port2, SPEED, Numofsamples2, timeout2);
				break;
			case HEIGHT:
				StreamtoPort(module2, port2, HEIGHT, Numofsamples2, timeout2);
				break;
			default:
				break;
			}

			break;

		case STREAM_TO_Terminal:

			switch (mode1) {
			case POSITION:
				StreamToTerminal(port3, POSITION, Numofsamples3, timeout3);
				break;

			case UTC:
				StreamToTerminal(port3, UTC, Numofsamples3, timeout3);;
				break;

			case SPEED:
				StreamToTerminal(port3, SPEED, Numofsamples3, timeout3);;
				break;

			case HEIGHT:
				StreamToTerminal(port3, HEIGHT, Numofsamples3, timeout3);;
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

static Module_Status PollingSleepCLISafe(uint32_t period, long Numofsamples) {
	const unsigned DELTA_SLEEP_MS = 100; // milliseconds
	long numDeltaDelay = period / DELTA_SLEEP_MS;
	unsigned lastDelayMS = period % DELTA_SLEEP_MS;

	while (numDeltaDelay-- > 0) {
		vTaskDelay(pdMS_TO_TICKS(DELTA_SLEEP_MS));

		/* Look for ENTER key to stop the stream */
		for (uint8_t chr = 1; chr < MSG_RX_BUF_SIZE; chr++) {
			if (UARTRxBuf [pcPort - 1] [chr] == '\r') {
				UARTRxBuf [pcPort - 1] [chr] = 0;
				flag = 1;
				return H1FR5_ERR_TERMINATED;
			}
		}

		if (stopStream)
			return H1FR5_ERR_TERMINATED;
	}

	vTaskDelay(pdMS_TO_TICKS(lastDelayMS));
	return H1FR5_OK;
}

/***************************************************************************/
/* */
void GPSHandel(void) {
	Incoming_Message_Handel();
}

/*
 * brief: Samples data and exports it to a specified port.
 * param dstModule: The module number to export data from.
 * param dstPort: The port number to export data to.
 * param dataFunction: Function to sample data (e.g., ACC, GYRO, MAG, TEMP).
 * retval: of type Module_Status indicating the success or failure of the operation.
 */

Module_Status SampleToPort(uint8_t dstModule, uint8_t dstPort, All_Data dataFunction)
{
		Module_Status Status = H1FR5_OK;
	static uint8_t Temp[12] ={0}; /* Buffer for data transmission */
	float Height =0.0f;
	float SpeedInch =0.0f, SpeedKm =0.0f;
	float LongDegree =0.0f, LatDegree =0.0f;
	uint8_t LongIndicator,LatIndicator;
	uint8_t Hours, Minutes, Seconds;

	/* Check if the port and module ID are valid */
	if((dstPort == 0) && (dstModule == myID)){
		return H1FR5_ERR_WRONGPARAMS;
	}

	/* Sample and export data based on function type */
	switch(dataFunction){
		case HEIGHT:
			if(GetHeight(&Height) != H1FR5_OK){
				return H1FR5_ERROR;
			}

			if(dstModule == myID || dstModule == 0){
				/* LSB first */
				Temp[0] =(uint8_t )((*(uint32_t* )&Height) >> 0);
				Temp[1] =(uint8_t )((*(uint32_t* )&Height) >> 8);
				Temp[2] =(uint8_t )((*(uint32_t* )&Height) >> 16);
				Temp[3] =(uint8_t )((*(uint32_t* )&Height) >> 24);

				writePxITMutex(dstPort,(char* )&Temp[0],4 * sizeof(uint8_t),10);
			}
			else{
				/* LSB first */
				MessageParams[1] =(H1FR5_OK == Status) ?BOS_OK: BOS_ERROR;
				MessageParams[0] =FMT_FLOAT;
				MessageParams[2] =1;
				MessageParams[3] =(uint8_t )((*(uint32_t* )&Height) >> 0);
				MessageParams[4] =(uint8_t )((*(uint32_t* )&Height) >> 8);
				MessageParams[5] =(uint8_t )((*(uint32_t* )&Height) >> 16);
				MessageParams[6] =(uint8_t )((*(uint32_t* )&Height) >> 24);

				SendMessageToModule(dstModule,CODE_READ_RESPONSE,(sizeof(float) * 1) + 3);
			}
			break;

		case SPEED:
			if(GetSpeed(&SpeedInch, &SpeedKm) != H1FR5_OK){
				return H1FR5_ERROR;
			}

			if(dstModule == myID || dstModule == 0){
				/* LSB first */
				Temp[0] =(uint8_t )((*(uint32_t* )&SpeedInch) >> 0);
				Temp[1] =(uint8_t )((*(uint32_t* )&SpeedInch) >> 8);
				Temp[2] =(uint8_t )((*(uint32_t* )&SpeedInch) >> 16);
				Temp[3] =(uint8_t )((*(uint32_t* )&SpeedInch) >> 24);
				Temp[4] =(uint8_t )((*(uint32_t* )&SpeedKm) >> 0);
				Temp[5] =(uint8_t )((*(uint32_t* )&SpeedKm) >> 8);
				Temp[6] =(uint8_t )((*(uint32_t* )&SpeedKm) >> 16);
				Temp[7] =(uint8_t )((*(uint32_t* )&SpeedKm) >> 24);

				writePxITMutex(dstPort,(char* )&Temp[0],8 * sizeof(uint8_t),10);
			}
			else{
				/* LSB first */
				MessageParams[1] =(H1FR5_OK == Status) ?BOS_OK: BOS_ERROR;
				MessageParams[0] =FMT_FLOAT;
				MessageParams[2] =2;
				MessageParams[3] =(uint8_t )((*(uint32_t* )&SpeedInch) >> 0);
				MessageParams[4] =(uint8_t )((*(uint32_t* )&SpeedInch) >> 8);
				MessageParams[5] =(uint8_t )((*(uint32_t* )&SpeedInch) >> 16);
				MessageParams[6] =(uint8_t )((*(uint32_t* )&SpeedInch) >> 24);
				MessageParams[7] =(uint8_t )((*(uint32_t* )&SpeedKm) >> 0);
				MessageParams[8] =(uint8_t )((*(uint32_t* )&SpeedKm) >> 8);
				MessageParams[9] =(uint8_t )((*(uint32_t* )&SpeedKm) >> 16);
				MessageParams[10] =(uint8_t )((*(uint32_t* )&SpeedKm) >> 24);

				SendMessageToModule(dstModule,CODE_READ_RESPONSE,(sizeof(float) * 2) + 3);
			}
			break;

		case UTC:
			if(GetUTC(&Hours, &Minutes, &Seconds) != H1FR5_OK){
				return H1FR5_ERROR;
			}

			if(dstModule == myID || dstModule == 0){
				/* LSB first */
				Temp[0] = Hours;
				Temp[1] = Minutes;
				Temp[2] = Seconds;

				writePxITMutex(dstPort,(char* )&Temp[0],3 * sizeof(uint8_t),10);
			}
			else{
				/* LSB first */
				MessageParams[1] =(H1FR5_OK == Status) ?BOS_OK: BOS_ERROR;
				MessageParams[0] =FMT_INT32;
				MessageParams[2] =3;
				MessageParams[3] = Hours;
				MessageParams[4] = Minutes;
				MessageParams[5] = Seconds;

				SendMessageToModule(dstModule,CODE_READ_RESPONSE,(sizeof(uint8_t) * 3) + 3);
			}
			break;

		case POSITION:
			if(GetPosition(&LongDegree, &LatDegree, &LongIndicator, &LatIndicator) != H1FR5_OK){
				return H1FR5_ERROR;
			}

			if(dstModule == myID || dstModule == 0){
				/* LSB first */
				Temp[0] =(uint8_t )((*(uint32_t* )&LongDegree) >> 0);
				Temp[1] =(uint8_t )((*(uint32_t* )&LongDegree) >> 8);
				Temp[2] =(uint8_t )((*(uint32_t* )&LongDegree) >> 16);
				Temp[3] =(uint8_t )((*(uint32_t* )&LongDegree) >> 24);
				Temp[4] =(uint8_t )((*(uint32_t* )&LatDegree) >> 0);
				Temp[5] =(uint8_t )((*(uint32_t* )&LatDegree) >> 8);
				Temp[6] =(uint8_t )((*(uint32_t* )&LatDegree) >> 16);
				Temp[7] =(uint8_t )((*(uint32_t* )&LatDegree) >> 24);
				Temp[8] = LongIndicator;
				Temp[9] = LatIndicator;

				writePxITMutex(dstPort,(char* )&Temp[0],10 * sizeof(uint8_t),10);
			}
			else{
				/* LSB first */
				MessageParams[1] =(H1FR5_OK == Status) ?BOS_OK: BOS_ERROR;
				MessageParams[0] =FMT_FLOAT;
				MessageParams[2] =10;
				MessageParams[3] =(uint8_t )((*(uint32_t* )&LongDegree) >> 0);
				MessageParams[4] =(uint8_t )((*(uint32_t* )&LongDegree) >> 8);
				MessageParams[5] =(uint8_t )((*(uint32_t* )&LongDegree) >> 16);
				MessageParams[6] =(uint8_t )((*(uint32_t* )&LongDegree) >> 24);
				MessageParams[7] =(uint8_t )((*(uint32_t* )&LatDegree) >> 0);
				MessageParams[8] =(uint8_t )((*(uint32_t* )&LatDegree) >> 8);
				MessageParams[9] =(uint8_t )((*(uint32_t* )&LatDegree) >> 16);
				MessageParams[10] =(uint8_t )((*(uint32_t* )&LatDegree) >> 24);
				MessageParams[11] = LongIndicator;
				MessageParams[12] = LatIndicator;

				SendMessageToModule(dstModule,CODE_READ_RESPONSE,(sizeof(uint8_t) * 10) + 3);
			}
			break;

		default:
			return H1FR5_ERR_WRONGPARAMS;
	}

	/* Clear the temp buffer */
	memset(&Temp[0],0,sizeof(Temp));

	return Status;
}

/* Streams a single sensor data sample to the terminal.
 * dstPort: Port number to stream data to.
 * dataFunction: Function to sample data (e.g., ACC, GYRO, MAG, TEMP).
 */

Module_Status SampleToTerminal(uint8_t dstPort,All_Data dataFunction)
{
	Module_Status Status =H1FR5_OK; /* Initialize operation status as success */
	int8_t *PcOutputString = NULL; /* Pointer to CLI output buffer */
	uint32_t Period =0u; /* Calculated period for the operation */
	char CString[100] ={0}; /* Buffer for formatted output string */
	float Height =0.0f;
	float SpeedInch =0.0f, SpeedKm =0.0f;
	float LongDegree =0.0f, LatDegree =0.0f;
	uint8_t LongIndicator,LatIndicator;
	uint8_t Hours, Minutes, Seconds;

	/* Process data based on the requested sensor function */
	switch(dataFunction){
		case HEIGHT:
			/* Get the CLI output buffer for writing */
			PcOutputString =FreeRTOS_CLIGetOutputBuffer();
			/* Sample accelerometer data in G units */
			if(GetHeight(&Height) != H1FR5_OK){
				return H1FR5_ERROR; /* Return error if sampling fails */
			}
			/* Format accelerometer data into a string */
			snprintf(CString,50,"Height(m): %.2f\r\n",Height);
			/* Send the formatted string to the specified port */
			writePxMutex(dstPort,(char* )CString,strlen((char* )CString),cmd500ms,HAL_MAX_DELAY);
			break;

		case SPEED:
			/* Get the CLI output buffer for writing */
			PcOutputString =FreeRTOS_CLIGetOutputBuffer();
			/* Sample gyroscope data in degrees per second */
			if(GetSpeed(&SpeedInch, &SpeedKm) != H1FR5_OK){
				return H1FR5_ERROR; /* Return error if sampling fails */
			}
			/* Format gyroscope data into a string */
			snprintf(CString,50,"SpeedInch: %.2f, SpeedKm: %.2f\r\n",SpeedInch,SpeedKm);
			/* Send the formatted string to the specified port */
			writePxMutex(dstPort,(char* )CString,strlen((char* )CString),cmd500ms,HAL_MAX_DELAY);
			break;

		case UTC:
			/* Get the CLI output buffer for writing */
			PcOutputString =FreeRTOS_CLIGetOutputBuffer();
			/* Sample magnetometer data in milliGauss */
			if(GetUTC(&Hours, &Minutes, &Seconds) != H1FR5_OK){
				return H1FR5_ERROR; /* Return error if sampling fails */
			}
			/* Format magnetometer data into a string */
			snprintf(CString,50,"Hours: %d, Minutes: %d, Seconds: %d\r\n",Hours,Minutes,Seconds);
			/* Send the formatted string to the specified port */
			writePxMutex(dstPort,(char* )CString,strlen((char* )CString),cmd500ms,HAL_MAX_DELAY);
			break;

		case POSITION:
			/* Get the CLI output buffer for writing */
			PcOutputString =FreeRTOS_CLIGetOutputBuffer();
			/* Sample temperature data in Celsius */
			if(GetPosition(&LongDegree, &LatDegree, &LongIndicator, &LatIndicator) != H1FR5_OK){
				return H1FR5_ERROR; /* Return error if sampling fails */
			}
			/* Format temperature data into a string */
			snprintf(CString,100,"LongDegree: %0.2f, LongIndicator: %c, LatDegree: %0.2f, LatIndicator: %c\r\n",LongDegree,LongIndicator,LatDegree,LatIndicator);
			/* Send the formatted string to the specified port */
			writePxMutex(dstPort,(char* )CString,strlen((char* )CString),cmd500ms,HAL_MAX_DELAY);
			break;

		default:
			/* Return error for invalid sensor function */
			return H1FR5_ERR_WRONGPARAMS;
	}

	/* Return final status indicating success or prior error */
	return Status;
}

/*
 * brief: Streams data to the specified port and module with a given number of samples.
 * param targetModule: The target module to which data will be streamed.
 * param portNumber: The port number on the module.
 * param portFunction: Type of data that will be streamed (ACC, GYRO, MAG, or TEMP).
 * param numOfSamples: The number of samples to stream.
 * param streamTimeout: The interval (in milliseconds) between successive data transmissions.
 * retval: of type Module_Status indicating the success or failure of the operation.
 */
Module_Status StreamtoPort(uint8_t dstModule,uint8_t dstPort,All_Data dataFunction,uint32_t numOfSamples,uint32_t streamTimeout)
{
	Module_Status Status =H1FR5_OK;
	uint32_t SamplePeriod =0u;

	/* Check timer handle and timeout validity */
	if((NULL == xTimerStream) || (0 == streamTimeout) || (0 == numOfSamples))
		return H1FR5_ERROR; /* Assuming H0BR4_ERROR is defined in Module_Status */

	/* Set streaming parameters */
	StreamMode = STREAM_MODE_TO_PORT;
	PortModule =dstModule;
	PortNumber =dstPort;
	PortFunction =dataFunction;
	PortNumOfSamples =numOfSamples;

	/* Calculate the period from timeout and number of samples */
	SamplePeriod =streamTimeout / numOfSamples;

	/* Stop (Reset) the TimerStream if it's already running */
	if(xTimerIsTimerActive(xTimerStream)){
		if(pdFAIL == xTimerStop(xTimerStream,100))
			return H1FR5_ERROR;
	}

	/* Start the stream timer */
	if(pdFAIL == xTimerStart(xTimerStream,100))
		return H1FR5_ERROR;

	/* Update timer timeout - This also restarts the timer */
	if(pdFAIL == xTimerChangePeriod(xTimerStream,SamplePeriod,100))
		return H1FR5_ERROR;

	return Status;
}

/*
 * brief: Streams data to the specified terminal port with a given number of samples.
 * param targetPort: The port number on the terminal.
 * param dataFunction: Type of data that will be streamed (ACC, GYRO, MAG, or TEMP).
 * param numOfSamples: The number of samples to stream.
 * param streamTimeout: The interval (in milliseconds) between successive data transmissions.
 * retval: of type Module_Status indicating the success or failure of the operation.
 */

Module_Status StreamToTerminal(uint8_t dstPort,All_Data dataFunction,uint32_t numOfSamples,uint32_t streamTimeout)
{
	Module_Status Status =H1FR5_OK;
	uint32_t SamplePeriod =0u;
	/* Check timer handle and timeout validity */
	if((NULL == xTimerStream) || (0 == streamTimeout) || (0 == numOfSamples))
		return H1FR5_ERROR; /* Assuming H0BR4_ERROR is defined in Module_Status */

	/* Set streaming parameters */
	StreamMode = STREAM_MODE_TO_TERMINAL;
	TerminalPort =dstPort;
	TerminalFunction =dataFunction;
	TerminalNumOfSamples =numOfSamples;

	/* Calculate the period from timeout and number of samples */
	SamplePeriod =streamTimeout / numOfSamples;

	/* Stop (Reset) the TimerStream if it's already running */
	if(xTimerIsTimerActive(xTimerStream)){
		if(pdFAIL == xTimerStop(xTimerStream,100))
			return H1FR5_ERROR;
	}

	/* Start the stream timer */
	if(pdFAIL == xTimerStart(xTimerStream,100))
		return H1FR5_ERROR;

	/* Update timer timeout - This also restarts the timer */
	if(pdFAIL == xTimerChangePeriod(xTimerStream,SamplePeriod,100))
		return H1FR5_ERROR;

	return Status;
}

/*
 * @brief: Streams data to a buffer.
 * @param buffer: Pointer to the buffer where data will be stored.
 * @param function: Function to sample data (e.g., ACC, GYRO, MAG, TEMP).
 * @param Numofsamples: Number of samples to take.
 * @param timeout: Timeout period for the operation.
 * @retval: Module status indicating success or error.
 */

Module_Status StreamToBuffer(float *buffer,All_Data function, uint32_t Numofsamples, uint32_t timeout)
{
	switch(function){
		case HEIGHT:
			return StreamToBuf(buffer,Numofsamples,timeout,SampleHeightBuf);
			break;
		case SPEED:
			return StreamToBuf(buffer,Numofsamples,timeout,SampleSpeedBuf);
			break;
		case UTC:
			return StreamToBuf(buffer,Numofsamples,timeout,SampleUtcBuf);
			break;
		case POSITION:
			return StreamToBuf(buffer,Numofsamples,timeout,SamplePositionBuf);
			break;
		default:
			break;
	}
}

/* Callback function triggered by a timer to manage data streaming.
 * xTimerStream: Handle of the timer that triggered the callback.
 */
void StreamTimeCallback(TimerHandle_t xTimerStream)
{
	/* Increment sample counter */
	++SampleCount;

	/* Stream mode to port: Send samples to port */
	if(STREAM_MODE_TO_PORT == StreamMode){
		if((SampleCount <= PortNumOfSamples) || (0 == PortNumOfSamples)){
			SampleToPort(PortModule,PortNumber,PortFunction);
		}
		else{
			SampleCount =0;
			xTimerStop(xTimerStream,0);
		}
	}
	/* Stream mode to terminal: Export to terminal */
	else if(STREAM_MODE_TO_TERMINAL == StreamMode){
		if((SampleCount <= TerminalNumOfSamples) || (0 == TerminalNumOfSamples)){
			SampleToTerminal(TerminalPort,TerminalFunction);
		}
		else{
			SampleCount =0;
			xTimerStop(xTimerStream,0);
		}
	}
}


/***************************************************************************/
static Module_Status StreamToCLI(uint32_t Numofsamples, uint32_t timeout, SampleToString function) {
	Module_Status status = H1FR5_OK;
	int8_t *pcOutputString = NULL;
	uint32_t period = timeout / Numofsamples;

	if (period < MIN_MEMS_PERIOD_MS)
		return H1FR5_ERR_WRONGPARAMS;

	// TODO: Check if CLI is enable or not
	for (uint8_t chr = 0; chr < MSG_RX_BUF_SIZE; chr++) {
		if (UARTRxBuf [pcPort - 1] [chr] == '\r') {
			UARTRxBuf [pcPort - 1] [chr] = 0;
		}
	}
	if (1 == flag) {
		flag = 0;
		static char *pcOKMessage = (int8_t*) "Stop stream !\n\r";
		writePxITMutex(pcPort, pcOKMessage, strlen(pcOKMessage), 10);
		return status;
	}
	if (period > timeout)
		timeout = period;

	long numTimes = timeout / period;
	stopStream = false;

	while ((numTimes-- > 0) || (timeout >= MAX_MEMS_TIMEOUT_MS)) {
		pcOutputString = FreeRTOS_CLIGetOutputBuffer();
		function((char*) pcOutputString, 100);

		writePxMutex(pcPort, (char*) pcOutputString, strlen((char*) pcOutputString), cmd500ms, HAL_MAX_DELAY);
		if (PollingSleepCLISafe(period, Numofsamples) != H1FR5_OK)
			break;
	}

	memset((char*) pcOutputString, 0, configCOMMAND_INT_MAX_OUTPUT_SIZE);
	sprintf((char*) pcOutputString, "\r\n");

	return status;
}


/***************************************************************************/
/* Streams sensor data to a buffer.
 * buffer: Pointer to the buffer where data will be stored.
 * Numofsamples: Number of samples to take.
 * timeout: Timeout period for the operation.
 * function: Function pointer to the sampling function (e.g., SampleAccBuf, SampleGyroBuf).
 */
static Module_Status StreamToBuf(float *buffer,uint32_t Numofsamples,uint32_t timeout,SampleToBuffer function){
	Module_Status status =H1FR5_OK;
	uint16_t StreamIndex =0;
	uint32_t period =timeout / Numofsamples;

	/* Check if the calculated period is valid */
	if(period < MIN_PERIOD_MS)
		return H1FR5_ERR_WRONGPARAMS;

	stopStream = false;

	float Axis[4];
	/* Stream data to buffer */
	while((Numofsamples-- > 0) || (timeout >= MAX_TIMEOUT_MS)){
		if(function == SampleHeightBuf){
			float sample;
			function(&sample);
			buffer[StreamIndex] =sample;
			StreamIndex++;
		}
		else if(function == SampleSpeedBuf)
		{
			function(Axis);
			buffer[StreamIndex] =Axis[0];
			buffer[StreamIndex + 1] =Axis[1];
			StreamIndex +=2;
		}
	    else if(function == SamplePositionBuf)
	    {
			function(Axis);
			buffer[StreamIndex] =Axis[0];
			buffer[StreamIndex + 1] =Axis[1];
			buffer[StreamIndex + 2] =Axis[2];
			buffer[StreamIndex + 3] =Axis[3];
			StreamIndex +=4;
	    }
		else
		{
			function(Axis);
			buffer[StreamIndex] =Axis[0];
			buffer[StreamIndex + 1] =Axis[1];
			buffer[StreamIndex + 2] =Axis[2];
			StreamIndex +=3;
		}
		/* Delay for the specified period */
		vTaskDelay(pdMS_TO_TICKS(period));

		/* Check if streaming should be stopped */
		if(stopStream){
			status =H1FR5_ERR_TERMINATED;
			break;
		}
	}

	return status;
}

/***************************************************************************/
/* Samples accelerometer data into a buffer.
 * buffer: Pointer to the buffer where accelerometer data will be stored.
 */
void SamplePositionBuf(float *buffer){
	float Position[4];
	GetPosition(Position, Position+1, (char *)Position+2, (char *)Position+3);
	buffer[0] =Position[0];
	buffer[1] =Position[1];
	buffer[2] =Position[2];
	buffer[3] =Position[3];
}

/***************************************************************************/
/* Samples gyroscope data into a buffer.
 * buffer: Pointer to the buffer where gyroscope data will be stored.
 */
void SampleUtcBuf(float *buffer){
	float Utc[3];
	GetUTC((uint8_t *)Utc, (uint8_t *)Utc+1, (uint8_t *)Utc+2);
	buffer[0] =Utc[0];
	buffer[1] =Utc[1];
	buffer[2] =Utc[2];
}

/***************************************************************************/
/* Samples magnetometer data into a buffer.
 * buffer: Pointer to the buffer where magnetometer data will be stored.
 */
void SampleSpeedBuf(float *buffer){
	float Speed[2];
	GetSpeed(Speed, Speed+1);
	buffer[0] =Speed[0];
	buffer[1] =Speed[1];
}

/***************************************************************************/
/* Samples temperature data into a buffer.
 * buffer: Pointer to the buffer where temperature data will be stored.
 */
void SampleHeightBuf(float *buffer){
	float Height;
	GetHeight(&Height);
	*buffer =Height;
}




/***************************************************************************/

void SamplePositionToString(char *cstring, size_t maxLen) {
	float longdegree, latdegree;
	char latindicator, longindicator;

	GetPosition(&longdegree, &latdegree, &longindicator, &latindicator);
	snprintf(cstring, maxLen, "GPS: longdegree: %.2f , latdegree: %.2f , latdegree: %c  , latindicator: %c \r\n",
			longdegree, latdegree, longindicator, latindicator);
}

/***************************************************************************/
void SampleUtcToString(char *cstring, size_t maxLen) {
	uint8_t hours, min, sec;

	GetUTC(&hours, &min, &sec);
	snprintf(cstring, maxLen, "GPS: hours: %d , min: %d , sec: %d   \r\n", hours, min, sec);

}

/***************************************************************************/
void SampleSpeedToString(char *cstring, size_t maxLen) {
	float speedinch, speedkm;

	GetSpeed(&speedinch, &speedkm);
	snprintf(cstring, maxLen, "GPS: speedinch: %.2f , speedkm: %.2f \r\n", speedinch, speedkm);

}
/*-----------------------------------------------------------*/
void SampleHeightToString(char *cstring, size_t maxLen) {
	float height;

	GetHeight(&height);
	snprintf(cstring, maxLen, "GPS: height: %.2f \r\n", height);

}

/*-----------------------------------------------------------*/
void stopStreamMems(void) {
	stopStream = true;
}

/***************************************************************************/
/***************************** General Functions ***************************/
/***************************************************************************/
Module_Status GetPosition(float *longdegree, float *latdegree, char *longindicator, char *latindicator) {
	Module_Status status = H1FR5_OK;

	*longdegree = GPS_INFO.Longitude.Degree; // tol
	*longindicator = GPS_INFO.Longitude.indicator;
	*latdegree = GPS_INFO.Latitude.Degree;  // ug
	*latindicator = GPS_INFO.Latitude.indicator;

	return status;
}

/***************************************************************************/
Module_Status GetUTC(uint8_t *hours, uint8_t *min, uint8_t *sec) {
	Module_Status status = H1FR5_OK;

	*hours = GPS_INFO.UTC_TIME.hrs;
	*min = GPS_INFO.UTC_TIME.mint;
	*sec = GPS_INFO.UTC_TIME.sec;

	return status;
}

/***************************************************************************/
Module_Status GetSpeed(float *speedinch, float *speedkm) {
	Module_Status status = H1FR5_OK;

	*speedinch = GPS_INFO.SPEED.speedKnot;
	*speedkm = GPS_INFO.SPEED.speedKm;

	return status;
}

/***************************************************************************/
Module_Status GetHeight(float *height) {
	Module_Status status = H1FR5_OK;

	*height = GPS_INFO.HEIGHT.height;

	return status;
}

/***************************************************************************/
/********************************* Commands ********************************/
/***************************************************************************/
static portBASE_TYPE SampleGPSCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString) {
	const char *const positionCmdName = "position";
	const char *const utcCmdName = "utc";
	const char *const speedCmdName = "speed";
	const char *const heightCmdName = "height";
	const char *pfuncName = NULL;

	portBASE_TYPE funcNameLen = 0;

	/* Make sure we return something */
	*pcWriteBuffer = '\0';

	pfuncName = (const char*) FreeRTOS_CLIGetParameter(pcCommandString, 1, &funcNameLen);

	if (pfuncName == NULL) {
		snprintf((char*) pcWriteBuffer, xWriteBufferLen, "Invalid Arguments\r\n");
		return pdFALSE;
	}

	do {
		if (!strncmp(pfuncName, positionCmdName, strlen(positionCmdName))) {
			SampleToTerminal(pcPort,POSITION);

		} else if (!strncmp(pfuncName, utcCmdName, strlen(utcCmdName))) {
			SampleToTerminal(pcPort,UTC);

		} else if (!strncmp(pfuncName, speedCmdName, strlen(speedCmdName))) {
			SampleToTerminal(pcPort,SPEED);

		} else if (!strncmp(pfuncName, heightCmdName, strlen(heightCmdName))) {
			SampleToTerminal(pcPort,HEIGHT);

		} else {
			snprintf((char*) pcWriteBuffer, xWriteBufferLen, "Invalid Arguments\r\n");
		}

		return pdFALSE;
	} while (0);

	snprintf((char*) pcWriteBuffer, xWriteBufferLen, "Error reading Sensor\r\n");

	return pdFALSE;
}

/***************************************************************************/
static bool StreamCommandParser(const int8_t *pcCommandString, const char **ppSensName, portBASE_TYPE *pSensNameLen,
bool *pPortOrCLI, uint32_t *pPeriod, uint32_t *pTimeout, uint8_t *pPort, uint8_t *pModule) {
	const char *pPeriodMSStr = NULL;
	const char *pTimeoutMSStr = NULL;
	const char *pPortStr = NULL;
	const char *pModStr = NULL;

	portBASE_TYPE periodStrLen = 0;
	portBASE_TYPE timeoutStrLen = 0;
	portBASE_TYPE portStrLen = 0;
	portBASE_TYPE modStrLen = 0;

	*ppSensName = (const char*) FreeRTOS_CLIGetParameter(pcCommandString, 1, pSensNameLen);
	pPeriodMSStr = (const char*) FreeRTOS_CLIGetParameter(pcCommandString, 2, &periodStrLen);
	pTimeoutMSStr = (const char*) FreeRTOS_CLIGetParameter(pcCommandString, 3, &timeoutStrLen);

	// At least 3 Parameters are required!
	if ((*ppSensName == NULL) || (pPeriodMSStr == NULL) || (pTimeoutMSStr == NULL))
		return false;

	// TODO: Check if Period and Timeout are integers or not!
	*pPeriod = atoi(pPeriodMSStr);
	*pTimeout = atoi(pTimeoutMSStr);
	*pPortOrCLI = true;

	pPortStr = (const char*) FreeRTOS_CLIGetParameter(pcCommandString, 4, &portStrLen);
	pModStr = (const char*) FreeRTOS_CLIGetParameter(pcCommandString, 5, &modStrLen);

	if ((pModStr == NULL) && (pPortStr == NULL))
		return true;
	if ((pModStr == NULL) || (pPortStr == NULL))	// If user has provided 4 Arguments.
		return false;

	*pPort = atoi(pPortStr);
	*pModule = atoi(pModStr);
	*pPortOrCLI = false;

	return true;
}

/***************************************************************************/
static portBASE_TYPE StreamGPSCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString) {
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

	if (!StreamCommandParser(pcCommandString, &pSensName, &sensNameLen, &portOrCLI, &Numofsamples, &timeout, &port,
			&module)) {
		snprintf((char*) pcWriteBuffer, xWriteBufferLen, "Invalid Arguments\r\n");
		return pdFALSE;
	}

	do {
		if (!strncmp(pSensName, positionCmdName, strlen(positionCmdName))) {
			if (portOrCLI) {
				StreamToCLI(Numofsamples,timeout,SamplePositionToString);

			} else {

				StreamtoPort(module, port, Numofsamples, timeout, POSITION);
			}

		} else if (!strncmp(pSensName, utcCmdName, strlen(utcCmdName))) {
			if (portOrCLI) {
				StreamToCLI(Numofsamples,timeout,SampleUtcToString);

			} else {

				StreamtoPort(module, port, Numofsamples, timeout, UTC);
			}

		} else if (!strncmp(pSensName, speedCmdName, strlen(speedCmdName))) {
			if (portOrCLI) {
				StreamToCLI(Numofsamples,timeout,SampleSpeedToString);

			} else {

				StreamtoPort(module, port, Numofsamples, timeout, SPEED);
			}

		} else if (!strncmp(pSensName, heightCmdName, strlen(heightCmdName))) {
			if (portOrCLI) {
				StreamToCLI(Numofsamples,timeout,SampleHeightToString);

			} else {

				StreamtoPort(module, port, Numofsamples, timeout, HEIGHT);
			}

		} else {
			snprintf((char*) pcWriteBuffer, xWriteBufferLen, "Invalid Arguments\r\n");
		}

		snprintf((char*) pcWriteBuffer, xWriteBufferLen, "\r\n");
		return pdFALSE;
	} while (0);

	snprintf((char*) pcWriteBuffer, xWriteBufferLen, "Error reading Sensor\r\n");

	return pdFALSE;
}

/***************************************************************************/
static portBASE_TYPE StopStreamCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString) {
	// Make sure we return something
	pcWriteBuffer [0] = '\0';
	snprintf((char*) pcWriteBuffer, xWriteBufferLen, "Stop Stream...\r\n");

	stopStreamMems();
	return pdFALSE;
}

/***************************************************************************/
/***************** (C) COPYRIGHT HEXABITZ ***** END OF FILE ****************/
