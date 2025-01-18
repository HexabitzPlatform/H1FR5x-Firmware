/*
 BitzOS (BOS) V0.3.6 - Copyright (C) 2017-2024 Hexabitz
 All rights reserved

 File Name     : BOS_eeprom.c
 Description   : EEPROM emulator library (Source file).

 */

/* Includes ------------------------------------------------------------------*/
#include "BOS.h"

/* Variables used for Erase pages under interruption */
extern FLASH_ProcessTypeDef pFlash;

/* Private functions ---------------------------------------------------------*/

/******************************************************************************/

/*
 * @brief  Restore the pages to a known good state in case of page's status
 *   corruption after a power loss.
 * @param  None.
 * @retval - BOS_OK.
 *         - BOS_ERR_EEPROM.
 */
BOS_Status EE_Init(void) {
	BOS_Status Status = BOS_OK;
	EE_Status eeStatus = EE_OK;

	HAL_FLASH_Unlock();
	eeStatus = EEPROM_Init(EE_FORCED_ERASE);
	if((EE_WRITE_ERROR == eeStatus || EE_INVALID_PAGE_SEQUENCE == eeStatus || EE_TRANSFER_ERROR == eeStatus))
	{
		EE_Format();
		HAL_FLASH_Lock();
		NVIC_SystemReset();
	}
	HAL_FLASH_Lock();

	return Status;
}

/******************************************************************************/

/*
 * @brief  Read the last stored variable data.
 * @param  VirtAddress: Variable virtual address.
 * @param  Data: Global variable contains the read variable value.
 * @retval - BOS_OK.
 *         - BOS_ERR_EEPROM.
 */
BOS_Status EE_ReadVariable(uint16_t VirtAddress, uint16_t *Data) {
	BOS_Status Status = BOS_OK;

	if(EE_OK != EE_ReadVariable16bits(VirtAddress, Data))
		return Status = BOS_ERR_EEPROM;

	return Status;
}

/******************************************************************************/

/*
 * @brief  Writes/upadtes variable data in EEPROM.
 * @param  VirtAddress: Variable virtual address.
 * @param  Data: 16 bit data to be written.
 * @retval - BOS_OK.
 *         - BOS_ERR_EEPROM.
 */
BOS_Status EE_WriteVariable(uint16_t VirtAddress,uint16_t Data){
	BOS_Status Status = BOS_OK;
	EE_Status eeStatus = EE_OK;

	HAL_FLASH_Unlock();
	eeStatus = EE_WriteVariable16bits(VirtAddress, Data);
	if (EE_CLEANUP_REQUIRED == eeStatus)
	{
		EE_CleanUp();
	}
	else if(EE_OK != eeStatus)
	{
		HAL_FLASH_Lock();
		return BOS_ERR_EEPROM;
	}
	HAL_FLASH_Lock();

	return Status;
}

/******************************************************************************/

/*
 * @brief  Erasing reserve pages for EEPROM emulator and assign the first page as Active.
 * @param  None.
 * @retval - BOS_OK.
 *         - BOS_ERR_EEPROM.
 */
BOS_Status EE_Format(void) {
	BOS_Status Status = BOS_OK;

	HAL_FLASH_Unlock();
	if(EE_OK != EEPROM_Format(EE_FORCED_ERASE))
	{
		HAL_FLASH_Lock();
		return Status = BOS_ERR_EEPROM;
	}
	HAL_FLASH_Lock();

	return Status;

}

/******************************************************************************/

/*
 * @brief  Writes/upadtes variable data in Flash.
 * @param  Address: Variable address.
 * @param  Data: 16 bit data to be written.
 * @retval Success or error status:
 *           - FLASH_COMPLETE: on success.
 *           - PAGE_FULL: if valid page is full.
 *           - NO_VALID_PAGE: if no valid page was found.
 *           - Flash error code: on write Flash error.
 */
uint16_t Flash_WriteVariable(uint32_t Address, uint16_t Data) {
	HAL_StatusTypeDef FlashStatus = HAL_OK;

	HAL_FLASH_Unlock();
	/* Set variable data */
//	HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,Address,Data);
	//TOBECHECKED
	/* Wait for last operation to be completed */
	FlashStatus = FLASH_WaitForLastOperation((uint32_t) HAL_FLASH_TIMEOUT_VALUE);
	/* If program operation was failed, a Flash error code is returned */
	if (FlashStatus != HAL_OK) {
		return pFlash.ErrorCode;
	} else {
		/* If the program operation is completed, disable the PG Bit */
		CLEAR_BIT(FLASH->CR, FLASH_CR_PG);
	}
	HAL_FLASH_Lock();

	/* Return last operation flash status */
	return FlashStatus;
}

/******************************************************************************/

/*
 * @brief  Erase sector in flash memory.
 * @param  Sector : Sector to be erased.
 * @retval - BOS_OK.
 *         - BOS_ERR_REMOTE_WRITE_FLASH.
 */
BOS_Status EraseSector(uint32_t Sector) {
	BOS_Status Status = BOS_OK;
	FLASH_EraseInitTypeDef erase;
	uint32_t eraseError;

	erase.TypeErase = FLASH_TYPEERASE_PAGES;
	//erase.PageAddress =sector;
	erase.Page = Sector;
	//TOBECHECKED
	erase.NbPages = 1;
	if ((HAL_OK != HAL_FLASHEx_Erase(&erase, &eraseError)) || eraseError != 0xFFFFFFFF)
		responseStatus = BOS_ERR_REMOTE_WRITE_FLASH;

	return Status;
}

/************************ (C) COPYRIGHT HEXABITZ *****END OF FILE****/
