 /*
 BitzOS (BOS) V0.3.6 - Copyright (C) 2017-2024 Hexabitz
 All rights reserved

 File Name     : BOS_eeprom.h
 Description   : EEPROM emulator library (Header file).

 * EEPROM Emulation Description:
 * -----------------------------
 * - The EEPROM emulation is developed to store 1000 elements, where each element is 8 bytes (64 bits).
 * - The total storage requires 4 pages of flash memory, calculated as follows:
 *     Number of pages = (Number of elements × 8) / Flash page size
 *                    = (1000 × 8) / 2048
 *                    = 4 pages
 *
 * Memory Organization:
 * -------------------
 * - Element Structure (8 Bytes / 64 Bits):
 *   - Bits 63–32: Data Value (32 bits)
 *   - Bits 31–16: CRC (16 bits)
 *   - Bits 15–0:  Virtual Address (16 bits)
 *
 * - Page Allocation:
 *   - First Group:  4 pages reserved for storing EEPROM elements.
 *   - Second Group: 4 additional pages reserved as a backup.
 *
 * Storage Mechanism:
 * -----------------
 * 1. Initial Storage:
 *    - Data is stored in the first group of 4 pages.
 *
 *2. Group Swap:
 *    - If the first group becomes full, the latest changes are transferred from the first group to the second group.
 *    - The storage process is then completed in the second group.
 *    - If the second group becomes full, the latest changes are transferred from the second group to the first group.
 *    - The storage process is then completed in the first group.
 *
 * 3. Cycle Continuation:
 *    - This process alternates between the two groups to ensure efficient and reliable storage.
 *
 * Purpose of the Second Group:
 * ---------------------------
 * - The second group acts as a backup to facilitate the transfer of updated elements between the two groups.
 * - This ensures that the latest data is always preserved and the storage process is seamless.
 *
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BOS_EEPROM_H
#define __BOS_EEPROM_H

/* Includes ------------------------------------------------------------------*/
#include "BOS.h"

/* Exported constants --------------------------------------------------------*/

/* EEPROM Variables' number (up to 1000 16-bit variables) */
#define NumOfEEPROMvar              1000

/* EEPROM virtual addresses - Consider MaxNumOfModules is 25 */

/* BOS Addressing Space 1 - 499 */
#define _EE_NBASE					1
#define _EE_PORT_DIR_BASE			2		/* 25 modules - 25 variables */
#define _EE_ALIAS_BASE				28		/* 25 modules/10 chars - 125 variables */
#define _EE_GROUP_ALIAS_BASE		153		/* 10 groups/10 chars - 50 variables */
#define _EE_GROUP_MODULES_BASE		203		/* 25 modules - 25 variables */
#define _EE_DMA_STREAM_BASE			228		/* 8 variables */
#define _EE_BUTTON_BASE				236		/* 4 * MaxNumOfPorts (10) variables for buttons: port(4 bits), type (4 bits), events (8 bits)
											 * pressed_for_x_1 (8 bits), released_for_y_1 (8 bits), etc. */
#define _EE_PARAMS_BASE				276		/* Parameter base: BOS trace (MSB) | BOS response - 1 variable */
#define _EE_PARAMS_DEBOUNCE			277		/* Parameter: Button debounce - 1 variable */
#define _EE_PARAMS_SINGLE_CLICK		278		/* Parameter: Button single click - 1 variable */
#define _EE_PARAMS_DBL_CLICK		279		/* Parameter: Button double click (inter-click min and max) - 1 variable */
#define _EE_CLI_BAUD				280		/* Parameter: CLI baudrate - LSB halfword, MSB halfword - 2 variables */
#define _EE_PARAMS_RTC				282		/* Parameter: RTC hourformat | RTC daylightsaving - 1 variable */
#define _EE_PARAMS_DISABLE_CLI		283		/* Parameter: Disable CLI - 1 variable */
#define _EE_PARAMS_Messaging	    284		/* Parameter base: BOSMessaging Acknowledgment (MSB) | BOS trial - 1 variable */

/* Module Addressing Space 500 - 599 */
#define _EE_MODULE_BASE				500

/* User Addressing Space 600 - 1000 */
#define _EE_EMPTY_VAR_BASE			600

#if MaxNumOfModules > 26						// Update
 #warning "Data for 26 modules only will be stored in EEPROM."
#endif


/* Exported functions ------------------------------------------------------- */
BOS_Status EE_Init(void);
BOS_Status EE_Format(void);
BOS_Status EE_ReadVariable(uint16_t VirtAddress,uint16_t *Data);
BOS_Status EE_WriteVariable(uint16_t VirtAddress,uint16_t Data);
uint16_t Flash_WriteVariable(uint32_t Address,uint16_t Data);
BOS_Status EraseSector(uint32_t Sector);

#endif /* __BOS_EEPROM_H */

/************************ (C) COPYRIGHT HEXABITZ *****END OF FILE****/
