/*
 BitzOS (BOS) V0.3.6 - Copyright (C) 2017-2024 Hexabitz
 All rights reserved
 
 File Name     : H1FR5_MemoryMap.h
 Description   : Module MCU memory map header file.

 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef H1FR5_MEMORYMAP_H
#define H1FR5_MEMORYMAP_H

#ifdef __cplusplus
 extern "C" {
#endif
#ifdef FLASH_SIZE
#undef FLASH_SIZE
#endif
/* Memory map: - STM32G0B0
 - Application: 0x08000000 - 0x0807FFFF >> 512 KB
 - Read-only (RO): 0x08040000 -  >> 2 KB, used to store topology information
  - Read-only (RO): 0x08040800 -  >> 2 KB, used to store Command Snippets
 - Emulated EEPROM: 0x0807B800 -  >> 8 KB, fits 1000 16-bit variables in 4 pages
 */
#define APP_START_ADDRESS  		((uint32_t)0x08000000) 
#define TOPOLOGY_START_ADDRESS  ((uint32_t)0x08040000)      // topology is stored here
#define SNIPPETS_START_ADDRESS  ((uint32_t)0x08040800) 		// Snippets are stored here
#define EEPROM_START_ADDRESS  	((uint32_t)0x0807B800U)      // EE_Variables are stored gere
#define FLASH_SIZE				((uint32_t)0x20000)			// All sizes in bytes
#define SRAM_SIZE				((uint32_t)0x8000)
#define TOPOLOGY_PAGE_NUM		256
#define SNIPPETS_PAGE_NUM		258

#ifdef __cplusplus
}
#endif

#endif /* H1FR5_MEMORYMAP_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
