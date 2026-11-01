#ifndef USER_HAL_FLASH_H
#define USER_HAL_FLASH_H

#include "main.h"

uint16_t MyFLASH_ReadHalfWord(uint32_t address);
int FLASH_programword(uint32_t address, uint64_t data);
int FLASH_ErasePage(uint32_t page);

#endif
