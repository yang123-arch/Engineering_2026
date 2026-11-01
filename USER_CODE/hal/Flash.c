#include "Flash.h"

#define FLASH_BANK2_PAGE_COUNT 128U

uint16_t MyFLASH_ReadHalfWord(uint32_t address) {
  return *(__IO uint16_t *)address;
}

int FLASH_programword(uint32_t address, uint64_t data) {
  if ((address & 7U) != 0U || HAL_FLASH_Unlock() != HAL_OK) {
    return -1;
  }

  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPTVERR);
  HAL_StatusTypeDef status = HAL_FLASH_Program(
      FLASH_TYPEPROGRAM_DOUBLEWORD, address, data);
  HAL_FLASH_Lock();
  return status == HAL_OK ? 0 : -1;
}

int FLASH_ErasePage(uint32_t page) {
  if (page >= FLASH_BANK2_PAGE_COUNT || HAL_FLASH_Unlock() != HAL_OK) {
    return -1;
  }

  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPTVERR);
  FLASH_EraseInitTypeDef erase = {0};
  uint32_t page_error = 0;
  erase.TypeErase = FLASH_TYPEERASE_PAGES;
  erase.Banks = FLASH_BANK_2;
  erase.Page = page;
  erase.NbPages = 1;

  HAL_StatusTypeDef status = HAL_FLASHEx_Erase(&erase, &page_error);
  HAL_FLASH_Lock();
  return status == HAL_OK ? 0 : -1;
}
