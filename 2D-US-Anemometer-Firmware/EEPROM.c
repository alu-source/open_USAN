/*
Copyright 2025 Till Weise

Redistribution and use in source and binary forms, with or without modification, 
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice,
 this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
 this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products
 derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS “AS IS” AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING,
 BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
 OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#include "main.h"
#include "EEPROM.h"
#include "stdlib.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_flash.h"

uint8_t EEPROM_Init(void);
uint8_t EEPROM_write_data(uint32_t virt_addr, uint32_t *data_in, uint32_t length, uint8_t erase_flag);
uint8_t EEPROM_read_data(uint32_t virt_addr, uint32_t *data_out, uint32_t length);

uint8_t EEPROM_read_data(uint32_t virt_addr, uint32_t *data_out, uint32_t length) {
  //Check for size
  if (virt_addr * 4 + length * 4 > EEPROM_SIZE) {
    return EXIT_FAILURE;
  }
  if (data_out == NULL) {
    return EXIT_FAILURE;
  }
  
  //Read flash 4 bits at a time
  uint32_t StartPageAddress = virt_addr + EEPROM_BASE_ADR;
  while (1) {
    *data_out = *(__IO uint32_t *)StartPageAddress;
    StartPageAddress += 4;
    data_out++;
    if (!(length--))
      break;
  }
  return EXIT_SUCCESS;
}

uint8_t EEPROM_write_data(uint32_t virt_addr, uint32_t *data_in, uint32_t length, uint8_t erase_flag) {
  if (erase_flag == 1) {
    // Erase PAGES -> only erased flash can be written to
    FLASH_EraseInitTypeDef eraseStruct = {.TypeErase    = FLASH_TYPEERASE_SECTORS,
                                          .Banks        = FLASH_BANK_1,
                                          .Sector       = EEPROM_START_SECTOR,
                                          .NbSectors    = EEPROM_END_SECTOR - EEPROM_START_SECTOR + 1,
                                          .VoltageRange = VOLTAGE_RANGE_3};
    uint32_t error                     = 0;
    HAL_FLASH_Unlock();
    FLASH_WaitForLastOperation(2000);
    uint32_t status = HAL_FLASHEx_Erase(&eraseStruct, &error);
    FLASH_WaitForLastOperation(5000); // otherwise the erase might be incomplete

    if (error != 0xFFFFFFFFU || status != HAL_OK) {
      HAL_Delay(1);
    }
    HAL_Delay(100);
  }

  // Write new data
  if (virt_addr + length * 4 > EEPROM_SIZE) {
    return EXIT_FAILURE;
  }
  if (data_in == NULL) {
    return EXIT_FAILURE;
  }
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPERR);

  // Unlock Flash
  HAL_FLASH_Unlock();

  uint32_t written          = 0;
  uint32_t StartPageAddress = virt_addr + EEPROM_BASE_ADR;

  while (written < length) {
    uint32_t l_data = data_in[written];
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, StartPageAddress, l_data) == HAL_OK) {
      StartPageAddress += 4;    // because we write 4 bytes at a time
      written++;
    } else {
      // Error occurred while writing data in Flash memory
      return HAL_FLASH_GetError();
    }
  }

  HAL_FLASH_Lock();
  return EXIT_SUCCESS;
}
