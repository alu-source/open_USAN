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
#include "stdint.h"

// Used flash is included from mem map to avoid colisons 
#define EEPROM_BASE_ADR 0x080A0000//View ref manual RM0386
#define EEPROM_SIZE 0x00060000 //384k -> coulde be smaller but due to the sector sizes not feasible
#define EEPROM_START_SECTOR FLASH_SECTOR_9 // RM0386
#define EEPROM_END_SECTOR FLASH_SECTOR_11 //RM0386



/**
  Unused
*/
uint8_t EEPROM_Init();

/**
  Write stuff to the EEPROM
  in 4 Byte stepps
*/
uint8_t EEPROM_write_data(uint32_t virt_addr, uint32_t * data_in, uint32_t length, uint8_t erase_flag);

/**
  Reads stuff from the EEPROM
  in 4 Byte stepps
*/
uint8_t EEPROM_read_data(uint32_t virt_addr, uint32_t * data_out, uint32_t length);
