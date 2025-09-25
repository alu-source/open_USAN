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

#ifndef INC_SRAM_IS66_H_
#define INC_SRAM_IS66_H_

#include "main.h"
#include "stdlib.h"

//SRAM Setup
#define SRAM_CE_PIN 15
#define SRAM_CE_PORT BANK_A
#define SRAM_SPI_HANDLE hspi3
#define SRAM_PAGES 4096

/**
  Init
*/
uint8_t SRAM_Init();

/**
  Toggle Burst length from 1024 (default) to 32 
*/
uint8_t SRAM_toggle_burstsize(uint16_t Burst);

/**
  Writes whole Page 
    data must be of size 1024
    Page is Page number
*/
uint8_t SRAM_write_page(uint8_t *data,uint16_t Page);

/**
  reads whole Page 
    data must be of size 1024
    Page is Page number
*/
uint8_t SRAM_read_page(uint8_t *data,uint16_t Page);

/**
  Callbacks to set/reset Status of SRAM
*/
void SRAM_write_cmp(void);
void SRAM_read_cmp(void);
uint8_t SRAM_Status(void);

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef* hspi);
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef* hspi);
void DMA1_Stream0_IRQHandler(void);
void DMA1_Stream5_IRQHandler(void);


typedef struct{
	uint8_t Status; // 0 Ready, 1 Write in progress, 2 Read in progress
	uint16_t Burst_size;
}SRAM_Instance;


#endif /* INC_SRAM_IS66_H_ */
