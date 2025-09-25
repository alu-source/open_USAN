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
#include "stdlib.h"
#include "gpio.h"
#include "SRAM_IS66.h"
#include "Shared.h"

//Command def
static uint8_t SET_BURST_LENGTH = 0xC0;
static uint8_t IS66_WRITE = 0x02;
static uint8_t IS66_READ = 0x03;
static uint8_t IS66_GET_ID = 0x9F;


//SPI and DMA def
SPI_HandleTypeDef hspi3;
DMA_HandleTypeDef hdma_spi3_tx;
DMA_HandleTypeDef hdma_spi3_rx;
gpio_pin_init_t gpio_pin_CE;


SRAM_Instance IS66;

uint8_t SRAM_Init(void);
uint8_t SRAM_toggle_burstsize(uint16_t Burst);
uint8_t SRAM_write_page(uint8_t *data,uint16_t Page);
uint8_t SRAM_read_page(uint8_t *data,uint16_t Page);
void SRAM_write_cmp(void);
void SRAM_read_cmp(void);




uint8_t SRAM_Init(void){

  // Starting SPI
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK) {
    Error_Handler(3);
  }


  // Init the GPIO_Pins
  gpio_pin_CE.pin.num = SRAM_CE_PIN;
  gpio_pin_CE.pin.bank = SRAM_CE_PORT;
  gpio_pin_CE.mode = OUTPUT_PUSH_PULL;
  gpio_pin_CE.pull = PULL_NONE;
  gpio_pin_CE.speed = SPEED_HIGH;

  gpio_pin_init(&gpio_pin_CE);

  IS66.Burst_size = 1024;

  uint8_t ret[8];
  // Get ID
  gpio_pin_write(gpio_pin_CE.pin,1);
  HAL_Delay(10);
  gpio_pin_write(gpio_pin_CE.pin,0);
  uint8_t msg[4] = {};
  msg[0] = IS66_GET_ID;
  HAL_SPI_Transmit(&SRAM_SPI_HANDLE, (uint8_t *)msg, 4, 10);
  HAL_SPI_Receive(&SRAM_SPI_HANDLE, ret, 8, 100);
  gpio_pin_write(gpio_pin_CE.pin,1);

  if (ret[1] != 0b01011101) { // Check for bad RAM chip
    Warning_Handler(3);
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}



uint8_t SRAM_toggle_burstsize(uint16_t Burst){

	if(Burst != 1024 || Burst != 32){ //Checking for Range
		return EXIT_FAILURE;
	}
	else {
		if(Burst == IS66.Burst_size){ //Checking for Change
			return EXIT_SUCCESS;
		}

		//Toggle burst length
		gpio_pin_write(gpio_pin_CE.pin,0);
		HAL_SPI_Transmit(&SRAM_SPI_HANDLE,(uint8_t*)&SET_BURST_LENGTH, 1, 10);
		gpio_pin_write(gpio_pin_CE.pin,1);


		if(IS66.Burst_size == 1024){
			IS66.Burst_size = 32;
		}
		else{
			IS66.Burst_size = 1024;
		}
		return EXIT_SUCCESS;
	}
}



uint8_t SRAM_write_page(uint8_t *data,uint16_t Page){

	//Check Status and Page limit
	if(IS66.Status != 0 || Page > SRAM_PAGES -1){
                Warning_Handler(3);
		return EXIT_FAILURE;
	}
	IS66.Status = 1;

	if(IS66.Burst_size != 1024){
		SRAM_toggle_burstsize(1024);
	}

        //Shift to Pahe start Address, Page is 1024b
	uint32_t Address = (uint32_t)Page *1024;

        uint8_t t_data[4];
        t_data[0] = IS66_WRITE;
        t_data[1] = (uint8_t)(Address>>16);
        t_data[2] = (uint8_t)(Address>>8);
        t_data[3] = (uint8_t)(Address>>0);

	//Start Read
	gpio_pin_write(gpio_pin_CE.pin,0);
        HAL_SPI_Transmit(&SRAM_SPI_HANDLE,t_data, 4, 10); //CMD
        uint8_t ret = HAL_SPI_Transmit_DMA(&SRAM_SPI_HANDLE,(uint8_t*)data, IS66.Burst_size); //Start DMA
        
	return EXIT_SUCCESS;
}

uint8_t SRAM_read_page(uint8_t *data,uint16_t Page){

	//Check Status and Page limit
	if(IS66.Status != 0 || Page > SRAM_PAGES - 1){
                Warning_Handler(3);
		return EXIT_FAILURE;
	}

	IS66.Status = 2;
	if(IS66.Burst_size != 1024){
		SRAM_toggle_burstsize(1024);
	}

        //Shift to Pahe start Address, Page is 1024b
	uint32_t Address = (uint32_t)Page * 1024;
        uint8_t t_data[4];
        t_data[0] = IS66_READ;
        t_data[1] = (uint8_t)(Address>>16);
        t_data[2] = (uint8_t)(Address>>8);
        t_data[3] = (uint8_t)(Address>>0);
      
	//Start Read
	gpio_pin_write(gpio_pin_CE.pin,0);
        HAL_SPI_Transmit(&SRAM_SPI_HANDLE,t_data, 4, 10); //CMD

	uint8_t ret = HAL_SPI_Receive_DMA(&SRAM_SPI_HANDLE,(uint8_t*)data, IS66.Burst_size); //Start DMA
      
	return EXIT_SUCCESS;
}

uint8_t SRAM_Status(){
  return IS66.Status;
}

// Interupt stuff to unlock the RAM data 
void SRAM_write_cmp(){
	if(IS66.Status == 1){
		IS66.Status = 0;
		gpio_pin_write(gpio_pin_CE.pin,1);
	}
}

void SRAM_read_cmp(){
	if(IS66.Status == 2){
		IS66.Status = 0;
		gpio_pin_write(gpio_pin_CE.pin,1);
	}
}


//Interupt stuff
void DMA1_Stream0_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_spi3_rx);

}
void DMA1_Stream5_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_spi3_tx);
}


void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef* hspi){
  SRAM_write_cmp();
}
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef* hspi){
  SRAM_read_cmp();
}
