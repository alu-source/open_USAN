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
#include "DAC_AD5732.h"
#include "gpio.h"
#include "Shared.h"

//Register Definitions
#define REG_DAC      0b00000000
#define REG_RANGE    0b00001000
#define REG_PW       0b00010000
#define REG_CON      0b00011000
#define REG_READ     0b10000000
#define REG_WRITE    0b00000000
#define REG_NOP      0b00011000


#define RANGE_u_5    0b00000000 // Unipolar
#define RANGE_u_10   0b00000001
#define RANGE_u_10_8 0b00000010
#define RANGE_b_5    0b00000011 // Bipolar
#define RANGE_b_10   0b00000100
#define RANGE_b_10_8 0b00000101

#define ADDR_A       0b00000000
#define ADDR_B       0b00000010
#define ADDR_AB      0b00000100

#define PW_UP_AB     0b00000101
  

#define Volt_to_cnt (DAC_V_RANGE / 16384.0)
#define V_max DAC_V_RANGE / DAC_V_TYPE - DAC_V_MARGIN
#define V_min (-DAC_V_RANGE *(DAC_V_TYPE - 1)) / DAC_V_TYPE + DAC_V_MARGIN



SPI_HandleTypeDef DAC_SPI_HANDLE;
SPI_HandleTypeDef hspi1;

/*
 * Extern
 */
uint8_t DAC_Init();
uint8_t DAC_write(float V_A, float V_B, uint8_t Update);

/*
 * Intern
 */
uint16_t DAC_Volt_to_CNT_A(float V_in);
uint16_t DAC_Volt_to_CNT_B(float V_in);

// GPIO Data struct
gpio_pin_init_t gpio_pin_NSS, gpio_pin_Clear, gpio_pin_UP;




uint8_t DAC_Init(){
  // Starting SPI
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK) {
    Error_Handler(2);
  }

  // Init Pins
  gpio_pin_NSS.pin.num = DAC_NSS_PIN;
  gpio_pin_NSS.pin.bank = DAC_NSS_PORT;
  gpio_pin_NSS.mode = OUTPUT_PUSH_PULL;
  gpio_pin_NSS.pull = PULL_NONE;
  gpio_pin_NSS.speed = SPEED_HIGH;

  gpio_pin_Clear.pin.num = DAC_CLEAR_PIN;
  gpio_pin_Clear.pin.bank = DAC_CLEAR_PORT;
  gpio_pin_Clear.mode = OUTPUT_PUSH_PULL;
  gpio_pin_Clear.pull = PULL_NONE;
  gpio_pin_Clear.speed = SPEED_HIGH;

  gpio_pin_UP.pin.num = DAC_UP_PIN;
  gpio_pin_UP.pin.bank = DAC_UP_PORT;
  gpio_pin_UP.mode = OUTPUT_PUSH_PULL;
  gpio_pin_UP.pull = PULL_NONE;
  gpio_pin_UP.speed = SPEED_HIGH;
  gpio_pin_init(&gpio_pin_NSS);
  gpio_pin_init(&gpio_pin_Clear);
  gpio_pin_init(&gpio_pin_UP);

  gpio_pin_write(gpio_pin_NSS.pin, 1);
  gpio_pin_write(gpio_pin_UP.pin, 1);
  gpio_pin_write(gpio_pin_Clear.pin,1);

  // Set Range
  uint8_t MSG[3] = {0};
  MSG[0] = MSG[0] | REG_WRITE | REG_RANGE | ADDR_AB;
  MSG[2] = MSG[2] | DAC_USE_RANGE;

  gpio_pin_write(gpio_pin_NSS.pin, 0);
    __NOP(); //Empty instruction
  HAL_SPI_Transmit(&DAC_SPI_HANDLE, &MSG[0], 1, 10);
  HAL_SPI_Transmit(&DAC_SPI_HANDLE, &MSG[1], 1, 10);
  HAL_SPI_Transmit(&DAC_SPI_HANDLE, &MSG[2], 1, 10);
  gpio_pin_write(gpio_pin_NSS.pin, 1);
  HAL_Delay(10);

  // Power up channels
  MSG[0] = 0 | REG_WRITE | REG_PW;
  MSG[1] = 0;
  MSG[2] = 0 | PW_UP_AB;

  gpio_pin_write(gpio_pin_NSS.pin, 0);
  __NOP(); //Empty instruction so register is updated
  HAL_SPI_Transmit(&DAC_SPI_HANDLE, &MSG[0], 1, 10);
  HAL_SPI_Transmit(&DAC_SPI_HANDLE, &MSG[1], 1, 10);
  HAL_SPI_Transmit(&DAC_SPI_HANDLE, &MSG[2], 1, 10);
  gpio_pin_write(gpio_pin_NSS.pin, 1);
  HAL_Delay(10);

  return EXIT_SUCCESS;
}

uint8_t DAC_write(float V_A, float V_B, uint8_t Update){
	if(V_A > V_max | V_A < V_min | V_B > V_max | V_B < V_min){ //Checking Input
		return EXIT_FAILURE;
	}

        //Shifting data in to Format
	uint16_t A_cnt = DAC_Volt_to_CNT_A(V_A);
	uint16_t B_cnt = DAC_Volt_to_CNT_B(V_B);

        //Writing OUT A Register
	uint8_t MSG_A[3] = {0};
	uint16_t *ptr =(uint16_t*)&MSG_A[1];
	MSG_A[0] = REG_DAC | ADDR_A;
        MSG_A[1] = (A_cnt >> 8) & 0x00FF;
        MSG_A[2] = (A_cnt) & 0x00FF;

        gpio_pin_write(gpio_pin_NSS.pin,0);
        __NOP();
       	HAL_SPI_Transmit(&DAC_SPI_HANDLE, MSG_A, 3, 10);
	gpio_pin_write(gpio_pin_NSS.pin,1);



        //Writing OUT B Register
	uint8_t MSG_B[3] = {0};
	ptr =(uint16_t*)&MSG_B[1];
	MSG_B[0] = REG_WRITE | REG_DAC | ADDR_B;
        MSG_B[1] = (B_cnt >> 8) & 0x00FF;
        MSG_B[2] = (B_cnt) & 0x00FF;

	gpio_pin_write(gpio_pin_NSS.pin,0);
        __NOP();
	HAL_SPI_Transmit(&DAC_SPI_HANDLE, MSG_B, 3, 10);
	gpio_pin_write(gpio_pin_NSS.pin,1);


	if(Update == 1){ //Trigger update PIN
		gpio_pin_write(gpio_pin_UP.pin,0);
                // Wait for the register to be written 
                __NOP();
                __NOP();
                __NOP();
		gpio_pin_write(gpio_pin_UP.pin,1);

	}
	return EXIT_SUCCESS;
}


uint16_t DAC_Volt_to_CNT_A(float V_in){
	//Future space for Calibration

	 uint16_t ret = 0b0010000000000000; //Offset for 0 V,, 2 bit shifted
         float h1 = (1.0/Volt_to_cnt);
         float h2 = V_in * h1;
         int16_t h3 = (int16_t)h2;
         ret = ret + h3;
 
	 return ret << 2; // Shift by to bit see datasheet
}
uint16_t DAC_Volt_to_CNT_B(float V_in){
	//Future space for Calibration

	 uint16_t ret = 0b0010000000000000; //Offset for 0 V,, 2 bit shifted
         float h1 = (1.0/Volt_to_cnt);
         float h2 = V_in * h1;
         int16_t h3 = (int16_t)h2;
         ret = ret + h3;
 
	 return ret << 2; // Shift by to bit see datasheet
}
