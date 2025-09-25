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
#pragma once

#include "main.h"
#include "stm32f4xx_hal_adc.h"
#include "Sensor_config.h"

#define SHOT_KEY_LENGTH 12
#define SHOT_DMA_CYCLES_PER_PERIODE 20
#define SHOT_DMA_LENGTH  512 
#define SHOT_FREQ 800000
#define SHOT_ADC_OFFSET ADC_OFFSET

#define SHOT_PORT GPIOD
//Transceiver 0
#define SHOT_TR0_H GPIO_PIN_0
#define SHOT_TR0_L GPIO_PIN_1
#define SHOT_TR0_C GPIO_PIN_2
#define SHOT_TR0_A ADC_CHANNEL_0

//Transceiver 1
#define SHOT_TR1_H GPIO_PIN_3
#define SHOT_TR1_L GPIO_PIN_4
#define SHOT_TR1_C GPIO_PIN_5
#define SHOT_TR1_A ADC_CHANNEL_1

//Transceiver 2
#define SHOT_TR2_H GPIO_PIN_7
#define SHOT_TR2_L GPIO_PIN_8
#define SHOT_TR2_C GPIO_PIN_9
#define SHOT_TR2_A ADC_CHANNEL_2

//Transceiver 3
#define SHOT_TR3_H GPIO_PIN_10
#define SHOT_TR3_L GPIO_PIN_14
#define SHOT_TR3_C GPIO_PIN_15
#define SHOT_TR3_A ADC_CHANNEL_3

//Modulo for fire squence 
#define SHOT_POW_MODULO 2

typedef struct{
  uint16_t TR_H;  //Highside of Transmitter
  uint16_t TR_L;  //Lowside of Transmitter
  uint16_t TR_C;  //F-FETs that have to be deactivate for send and receive 
  uint16_t TR_CC; //  ->For Dampening after Sending
  uint32_t TR_A;  //Analog of receiver
  uint16_t gpio_data[SHOT_DMA_LENGTH];
}Transceiver_handle;

typedef struct{
uint8_t lock; //0 Open , 1 Locked do not touch 
uint16_t data[SHOT_DMA_LENGTH + SHOT_ADC_OFFSET];
}Shot_data_struct;


/**
  GPIO, ADC, DMA, Tim set up 
*/
uint8_t Shot_Init(Transceiver_handle* handle);

/**
  Generates GPIO register states from KEY for usage with DMA
  handle is must be array of 4
*/
uint8_t Shot_generate_gpio_from_Key(uint8_t *KEY, Transceiver_handle *handle);

//Alternativ ways to generate the KEY, sometimes very interesting results, not all bug free
uint8_t Shot_generate_gpio_from_Key_2(uint8_t *KEY, Transceiver_handle *handle);
uint8_t Shot_generate_gpio_from_Key_3(uint8_t *KEY, Transceiver_handle *handle);
uint8_t Shot_generate_gpio_from_Key_4(uint8_t *KEY, Transceiver_handle *handle);

/**
  Fire in the hole 
  Data has to be Unlocked after DMA is finnished 
*/
uint8_t Shot_fire(Transceiver_handle *handle, Shot_data_struct *data);

/**
  Unlocks data struct if Shot is finshed
*/
uint8_t Shot_unlock_data(Shot_data_struct * data);