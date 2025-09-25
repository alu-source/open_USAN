/*
Copyright 2025 FH Aachen - University of Applied Sciences

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
#include "gpio.h"

static GPIO_TypeDef* const gpio_banks[] = {
  GPIOA, GPIOB, GPIOC, GPIOD, GPIOE, GPIOF, GPIOG, GPIOH
};

static const uint32_t gpio_pins [] = {
  GPIO_PIN_0, GPIO_PIN_1, GPIO_PIN_2, GPIO_PIN_3, 
  GPIO_PIN_4, GPIO_PIN_5, GPIO_PIN_6, GPIO_PIN_7,
  GPIO_PIN_8, GPIO_PIN_9, GPIO_PIN_10, GPIO_PIN_11, 
  GPIO_PIN_12, GPIO_PIN_13, GPIO_PIN_14, GPIO_PIN_15
};

static GPIO_InitTypeDef gpio_get_initStruct(uint8_t pin_num, gpio_mode_t mode, gpio_pull_t pull, gpio_speed_t speed, gpio_alternate_t alternateFunction);
static void gpio_enable_rcc(uint8_t pin_bank);

void gpio_pin_init(gpio_pin_init_t* gpio_init)
{
  gpio_enable_rcc(gpio_init->pin.bank);
  GPIO_InitTypeDef initStruct = gpio_get_initStruct(gpio_init->pin.num, gpio_init->mode, gpio_init->pull, gpio_init->speed, gpio_init->alternateFunction);

  HAL_GPIO_Init(gpio_banks[gpio_init->pin.bank], &initStruct);
}

void gpio_pin_write(gpio_pin_t pin, bool state)
{
  if(state == true) {
    HAL_GPIO_WritePin(gpio_banks[pin.bank], gpio_pins[pin.num], GPIO_PIN_SET);
  }
  else if(state == false) {
    HAL_GPIO_WritePin(gpio_banks[pin.bank], gpio_pins[pin.num], GPIO_PIN_RESET);
  }
}

bool gpio_pin_read(gpio_pin_t pin)
{
  return HAL_GPIO_ReadPin(gpio_banks[pin.bank], gpio_pins[pin.num]) != GPIO_PIN_RESET;
}

static GPIO_InitTypeDef gpio_get_initStruct(uint8_t pin_num, gpio_mode_t mode, gpio_pull_t pull, gpio_speed_t speed, gpio_alternate_t alternateFunction)
{
  GPIO_InitTypeDef initStruct;
  
  if(pin_num < 0 || pin_num > 15) {
    while(1); // No valid pin number specified
  }
  else {
    initStruct.Pin = gpio_pins[pin_num];
  }

  if(mode == INPUT) {
    initStruct.Mode = GPIO_MODE_INPUT;
  }
  else if(mode == OUTPUT_PUSH_PULL) {
    initStruct.Mode = GPIO_MODE_OUTPUT_PP;
  }
  else if(mode == OUTPUT_OPEN_DRAIN) {
    initStruct.Mode = GPIO_MODE_OUTPUT_OD;
  }
  else if(mode == AF_PUSH_PULL) {
    initStruct.Mode = GPIO_MODE_AF_PP;
  }
  else if(mode == AF_OPEN_DRAIN) {
    initStruct.Mode = GPIO_MODE_AF_OD;
  }
  else if(mode == ANALOG) {
    initStruct.Mode = GPIO_MODE_ANALOG;
  }
  else if(mode == INPUT_IT_RISING){
     initStruct.Mode = GPIO_MODE_IT_RISING;
  }
  else if(mode == INPUT_IT_FALING){
     initStruct.Mode = GPIO_MODE_IT_FALLING;
  }
  else if(mode == INPUT_IT_RISING_FALING){
      initStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  }
  
  if(pull == PULL_NONE) {
    initStruct.Pull = GPIO_NOPULL;
  }
  else if(pull == PULL_UP) {
    initStruct.Pull = GPIO_PULLUP;
  }
  else if(pull == PULL_DOWN) {
    initStruct.Pull = GPIO_PULLDOWN;
  }
  
  if(speed == SPEED_LOW) {
    initStruct.Speed = GPIO_SPEED_FREQ_LOW;
  }
  else if(speed == SPEED_MEDIUM) {
    initStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  }
  else if(speed == SPEED_HIGH) {
    initStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  }
  else if(speed == SPEED_VERY_HIGH) {
    initStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  }

  initStruct.Alternate = alternateFunction;

  return initStruct;
}

static void gpio_enable_rcc(uint8_t pin_bank)
{
  if(pin_bank == BANK_A) {
    __HAL_RCC_GPIOA_CLK_ENABLE();
  }
  else if(pin_bank == BANK_B) {
    __HAL_RCC_GPIOB_CLK_ENABLE(); 
  }
  else if(pin_bank == BANK_C) {
    __HAL_RCC_GPIOC_CLK_ENABLE(); 
  }
  else if(pin_bank == BANK_D) {
    __HAL_RCC_GPIOD_CLK_ENABLE(); 
  }
  else if(pin_bank == BANK_E) {
    __HAL_RCC_GPIOE_CLK_ENABLE(); 
  }
  else if(pin_bank == BANK_F) {
    __HAL_RCC_GPIOF_CLK_ENABLE(); 
  }
  else if(pin_bank == BANK_G) {
    __HAL_RCC_GPIOG_CLK_ENABLE(); 
  }
  else if(pin_bank == BANK_H) {
    __HAL_RCC_GPIOH_CLK_ENABLE();
  }
}
