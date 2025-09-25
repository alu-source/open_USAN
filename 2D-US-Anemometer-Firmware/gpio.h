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

//********************************************************************************
//* FH Aachen - University of Applied Sciences
//*
//* Driver : GPIO
//* Brief  : This driver comprises functions to configure and to operate
//*          the GPIOs:
//*            + Initialization of pins in a specific mode and configuration
//*            + Write and read the states (SET/RESET) of pins
//*
//********************************************************************************
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stm32f4xx_hal.h>

typedef struct {
  enum {
    BANK_A, BANK_B, BANK_C, 
    BANK_D, BANK_E, BANK_F, 
    BANK_G, BANK_H
  } bank;
  uint8_t num;
} gpio_pin_t;

typedef enum {
  INPUT,
  INPUT_IT_RISING,
  INPUT_IT_FALING,
  INPUT_IT_RISING_FALING,
  OUTPUT_PUSH_PULL,
  OUTPUT_OPEN_DRAIN,
  AF_PUSH_PULL,
  AF_OPEN_DRAIN,
  ANALOG
} gpio_mode_t;

typedef enum {
  PULL_NONE,
  PULL_UP,
  PULL_DOWN
} gpio_pull_t;

typedef enum {
  SPEED_LOW,
  SPEED_MEDIUM,
  SPEED_HIGH,
  SPEED_VERY_HIGH
} gpio_speed_t;

typedef uint8_t gpio_alternate_t;

typedef struct {
  gpio_pin_t pin;
  gpio_mode_t mode;
  gpio_pull_t pull;
  gpio_speed_t speed;
  gpio_alternate_t alternateFunction;
} gpio_pin_init_t;

void gpio_pin_init(gpio_pin_init_t* gpio_init);

void gpio_pin_write(gpio_pin_t pin, bool state);

bool gpio_pin_read(gpio_pin_t pin);
