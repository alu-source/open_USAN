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
//* Driver : UART
//* Brief  : This driver comprises functions to configure and utilize the
//*          universal asynchronous receiver transmitter (UART) module:
//*            + Initialization of UART instance
//*            + Change of a UART instance's baudrate
//*            + Check if an UART instance received data 
//*            + Reception and transmission of data via UART instance
//*
//********************************************************************************
#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include <stm32f4xx_hal.h>

#include "gpio.h"

#define UART_RX_BUFFERSIZE 1024
#define UART_TX_BUFFERSIZE 1024

typedef enum {
  UART_1,
  UART_2,
  UART_3,
  UART_4,
  UART_5,
  UART_6
} uart_instance_t;

typedef struct {
  uart_instance_t uart_instance;
  uint32_t baudrate;
  gpio_pin_t pin_rx;
  gpio_pin_t pin_tx;
} uart_init_t;

typedef struct {
  uart_instance_t uart_instance;
  volatile uint8_t buffer_rx[UART_RX_BUFFERSIZE];
  volatile uint8_t buffer_tx[UART_TX_BUFFERSIZE];
} uart_t;

// Initialization of UART instance
void uart_init(const uart_init_t* uart_init, uart_t* uart_struct);

// Change of a UART instance's baudrate 
void uart_changeBaudrate(const uart_t* uart_struct, uint32_t baudrate);

// Read received data of UART instance
uint16_t uart_receive(const uart_t* uart_struct, const void* data, uint16_t dataSize);

// Transmit data via UART instance
uint16_t uart_transmit(const uart_t* uart_struct, const void* data, uint16_t dataSize);

// Check if an UART instance received data
uint16_t uart_bytesReadable(const uart_t *uart_struct);

// Functions for IRQHandler
void DMA2_Stream7_IRQHandler(void); // for UART_1
void USART1_IRQHandler(void);

void DMA1_Stream6_IRQHandler(void); // for UART_2
void USART2_IRQHandler(void);

void DMA1_Stream3_IRQHandler(void); // for UART_3
void USART3_IRQHandler(void);

void DMA1_Stream4_IRQHandler(void); // for UART_4
void UART4_IRQHandler(void);

void DMA1_Stream7_IRQHandler(void); // for UART_5
void UART5_IRQHandler(void);

void DMA2_Stream6_IRQHandler(void); // for UART_6
void USART6_IRQHandler(void);


