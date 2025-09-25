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
#include "uart.h"

static struct uart_infoStruct {
  UART_HandleTypeDef huart;
  const IRQn_Type huart_IRQn;

  DMA_HandleTypeDef hdma_rx;
  volatile uint8_t* buffer_rx;
  volatile uint16_t start_rx;

  DMA_HandleTypeDef hdma_tx;
  const IRQn_Type hdma_tx_IRQn;
  volatile uint8_t* buffer_tx;
  volatile uint16_t first_tx;
  volatile uint16_t afterLast_tx;

  const uint8_t gpio_alternateFunction;

  volatile bool isInitialized;

  volatile bool onTransmission;
  volatile uint16_t transmissionSize;
} uart_infoStructs[] = {
  [UART_1] = {
    .huart = {.Instance = USART1},
    .huart_IRQn = USART1_IRQn,

    .hdma_rx = {.Instance = DMA2_Stream2, .Init.Channel = DMA_CHANNEL_4},
    .hdma_tx = {.Instance = DMA2_Stream7, .Init.Channel = DMA_CHANNEL_4},
    .hdma_tx_IRQn = DMA2_Stream7_IRQn,

    .gpio_alternateFunction = GPIO_AF7_USART1,
    
    .isInitialized = false
  },
  [UART_2] = {
    .huart = {.Instance = USART2},
    .huart_IRQn = USART2_IRQn,

    .hdma_rx = {.Instance = DMA1_Stream5, .Init.Channel = DMA_CHANNEL_4},
    .hdma_tx = {.Instance = DMA1_Stream6, .Init.Channel = DMA_CHANNEL_4},
    .hdma_tx_IRQn = DMA1_Stream6_IRQn,

    .gpio_alternateFunction = GPIO_AF7_USART2,
    
    .isInitialized = false
  },
  [UART_3] = {
    .huart = {.Instance = USART3},
    .huart_IRQn = USART3_IRQn,

    .hdma_rx = {.Instance = DMA1_Stream1, .Init.Channel = DMA_CHANNEL_4},
    .hdma_tx = {.Instance = DMA1_Stream3, .Init.Channel = DMA_CHANNEL_4},
    .hdma_tx_IRQn = DMA1_Stream3_IRQn,

    .gpio_alternateFunction = GPIO_AF7_USART3,
    
    .isInitialized = false
  },
  [UART_4] = {
    .huart = {.Instance = UART4},
    .huart_IRQn = UART4_IRQn,

    .hdma_rx = {.Instance = DMA1_Stream2, .Init.Channel = DMA_CHANNEL_4},
    .hdma_tx = {.Instance = DMA1_Stream4, .Init.Channel = DMA_CHANNEL_4},
    .hdma_tx_IRQn = DMA1_Stream4_IRQn,

    .gpio_alternateFunction = GPIO_AF8_UART4,
    
    .isInitialized = false
  },
  [UART_5] = {
    .huart = {.Instance = UART5},
    .huart_IRQn = UART5_IRQn,

    .hdma_rx = {.Instance = DMA1_Stream0, .Init.Channel = DMA_CHANNEL_4},
    .hdma_tx = {.Instance = DMA1_Stream7, .Init.Channel = DMA_CHANNEL_4},
    .hdma_tx_IRQn = DMA1_Stream7_IRQn,

    .gpio_alternateFunction = GPIO_AF8_UART5,
    
    .isInitialized = false
  },
  [UART_6] = {
    .huart = {.Instance = USART6},
    .huart_IRQn = USART6_IRQn,

    .hdma_rx = {.Instance = DMA2_Stream1, .Init.Channel = DMA_CHANNEL_5},
    .hdma_tx = {.Instance = DMA2_Stream6, .Init.Channel = DMA_CHANNEL_5},
    .hdma_tx_IRQn = DMA2_Stream6_IRQn,

    .gpio_alternateFunction = GPIO_AF8_USART6,
    
    .isInitialized = false
  }
};

// Initialization-related function prototypes
static void uart_enable_rcc(uart_instance_t uart_instance);
static void uart_enable_dma(uart_instance_t uart_instance);
static void uart_enable_gpio(uart_instance_t uart_instance, gpio_pin_t pin_rx, gpio_pin_t pin_tx);
static void uart_enable_IRQn(uart_instance_t uart_instance);
static UART_InitTypeDef uart_get_initStruct(uart_instance_t uart_instance, uint32_t baudrate);

// Transmision-related function prototypes
static void uart_transmissionHandler(uart_instance_t uart_instance);
static void uart_errorHandler(UART_HandleTypeDef* huart);

void uart_init(const uart_init_t* uart_init, uart_t* uart_struct)
{
  uart_instance_t uart_instance = uart_init->uart_instance;
  uart_struct->uart_instance = uart_instance;

  if(uart_infoStructs[uart_instance].isInitialized || uart_instance > UART_6) {
    while(1); // UART instance already initialized or UART instance non-existent
  }

  uart_enable_rcc(uart_instance);
  uart_enable_dma(uart_instance);
  uart_enable_gpio(uart_instance, uart_init->pin_rx, uart_init->pin_tx);
  uart_enable_IRQn(uart_instance);

  uart_infoStructs[uart_instance].start_rx = 0;
  uart_infoStructs[uart_instance].first_tx = 0;
  uart_infoStructs[uart_instance].afterLast_tx = 0;
  uart_infoStructs[uart_instance].transmissionSize = 0;
  uart_infoStructs[uart_instance].buffer_rx = uart_struct->buffer_rx;
  uart_infoStructs[uart_instance].buffer_tx = uart_struct->buffer_tx;
  uart_infoStructs[uart_instance].onTransmission = false;
  
  UART_HandleTypeDef* huart = &uart_infoStructs[uart_instance].huart;
  huart->Init = uart_get_initStruct(uart_instance, uart_init->baudrate);
  HAL_UART_Init(huart);
  
  // Enable UART reception via DMA
  HAL_UART_Receive_DMA(huart, uart_infoStructs[uart_instance].buffer_rx, UART_RX_BUFFERSIZE);

  uart_infoStructs[uart_instance].isInitialized = true;
}

static void uart_enable_rcc(uart_instance_t uart_instance)
{
  if(uart_instance == UART_1) {
    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_DMA2_CLK_ENABLE();
  }
  else if(uart_instance == UART_2) {
    __HAL_RCC_USART2_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();
  }
  else if(uart_instance == UART_3) {
    __HAL_RCC_USART3_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();
  }
  else if(uart_instance == UART_4) {
    __HAL_RCC_UART4_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();
  }
  else if(uart_instance == UART_5) {
    __HAL_RCC_UART5_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();
  }
  else if(uart_instance == UART_6) {
    __HAL_RCC_USART6_CLK_ENABLE();
    __HAL_RCC_DMA2_CLK_ENABLE();
  }
}

static void uart_enable_dma(uart_instance_t uart_instance)
{
  UART_HandleTypeDef* huart = &uart_infoStructs[uart_instance].huart;

  // Enable DMA for TX
  HAL_NVIC_SetPriority(uart_infoStructs[uart_instance].hdma_tx_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(uart_infoStructs[uart_instance].hdma_tx_IRQn);
  
  DMA_HandleTypeDef* hdma_tx = &uart_infoStructs[uart_instance].hdma_tx;
  hdma_tx->Parent = huart; 
  hdma_tx->Init.Direction = DMA_MEMORY_TO_PERIPH;
  hdma_tx->Init.FIFOMode = DMA_FIFOMODE_DISABLE;
  hdma_tx->Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
  hdma_tx->Init.MemInc = DMA_MINC_ENABLE;
  hdma_tx->Init.Mode = DMA_NORMAL;
  hdma_tx->Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  hdma_tx->Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_tx->Init.Priority = DMA_PRIORITY_MEDIUM;
  
  HAL_DMA_Init(hdma_tx);
  huart->hdmatx = hdma_tx;

  // Enable DMA for RX
  DMA_HandleTypeDef* hdma_rx = &uart_infoStructs[uart_instance].hdma_rx;
  hdma_rx->Parent = huart; 
  hdma_rx->Init.Direction = DMA_PERIPH_TO_MEMORY;
  hdma_rx->Init.FIFOMode = DMA_FIFOMODE_DISABLE;
  hdma_rx->Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
  hdma_rx->Init.MemInc = DMA_MINC_ENABLE;
  hdma_rx->Init.Mode = DMA_CIRCULAR;
  hdma_rx->Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  hdma_rx->Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_rx->Init.Priority = DMA_PRIORITY_HIGH;
  
  HAL_DMA_Init(hdma_rx);
  huart->hdmarx = hdma_rx;
}

static void uart_enable_gpio(uart_instance_t uart_instance, gpio_pin_t pin_rx, gpio_pin_t pin_tx)
{
  uint8_t alternateFunction = uart_infoStructs[uart_instance].gpio_alternateFunction;
  
  gpio_pin_init_t gpio_init_UART_RX = {
    .pin = pin_rx,
    .mode = AF_PUSH_PULL,
    .pull = PULL_NONE,
    .speed = SPEED_VERY_HIGH,
    .alternateFunction = alternateFunction
  };
  gpio_pin_init(&gpio_init_UART_RX);

  gpio_pin_init_t gpio_init_UART_TX = {
    .pin = pin_tx,
    .mode = AF_PUSH_PULL,
    .pull = PULL_NONE,
    .speed = SPEED_VERY_HIGH,
    .alternateFunction = alternateFunction
  };
  gpio_pin_init(&gpio_init_UART_TX);
}

static void uart_enable_IRQn(uart_instance_t uart_instance)
{
  HAL_NVIC_SetPriority(uart_infoStructs[uart_instance].huart_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(uart_infoStructs[uart_instance].huart_IRQn);
}

static UART_InitTypeDef uart_get_initStruct(uart_instance_t uart_instance, uint32_t baudrate)
{
  UART_InitTypeDef initStruct;
  
  if(baudrate >= 1374 || baudrate <= 2812500) { // See RM0390, Section: 25.4.4
    initStruct.BaudRate = baudrate;
    initStruct.OverSampling = UART_OVERSAMPLING_16;

  }
  else if(baudrate > 5620000){ // See Datasheet (DS10693 REV10 Page: 33)
    initStruct.BaudRate = baudrate;
    initStruct.OverSampling = UART_OVERSAMPLING_8;
  }

  else {
    while(1); // Baudrate outside of allowed range
  }
  initStruct.HwFlowCtl = UART_HWCONTROL_NONE;
  initStruct.Mode = UART_MODE_TX_RX;
  initStruct.Parity = UART_PARITY_NONE;
  initStruct.StopBits = UART_STOPBITS_1;
  initStruct.WordLength = UART_WORDLENGTH_8B;

  return initStruct;
}

void uart_changeBaudrate(const uart_t *uart_struct, uint32_t baudrate)
{
  uart_instance_t uart_instance = uart_struct->uart_instance;

  if(uart_infoStructs[uart_instance].isInitialized)
  {
    UART_HandleTypeDef* huart = &uart_infoStructs[uart_instance].huart;
    
    HAL_UART_DMAStop(huart);

    uart_infoStructs[uart_instance].transmissionSize = 0;
    uart_infoStructs[uart_instance].onTransmission = false;
    uart_infoStructs[uart_instance].first_tx = 0;
    uart_infoStructs[uart_instance].afterLast_tx = 0;
    uart_infoStructs[uart_instance].start_rx = 0;

    if(baudrate >= 1374 || baudrate <= 2812500) { // See RM0390, Section: 25.4.4
      huart->Init.BaudRate = baudrate;
    }
    else {
      return; // Baudrate outside of allowed range
    }

    HAL_UART_Init(huart);
    HAL_UART_Receive_DMA(huart, uart_infoStructs[uart_instance].buffer_rx, UART_RX_BUFFERSIZE);
  }
  else {
    return; // UART instance was not initialized
  }
}

uint16_t uart_receive(const uart_t* uart_struct, const void* data, uint16_t dataSize)
{
  uart_instance_t UART_x = uart_struct->uart_instance;
  uint16_t start = uart_infoStructs[UART_x].start_rx;
  uint16_t end;
  uint8_t* buffer_rx = uart_infoStructs[UART_x].buffer_rx;
  DMA_HandleTypeDef* hdma_rx = &uart_infoStructs[UART_x].hdma_rx;
  uint16_t bytesToReceive;
  uint16_t bytesReceived = 0;
  
  end = UART_RX_BUFFERSIZE - hdma_rx->Instance->NDTR; // See RM0390, Section: 9.5.6
  if(end == start) {
      return 0; // No data for reception
  }
  
  // If the buffer has wrapped around, the data gets read until UART_RX_BUFFERSIZE
  // is reached or until dataSize bytes were read
  if(start > end)
  {
    bytesToReceive = UART_RX_BUFFERSIZE-start;
    if(bytesToReceive > dataSize)
    {
      memcpy(data, (void*)&buffer_rx[start], dataSize);
      bytesReceived += dataSize;
      start += dataSize;
    }
    else
    {
      memcpy(data, (void*)&buffer_rx[start], bytesToReceive);
      bytesReceived += bytesToReceive;
      start = 0;
    }
  }
  
  if(start < end)
  {
    bytesToReceive = end - start;
    if(bytesToReceive > dataSize-bytesReceived)
    {
      memcpy(&data[bytesReceived], (void*)&buffer_rx[start], dataSize-bytesReceived);
      start += dataSize-bytesReceived;
      bytesReceived += dataSize-bytesReceived;
    
    }
    else if(bytesToReceive != 0)
    {
      memcpy(&data[bytesReceived], (void*)&buffer_rx[start], bytesToReceive);
      bytesReceived += bytesToReceive;
      start += bytesToReceive;
    }
  }

  uart_infoStructs[UART_x].start_rx = start;
  
  return bytesReceived;
}

uint16_t uart_transmit(const uart_t* uart_struct, const void* data, uint16_t dataSize)
{
  const uint16_t first = uart_infoStructs[uart_struct->uart_instance].first_tx;
  uint16_t afterLast = uart_infoStructs[uart_struct->uart_instance].afterLast_tx;
  uint8_t* buffer_tx = uart_infoStructs[uart_struct->uart_instance].buffer_tx;
  uint16_t bytesToTransmit;
  uint16_t bytesTransmitted = 0;

  if(afterLast >= first)
  {
    bytesToTransmit = UART_TX_BUFFERSIZE - afterLast;
    if(bytesToTransmit > dataSize)
    {
      memcpy(&buffer_tx[afterLast], &data[bytesTransmitted], dataSize);
      bytesTransmitted += dataSize;
      afterLast += dataSize;
    }
    else
    {
      if(first == 0) {
        bytesToTransmit -= 1;
      }
      memcpy(&buffer_tx[afterLast], &data[bytesTransmitted], bytesToTransmit);
      bytesTransmitted += bytesToTransmit;
      afterLast += bytesToTransmit;
      if(afterLast == UART_TX_BUFFERSIZE) {
        afterLast = 0;
      }
    }
  }
  
  // If there is remaining data for transmission and still capacity in the buffer,
  // the remaining data gets transmitted
  if(bytesTransmitted < dataSize && first > (afterLast+1))
  {
    bytesToTransmit = first - afterLast - 1;
    if(bytesToTransmit > dataSize)
    {
      memcpy(&buffer_tx[afterLast], &data[bytesTransmitted], dataSize-bytesTransmitted);
      afterLast += dataSize-bytesTransmitted;
      bytesTransmitted += dataSize-bytesTransmitted;
    }
    else
    {
      memcpy(&buffer_tx[afterLast], &data[bytesTransmitted], dataSize-bytesToTransmit);
      bytesTransmitted += dataSize-bytesToTransmit;
      afterLast += dataSize-bytesToTransmit;
    }
  }

  uart_infoStructs[uart_struct->uart_instance].afterLast_tx = afterLast;
  
  // If a previous transmission is still ongoing, the TxCpltCallback will envoke
  // the uart_transmissionHandler after completion
  if(!uart_infoStructs[uart_struct->uart_instance].onTransmission) {
    uart_transmissionHandler(uart_struct->uart_instance);
  }

  return bytesTransmitted;
}

uint16_t uart_bytesReadable(const uart_t* uart_struct)
{
  uart_instance_t uart_instance = uart_struct->uart_instance;
  uint16_t start = uart_infoStructs[uart_instance].start_rx;
  uint16_t end;
  DMA_HandleTypeDef* hdma_rx = &uart_infoStructs[uart_instance].hdma_rx;

  end = UART_RX_BUFFERSIZE - hdma_rx->Instance->NDTR; // See RM0390, Section: 9.5.6

  if(start == end) {
    return 0; // No data for reception
  }
  else if(start < end) {
    return end - start;
  }
  else if(start > end) {
    return UART_RX_BUFFERSIZE - start + end; // Buffer has wrapped around
  }
}

uint16_t uart_bytesWritable(const uart_t* uart_struct)
{
  uart_instance_t uart_instance = uart_struct->uart_instance;
  uint16_t start = uart_infoStructs[uart_instance].first_tx;
  uint16_t end;
  DMA_HandleTypeDef* hdma_tx = &uart_infoStructs[uart_instance].hdma_tx;

  end = UART_TX_BUFFERSIZE - hdma_tx->Instance->NDTR; // See RM0390, Section: 9.5.6

  if(start == end) {
    return 0; // No data for reception
  }
  else if(start < end) {
    return end - start;
  }
  else if(start > end) {
    return UART_TX_BUFFERSIZE - start + end; // Buffer has wrapped around
  }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart)
{
  uart_instance_t uart_instance;

  if(huart == &uart_infoStructs[UART_1].huart) {
    uart_instance = UART_1;
  }
  else if(huart == &uart_infoStructs[UART_2].huart) {
    uart_instance = UART_2;
  }
  else if(huart == &uart_infoStructs[UART_3].huart) {
    uart_instance = UART_3;
  }
  else if(huart == &uart_infoStructs[UART_4].huart) {
    uart_instance = UART_4;
  }
  else if(huart == &uart_infoStructs[UART_5].huart) {
    uart_instance = UART_5;
  }
  else if(huart == &uart_infoStructs[UART_6].huart) {
    uart_instance = UART_6;
  }

  uart_infoStructs[uart_instance].first_tx += uart_infoStructs[uart_instance].transmissionSize;
  if(uart_infoStructs[uart_instance].first_tx >= UART_TX_BUFFERSIZE) {
    uart_infoStructs[uart_instance].first_tx -= UART_TX_BUFFERSIZE;
  }
  uart_infoStructs[uart_instance].transmissionSize = 0;
  uart_infoStructs[uart_instance].onTransmission = false;

  uart_transmissionHandler(uart_instance);
}

static void uart_transmissionHandler(uart_instance_t uart_instance)
{
  uint16_t first = uart_infoStructs[uart_instance].first_tx;
  uint16_t afterLast = uart_infoStructs[uart_instance].afterLast_tx;
  uint8_t* buffer_tx = uart_infoStructs[uart_instance].buffer_tx;
  UART_HandleTypeDef* huart = &uart_infoStructs[uart_instance].huart;
  uint16_t bytesForTransmission = 0;
  
  if(first == afterLast) {
    return; // No data for transmission
  }
  else if(first > afterLast) {
    bytesForTransmission = UART_TX_BUFFERSIZE - first;
  }
  else if(first < afterLast) {
    bytesForTransmission = afterLast - first;
  }
  
  uart_infoStructs[uart_instance].transmissionSize = bytesForTransmission;
  uart_infoStructs[uart_instance].onTransmission = true;

  HAL_UART_Transmit_DMA(huart, &buffer_tx[first], bytesForTransmission);
}

static void uart_errorHandler(UART_HandleTypeDef* huart) {
  uint32_t dummyReadReg;  

  // Check for PE (parity error) or FE (frame error) or ORE (overrun error) or NE (noise error)
  if(huart->Instance->SR & (uint32_t)(USART_SR_PE | USART_SR_FE | USART_SR_ORE | USART_SR_NE))
  {
    // Clear PE, FE, ORE and NE flag (see RM0390, Section: 25.6.1)
    dummyReadReg = huart->Instance->SR;
    dummyReadReg = huart->Instance->DR;
  }
    
  HAL_UART_IRQHandler(huart);
}

// Interrupt request handler for UART_1
void DMA2_Stream7_IRQHandler(void) {
  HAL_DMA_IRQHandler(&uart_infoStructs[UART_1].hdma_tx);
}
void USART1_IRQHandler(void) {
  uart_errorHandler(&uart_infoStructs[UART_1].huart);
}

// Interrupt request handler for UART_2
void DMA1_Stream6_IRQHandler(void) {
  HAL_DMA_IRQHandler(&uart_infoStructs[UART_2].hdma_tx);
}
void USART2_IRQHandler(void) {
  uart_errorHandler(&uart_infoStructs[UART_2].huart);
}

// Interrupt request handler for UART_3
void DMA1_Stream3_IRQHandler(void) {
  HAL_DMA_IRQHandler(&uart_infoStructs[UART_3].hdma_tx);
}
void USART3_IRQHandler(void) {
  uart_errorHandler(&uart_infoStructs[UART_3].huart);
}

// Interrupt request handler for UART_4
void DMA1_Stream4_IRQHandler(void) {
  HAL_DMA_IRQHandler(&uart_infoStructs[UART_4].hdma_tx);
}
void UART4_IRQHandler(void) {
  uart_errorHandler(&uart_infoStructs[UART_4].huart);
}

// Interrupt request handler for UART_5
void DMA1_Stream7_IRQHandler(void) {
  HAL_DMA_IRQHandler(&uart_infoStructs[UART_5].hdma_tx);
}
void UART5_IRQHandler(void) {
  uart_errorHandler(&uart_infoStructs[UART_5].huart);
}

// Interrupt request handler for UART_6
void DMA2_Stream6_IRQHandler(void) {
  HAL_DMA_IRQHandler(&uart_infoStructs[UART_6].hdma_tx);
}
void USART6_IRQHandler(void) {
  uart_errorHandler(&uart_infoStructs[UART_6].huart);
}


