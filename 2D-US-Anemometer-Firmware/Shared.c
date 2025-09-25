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
#include "DAC_AD5732.h"
#include "gpio.h"
#include "stm32f4xx_hal.h"
#include "stdio.h"

extern SPI_HandleTypeDef hspi3;
extern DMA_HandleTypeDef hdma_spi3_tx;
extern DMA_HandleTypeDef hdma_spi3_rx;
extern SPI_HandleTypeDef DAC_SPI_HANDLE;

extern uint8_t TRIGGER_EVENT_FLAG;


TIM_HandleTypeDef htim2;


gpio_pin_init_t gpio_trigger;
gpio_pin_t LED_0;
gpio_pin_t LED_1;
gpio_pin_t PE10;


void Error_Handler(uint8_t Error);

/**
  SPI Init Stuff
*/
void HAL_SPI_MspInit(SPI_HandleTypeDef* spiHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(spiHandle->Instance==SPI1)
  {
    /* SPI1 clock enable */
    __HAL_RCC_SPI1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  }
  else if(spiHandle->Instance==SPI3)
  {
    /* SPI3 clock enable */
    __HAL_RCC_SPI3_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    hdma_spi3_tx.Instance = DMA1_Stream5;
    hdma_spi3_tx.Init.Channel = DMA_CHANNEL_0;
    hdma_spi3_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_spi3_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_spi3_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_spi3_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_spi3_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_spi3_tx.Init.Mode = DMA_NORMAL;
    hdma_spi3_tx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_spi3_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_spi3_tx) != HAL_OK)
    {
      Error_Handler(3);
    }

    __HAL_LINKDMA(spiHandle,hdmatx,hdma_spi3_tx);

    hdma_spi3_rx.Instance = DMA1_Stream0;
    hdma_spi3_rx.Init.Channel = DMA_CHANNEL_0;
    hdma_spi3_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_spi3_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_spi3_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_spi3_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_spi3_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_spi3_rx.Init.Mode = DMA_NORMAL;
    hdma_spi3_rx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_spi3_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_spi3_rx) != HAL_OK)
    {
      Error_Handler(3);
    }
    __HAL_LINKDMA(spiHandle,hdmarx,hdma_spi3_rx);
  }
}

/**
  Clock, Bus, Tim and Trigger Pin Init 
*/
void System_clock_Init() {

  HAL_Init();

  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 360;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 8;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler(1);
  }

  if (HAL_PWREx_EnableOverDrive() != HAL_OK) {
    Error_Handler(1);
  }

  // Clock for the Buses
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
    Error_Handler(1);
  }

  //TIM2 for Time Messurement 1us per CNT
  __HAL_RCC_TIM2_CLK_ENABLE();
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 89;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.RepetitionCounter = 0;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler(1);
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler(1);
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler(1);
  }

  if(HAL_TIM_Base_Start(&htim2)!=HAL_OK){
    printf("TIM2 Error on startup ! \n");
  }

  //Trigger Init
  gpio_trigger.pin.bank = BANK_C;
  gpio_trigger.pin.num = 7;
  gpio_trigger.mode = INPUT_IT_RISING;
  gpio_trigger.pull = PULL_NONE;
  gpio_trigger.speed = SPEED_HIGH;
  gpio_pin_init(&gpio_trigger);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  //LED_0 PB8
  gpio_pin_init_t LED_init = {};
  LED_init.pin.bank = BANK_B;
  LED_init.pin.num = 8;
  LED_init.mode =  OUTPUT_PUSH_PULL;
  LED_init.pull = PULL_NONE;
  LED_init.speed = SPEED_LOW;
  gpio_pin_init(&LED_init);

  LED_0.bank = BANK_B;
  LED_0.num = 8;

  //LED_1 PB9
  LED_init.pin.bank = BANK_B;
  LED_init.pin.num = 9;
  LED_init.mode =  OUTPUT_PUSH_PULL;
  LED_init.pull = PULL_NONE;
  LED_init.speed = SPEED_LOW;
  gpio_pin_init(&LED_init);  gpio_pin_init(&LED_init);

  LED_1.bank = BANK_B;
  LED_1.num = 9;

  //Start DMAs
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();
}


//Interupt für PC7
uint8_t toggle_state = 0;
void EXTI9_5_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_7);
  //Setting flag for main process
  TRIGGER_EVENT_FLAG = 1;

  //Toggle LED 
  if(toggle_state == 0){
    toggle_state = 1;
    gpio_pin_write(LED_0, 1);
  }
  else {
    toggle_state = 0;
    gpio_pin_write(LED_0, 0);
  }
}



void Warning_Handler(uint8_t Warning_code){
  printf("Warning !\n");
  switch(Warning_code){
    case 1:
       printf("System_clock_Init() excited with Warning\n"); 
    break;
        case 2:
       printf("DAC_Init() excited with Warning\n"); 
    break;
        case 3:
       printf("RAM_Init() excited with Warning\n"); 
    break;
            case 4:
        printf("ADC_Init() excited with Warning\n"); 
    break;
        case 5:
        printf("Filter_Init() excited with Warning\n"); 
    break;
    case 7:
        printf("Bad match deteced !\n"); 
    break;


    default:
        printf("Unknown Warning occurred!\n");
    break;
  }
}


//Print in to serial as well !!!

void Error_Handler(uint8_t Error_code){
  printf("Due to a critical error the sensor has been stopped !\n");
  switch(Error_code){
    case 1:
       printf("System_clock_Init() FAILED\n"); 
    break;
        case 2:
       printf("DAC_Init() FAILED\n"); 
    break;
        case 3:
       printf("RAM_Init() FAILED\n"); 
    break;
        case 4:
        printf("ADC_Init() FAILED\n"); 
    break;
        case 5:
        printf("Filter_Init() FAILED\n"); 
    break;
        case 6:
        printf("Transceiver_Init_test FAILED\n"); 
    break;

    default:
        printf("Unknown Error occurred!\n");
    break;
  }
  while(1){
      //Disko machen !!!
      gpio_pin_write(LED_1,0);
      gpio_pin_write(LED_0,1);
      HAL_Delay(250);
      gpio_pin_write(LED_1,1);
      gpio_pin_write(LED_0,0);
       HAL_Delay(250);

  }
}


//Systick interupt
void SysTick_Handler(void)
{
  HAL_IncTick();
}