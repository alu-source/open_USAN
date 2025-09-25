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
#include "Shot.h"
#include "stm32f4xx_hal_dma.h"
#include "stm32f4xx_hal_adc.h"
#include "Shared.h"
#include "stdio.h"

TIM_HandleTypeDef htim1;
DMA_HandleTypeDef hdma_tim1_up;
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;


/**
  GPIO set up 
*/
 uint8_t Shot_Init(Transceiver_handle* handle){
    // TIM1 Config, 800kHz Update Rate, Triggers DMA2 Request
    __HAL_RCC_TIM1_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    
    htim1.Instance = TIM1;
    htim1.Init.Prescaler = 74;
    htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim1.Init.Period = 2;
    htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    HAL_TIM_Base_Init(&htim1);
    
    TIM_ClockConfigTypeDef sClockSourceConfig;
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig);
    
    TIM_MasterConfigTypeDef sMasterConfig;
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig);
    
    
    __HAL_TIM_ENABLE_DMA(&htim1, TIM_DMA_UPDATE);
    
    TIM_OC_InitTypeDef sConfigOC;
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 1;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1);

    //Start Timer
    if(HAL_TIM_Base_Start(&htim1)!=HAL_OK){
      printf("TIM 1 Start failed!\n");
    }

    // DMA2 Config, Request by TIM1 Update
    
    __HAL_RCC_DMA2_CLK_ENABLE();
    
    hdma_tim1_up.Instance = DMA2_Stream5;
    hdma_tim1_up.Init.Channel = DMA_CHANNEL_6;
    hdma_tim1_up.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_tim1_up.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_tim1_up.Init.MemInc = DMA_MINC_ENABLE;
    hdma_tim1_up.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_tim1_up.Init.MemDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_tim1_up.Init.Mode = DMA_NORMAL;
    hdma_tim1_up.Init.Priority = DMA_PRIORITY_LOW;
    hdma_tim1_up.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    
    __HAL_LINKDMA(&htim1,hdma[TIM_DMA_ID_UPDATE],hdma_tim1_up);



  //GPIO_Init

  GPIO_InitTypeDef initStruct;
  initStruct.Mode = GPIO_MODE_OUTPUT_PP;
  initStruct.Speed = GPIO_SPEED_FREQ_LOW;
  initStruct.Pull = GPIO_NOPULL;
  initStruct.Pin = SHOT_TR0_H | SHOT_TR0_L | SHOT_TR0_C |
                   SHOT_TR1_H | SHOT_TR1_L | SHOT_TR1_C |
                   SHOT_TR2_H | SHOT_TR2_L | SHOT_TR2_C |
                   SHOT_TR3_H | SHOT_TR3_L | SHOT_TR3_C;

  HAL_GPIO_Init(SHOT_PORT,&initStruct);

  //Fill GPIO bit masks into struct
  handle[0].TR_H = SHOT_TR0_H;
  handle[0].TR_L = SHOT_TR0_L;
  handle[0].TR_C = SHOT_TR0_C | SHOT_TR3_C;
  handle[0].TR_CC = SHOT_TR0_C;
  handle[0].TR_A = SHOT_TR3_A;


  handle[1].TR_H = SHOT_TR1_H;
  handle[1].TR_L = SHOT_TR1_L;
  handle[1].TR_C = SHOT_TR1_C | SHOT_TR2_C;
  handle[1].TR_CC = SHOT_TR1_C;
  handle[1].TR_A = SHOT_TR2_A;


  handle[2].TR_H = SHOT_TR3_H;
  handle[2].TR_L = SHOT_TR3_L;
  handle[2].TR_C = SHOT_TR0_C | SHOT_TR3_C;
  handle[2].TR_CC = SHOT_TR3_C;
  handle[2].TR_A = SHOT_TR0_A;


  handle[3].TR_H = SHOT_TR2_H;
  handle[3].TR_L = SHOT_TR2_L;
  handle[3].TR_C = SHOT_TR1_C | SHOT_TR2_C;
  handle[3].TR_CC = SHOT_TR2_C;
  handle[3].TR_A = SHOT_TR1_A;


  // Preperare ADC
  __HAL_RCC_ADC1_CLK_ENABLE();

  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T1_CC1;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK) {
    Error_Handler(4);
  }


  __HAL_RCC_GPIOA_CLK_ENABLE();

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  //ADC DMA setup
  hdma_adc1.Instance = DMA2_Stream0;
  hdma_adc1.Init.Channel = DMA_CHANNEL_0;
  hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
  hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;
  hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
  hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
  hdma_adc1.Init.Mode = DMA_NORMAL;
  hdma_adc1.Init.Priority = DMA_PRIORITY_LOW; //Changed
  hdma_adc1.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
  HAL_DMA_Init(&hdma_adc1);

  __HAL_LINKDMA(&hadc1, DMA_Handle, hdma_adc1);

  return EXIT_SUCCESS;
}



 __attribute__((always_inline)) inline uint8_t Shot_fire(Transceiver_handle *handle, Shot_data_struct *data) {
  data->lock = 1; // Lock data

  if(hdma_tim1_up.Instance->NDTR != 0){
    __NOP(); // Caused by to tight shot timing 
  }

  // Reset Timer and Set up Shot
  HAL_TIM_OC_Stop(&htim1, TIM_CHANNEL_1);
  HAL_TIM_Base_Stop(&htim1);
  HAL_ADC_Stop_DMA(&hadc1);
  //HAL_DMA_DeInit(&hdma_tim1_up);

  // Setting gpio Pattern up
  HAL_DMA_Init(&hdma_tim1_up);
  HAL_DMA_Start(&hdma_tim1_up, (uint32_t)(handle->gpio_data), ((uint32_t) & (SHOT_PORT->ODR)),SHOT_DMA_LENGTH); // Write Port register directly with DMA

  // Setting up ADC
  ADC_ChannelConfTypeDef sConfig;
  sConfig.Channel = handle->TR_A;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

  // Begin Countdown
  __HAL_TIM_SET_COUNTER(&htim1, 0);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)data->data, SHOT_DMA_LENGTH + SHOT_ADC_OFFSET);

  // Lift off
  HAL_TIM_OC_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_Base_Start(&htim1);

  return EXIT_SUCCESS;
}

 __attribute__((always_inline)) inline uint8_t Shot_unlock_data(Shot_data_struct *data) {
  if (hdma_adc1.Instance->NDTR != 0) { // If more transvers are Pending
    __NOP();
    return EXIT_FAILURE;
  } else {
    data->lock = 0;
  }
  return EXIT_SUCCESS;
}

uint8_t Shot_generate_gpio_from_Key(uint8_t *KEY, Transceiver_handle * handle){

  // For each TR
  for (uint8_t j = 0; j < 4; j++) { 
    // Erase Buffers and Activate J-Fet (Reset == J-Fet off) for not used Channels
    for (uint16_t ii = 0; ii < SHOT_DMA_LENGTH; ii++) {
         handle[j].gpio_data[ii] = handle[j].TR_C; //J-FET is set so F-FET is off
    }
    
    //Acitvate Mosfets according to KEY on Periode at a time
    for (uint8_t i = 0; i < SHOT_KEY_LENGTH; i++) {
      //One Key Token at a time 
      if (KEY[i] == 0) {
        //first half
        for (uint16_t ii = i*SHOT_DMA_CYCLES_PER_PERIODE +1; ii < SHOT_DMA_CYCLES_PER_PERIODE / 2 + i*SHOT_DMA_CYCLES_PER_PERIODE; ii++) {
          if(ii % SHOT_POW_MODULO == 1){
            handle[j].gpio_data[ii] = handle[j].gpio_data[ii] | handle[j].TR_H;
          }

        }
        //second half
        for (uint16_t ii = i*SHOT_DMA_CYCLES_PER_PERIODE + SHOT_DMA_CYCLES_PER_PERIODE / 2 +1; ii < SHOT_DMA_CYCLES_PER_PERIODE + i*SHOT_DMA_CYCLES_PER_PERIODE; ii++) {
          if(ii % SHOT_POW_MODULO == 1){
            handle[j].gpio_data[ii] = handle[j].gpio_data[ii] | handle[j].TR_L;
          }

        }
      } else {
        //first half
        for (uint16_t ii =  i*SHOT_DMA_CYCLES_PER_PERIODE +1; ii < SHOT_DMA_CYCLES_PER_PERIODE / 2 + i*SHOT_DMA_CYCLES_PER_PERIODE; ii++) {
          if(ii % SHOT_POW_MODULO == 1){
            handle[j].gpio_data[ii] = handle[j].gpio_data[ii] | handle[j].TR_L;
          }

         
        }
        //second half
        for (uint16_t ii =i*SHOT_DMA_CYCLES_PER_PERIODE + SHOT_DMA_CYCLES_PER_PERIODE / 2 +1; ii < SHOT_DMA_CYCLES_PER_PERIODE + i*SHOT_DMA_CYCLES_PER_PERIODE; ii++) {
          if(ii % SHOT_POW_MODULO == 1){
            handle[j].gpio_data[ii] = handle[j].gpio_data[ii] | handle[j].TR_H;
          }

        }
      }
    }

   

   
    //For the rest Activate J-FET for Transmitter and Deactivate all Mosfet
    if(SHOT_KEY_LENGTH * SHOT_DMA_CYCLES_PER_PERIODE >= SHOT_DMA_LENGTH){
      return EXIT_FAILURE; //Key to long
    }
    for(uint16_t i = SHOT_KEY_LENGTH * SHOT_DMA_CYCLES_PER_PERIODE; i < SHOT_DMA_LENGTH; i++){
      handle[j].gpio_data[i] =  handle[j].gpio_data[i] &~ handle[j].TR_CC;
    }


  }
 
  return EXIT_SUCCESS;
}


/*
  Experimental GPIO sequence generators some are broken ! 
*/

//Sub-Periode Sequences:
uint8_t LP_Profile[] =  {0,1,0,0,1,1,0,0,1,0};
uint8_t HP_Profile[] =  {0,1,0,1,1,1,1,0,1,0};
uint8_t PC1_Profile[] = {0,1,1,1,1,1,1,1,1,1};
uint8_t PC2_Profile[] = {1,1,1,1,1,1,1,1,1,0};



uint8_t Shot_generate_gpio_from_Key_2(uint8_t *KEY, Transceiver_handle *handle) {
  for (uint8_t j = 0; j < 4; j++) {
    for (uint16_t ii = 0; ii < SHOT_DMA_LENGTH; ii++) {
      handle[j].gpio_data[ii] = handle[j].TR_C; // J-FET is set so F-FET is off
    }

    // Key 1
    for (uint8_t ii = 0; ii < SHOT_DMA_CYCLES_PER_PERIODE / 2; ii++) {
      if (KEY[0] == 0) {
       handle[j].gpio_data[ii] = handle[j].gpio_data[ii] | HP_Profile[ii] * handle[j].TR_H;
      } else {
       handle[j].gpio_data[ii] = handle[j].gpio_data[ii] | HP_Profile[ii] * handle[j].TR_L;
      }
    }
    for (uint8_t ii = SHOT_DMA_CYCLES_PER_PERIODE / 2; ii < SHOT_DMA_CYCLES_PER_PERIODE; ii++) {
      if (KEY[0] == 0) {
       handle[j].gpio_data[ii] = handle[j].gpio_data[ii] | HP_Profile[ii] * handle[j].TR_L;
      } else {
       handle[j].gpio_data[ii] = handle[j].gpio_data[ii] | HP_Profile[ii] * handle[j].TR_H;
      }
    }

    for (uint8_t i = 1; i < SHOT_KEY_LENGTH; i++) {
      uint16_t buffer_offset = i * SHOT_DMA_CYCLES_PER_PERIODE;
      if (KEY[i - 1] == KEY[i] && KEY[i + 1] == KEY[i]) {
       for (uint8_t ii = 0; ii < SHOT_DMA_CYCLES_PER_PERIODE / 2; ii++) {
          if (KEY[i] == 0) {
            handle[j].gpio_data[ii + buffer_offset] = handle[j].gpio_data[ii + buffer_offset] | HP_Profile[ii] * handle[j].TR_H;
          } else {
            handle[j].gpio_data[ii + buffer_offset] = handle[j].gpio_data[ii + buffer_offset] | HP_Profile[ii] * handle[j].TR_L;
          }
       }
       for (uint8_t ii = SHOT_DMA_CYCLES_PER_PERIODE / 2; ii < SHOT_DMA_CYCLES_PER_PERIODE; ii++) {
          if (KEY[i] == 0) {
            handle[j].gpio_data[ii + buffer_offset] = handle[j].gpio_data[ii + buffer_offset] | HP_Profile[ii - SHOT_DMA_CYCLES_PER_PERIODE / 2] * handle[j].TR_L;
          } else {
            handle[j].gpio_data[ii + buffer_offset] = handle[j].gpio_data[ii + buffer_offset] | HP_Profile[ii - SHOT_DMA_CYCLES_PER_PERIODE / 2] * handle[j].TR_H;
          }
       }

      } else {
             for (uint8_t ii = 0; ii < SHOT_DMA_CYCLES_PER_PERIODE / 2; ii++) {
          if (KEY[i] == 0) {
            handle[j].gpio_data[ii + buffer_offset] = handle[j].gpio_data[ii + buffer_offset] | LP_Profile[ii] * handle[j].TR_H;
          } else {
            handle[j].gpio_data[ii + buffer_offset] = handle[j].gpio_data[ii + buffer_offset] | LP_Profile[ii] * handle[j].TR_L;
          }
       }
       for (uint8_t ii = SHOT_DMA_CYCLES_PER_PERIODE / 2; ii < SHOT_DMA_CYCLES_PER_PERIODE; ii++) {
          if (KEY[i] == 0) {
            handle[j].gpio_data[ii + buffer_offset] = handle[j].gpio_data[ii + buffer_offset] | LP_Profile[ii - SHOT_DMA_CYCLES_PER_PERIODE / 2] * handle[j].TR_L;
          } else {
            handle[j].gpio_data[ii + buffer_offset] = handle[j].gpio_data[ii + buffer_offset] | LP_Profile[ii - SHOT_DMA_CYCLES_PER_PERIODE / 2] * handle[j].TR_H;
          }
       }
      }
    }
  
  //For the rest Activate J-FET for Transmitter and Deactivate all Mosfet
    if(SHOT_KEY_LENGTH * SHOT_DMA_CYCLES_PER_PERIODE >= SHOT_DMA_LENGTH){
      return EXIT_FAILURE; //Key to long
    }
    //for(uint16_t i = SHOT_KEY_LENGTH * SHOT_DMA_CYCLES_PER_PERIODE; i < SHOT_DMA_LENGTH; i++){
   
    //  handle[j].gpio_data[i] =  handle[j].gpio_data[i] &~ handle[j].TR_CC;
    //}
    }
}






uint8_t Shot_generate_gpio_from_Key_3(uint8_t *KEY, Transceiver_handle *handle) {
    for (uint8_t j = 0; j < 4; j++) {
      for (uint16_t ii = 0; ii < SHOT_DMA_LENGTH; ii++) {
        handle[j].gpio_data[ii] = handle[j].TR_C; // J-FET is set so F-FET is off
      }

      // Key 1
      for (uint8_t ii = 0; ii < SHOT_DMA_CYCLES_PER_PERIODE / 2; ii++) {
        if (KEY[0] == 0) {
         handle[j].gpio_data[ii] = handle[j].gpio_data[ii] | HP_Profile[ii] * handle[j].TR_H;
        } else {
         handle[j].gpio_data[ii] = handle[j].gpio_data[ii] | HP_Profile[ii] * handle[j].TR_L;
        }
      }
      for (uint8_t ii = SHOT_DMA_CYCLES_PER_PERIODE / 2; ii < SHOT_DMA_CYCLES_PER_PERIODE; ii++) {
        if (KEY[0] == 0) {
         handle[j].gpio_data[ii] = handle[j].gpio_data[ii] | HP_Profile[ii] * handle[j].TR_L;
        } else {
         handle[j].gpio_data[ii] = handle[j].gpio_data[ii] | HP_Profile[ii] * handle[j].TR_H;
        }
      }

      //For the rest
      for (uint8_t i = 1; i < SHOT_KEY_LENGTH; i++) {
        uint16_t buffer_offset = i * SHOT_DMA_CYCLES_PER_PERIODE;

        //If next Phase Changes
        if (KEY[i + 1] != KEY[i]){                   
         for (uint8_t ii = 0; ii < SHOT_DMA_CYCLES_PER_PERIODE / 2; ii++) {
            if (KEY[i] == 0) {
              handle[j].gpio_data[ii + buffer_offset] = handle[j].gpio_data[ii + buffer_offset] | PC1_Profile[ii] * handle[j].TR_H;
            } else {
              handle[j].gpio_data[ii + buffer_offset] = handle[j].gpio_data[ii + buffer_offset] | PC1_Profile[ii] * handle[j].TR_L;
            }
         }
         for (uint8_t ii = SHOT_DMA_CYCLES_PER_PERIODE / 2; ii < SHOT_DMA_CYCLES_PER_PERIODE; ii++) {
            if (KEY[i] == 0) {
              handle[j].gpio_data[ii + buffer_offset] = handle[j].gpio_data[ii + buffer_offset] | PC1_Profile[ii - SHOT_DMA_CYCLES_PER_PERIODE / 2] * handle[j].TR_L;
            } else {
              handle[j].gpio_data[ii + buffer_offset] = handle[j].gpio_data[ii + buffer_offset] | PC1_Profile[ii - SHOT_DMA_CYCLES_PER_PERIODE / 2] * handle[j].TR_H;
            }
         }

        }
      
        //If prev. Phase Changed
        else if (KEY[i - 1] != KEY[i] ) {
         for (uint8_t ii = 0; ii < SHOT_DMA_CYCLES_PER_PERIODE / 2; ii++) {
            if (KEY[i] == 0) {
              handle[j].gpio_data[ii + buffer_offset] = handle[j].gpio_data[ii + buffer_offset] | PC2_Profile[ii] * handle[j].TR_H;
            } else {
              handle[j].gpio_data[ii + buffer_offset] = handle[j].gpio_data[ii + buffer_offset] | PC2_Profile[ii] * handle[j].TR_L;
            }
         }
         for (uint8_t ii = SHOT_DMA_CYCLES_PER_PERIODE / 2; ii < SHOT_DMA_CYCLES_PER_PERIODE; ii++) {
            if (KEY[i] == 0) {
              handle[j].gpio_data[ii + buffer_offset] = handle[j].gpio_data[ii + buffer_offset] | PC2_Profile[ii - SHOT_DMA_CYCLES_PER_PERIODE / 2] * handle[j].TR_L;
            } else {
              handle[j].gpio_data[ii + buffer_offset] = handle[j].gpio_data[ii + buffer_offset] | PC2_Profile[ii - SHOT_DMA_CYCLES_PER_PERIODE / 2] * handle[j].TR_H;
            }
         }

      } 
      
      //If Phase stays the Same
      else {
       for (uint8_t ii = 0; ii < SHOT_DMA_CYCLES_PER_PERIODE / 2; ii++) {
          if (KEY[i] == 0) {
            handle[j].gpio_data[ii + buffer_offset] = handle[j].gpio_data[ii + buffer_offset] | HP_Profile[ii] * handle[j].TR_H;
          } else {
            handle[j].gpio_data[ii + buffer_offset] = handle[j].gpio_data[ii + buffer_offset] | HP_Profile[ii] * handle[j].TR_L;
          }
       }
       for (uint8_t ii = SHOT_DMA_CYCLES_PER_PERIODE / 2; ii < SHOT_DMA_CYCLES_PER_PERIODE; ii++) {
          if (KEY[i] == 0) {
            handle[j].gpio_data[ii + buffer_offset] = handle[j].gpio_data[ii + buffer_offset] | HP_Profile[ii - SHOT_DMA_CYCLES_PER_PERIODE / 2] * handle[j].TR_L;
          } else {
            handle[j].gpio_data[ii + buffer_offset] = handle[j].gpio_data[ii + buffer_offset] | HP_Profile[ii - SHOT_DMA_CYCLES_PER_PERIODE / 2] * handle[j].TR_H;
          }
       }
      }
    }

    // For the rest Activate J-FET for Transmitter and Deactivate all Mosfet
    if (SHOT_KEY_LENGTH * SHOT_DMA_CYCLES_PER_PERIODE >= SHOT_DMA_LENGTH) {
      return EXIT_FAILURE; // Key to long
    }
    }
}

#define PULSE_PHASE_1 3
#define PULSE_PHASE_2 5
#define PULSE_PHASE_3 4

#define PULSE_SHIFT 10//SHOT_DMA_CYCLES_PER_PERIODE/2

uint8_t PULSE_P1[] = {0,1,0,1,0,1,1,0,1,0};
uint8_t PULSE_P2[] = {0,1,1,1,0,0,1,1,1,0};
uint8_t PULSE_P3[] = {0,1,0,1,1,1,1,0,1,0};

uint8_t Shot_generate_gpio_from_Key_4(uint8_t *KEY, Transceiver_handle *handle) {
    for (uint8_t j = 0; j < 4; j++) {

      //Turn the J-Fets off
      for (uint16_t ii = 0; ii < SHOT_DMA_LENGTH; ii++) {
        handle[j].gpio_data[ii] = handle[j].TR_C; // J-FET is set so F-FET is off
      }

      uint16_t cur_buffer_pos = 0; 
      //Phase 1
      for(uint8_t i = 0; i < PULSE_PHASE_1; i++){
        //first halfwave 
        for(uint16_t ii = 0; ii < SHOT_DMA_CYCLES_PER_PERIODE/2; ii++){
          handle[j].gpio_data[ii + cur_buffer_pos] = handle[j].gpio_data[ii + cur_buffer_pos] | PULSE_P1[ii] * handle[j].TR_H;
        }
        cur_buffer_pos += SHOT_DMA_CYCLES_PER_PERIODE/2;
        //second halfwave
        for(uint16_t ii = 0; ii < SHOT_DMA_CYCLES_PER_PERIODE/2; ii++){
          handle[j].gpio_data[ii + cur_buffer_pos] = handle[j].gpio_data[ii + cur_buffer_pos] | PULSE_P1[ii] * handle[j].TR_L;
        }
        cur_buffer_pos += SHOT_DMA_CYCLES_PER_PERIODE/2;
      }

      //Dampen between Phases 
      for (uint16_t ii = 0; ii < PULSE_SHIFT; ii++) {
        handle[j].gpio_data[ii + cur_buffer_pos] = handle[j].gpio_data[ii + cur_buffer_pos] & ~handle[j].TR_CC;
      }
      cur_buffer_pos += PULSE_SHIFT;



      //Phase 2
      for(uint8_t i = 0; i < PULSE_PHASE_2; i++){
        //first halfwave 
        for(uint16_t ii = 0; ii < SHOT_DMA_CYCLES_PER_PERIODE/2; ii++){
          handle[j].gpio_data[ii + cur_buffer_pos] = handle[j].gpio_data[ii + cur_buffer_pos] | PULSE_P2[ii] * handle[j].TR_H;
        }
        cur_buffer_pos += SHOT_DMA_CYCLES_PER_PERIODE/2;
        //second halfwave
        for(uint16_t ii = 0; ii < SHOT_DMA_CYCLES_PER_PERIODE/2; ii++){
          handle[j].gpio_data[ii + cur_buffer_pos] = handle[j].gpio_data[ii + cur_buffer_pos] | PULSE_P2[ii] * handle[j].TR_L;
        }
        cur_buffer_pos += SHOT_DMA_CYCLES_PER_PERIODE/2;
      }

      //Dampen between Phases 
      for (uint16_t ii = 0; ii < PULSE_SHIFT; ii++) {
        handle[j].gpio_data[ii + cur_buffer_pos] = handle[j].gpio_data[ii + cur_buffer_pos] & ~handle[j].TR_CC;
      }
      cur_buffer_pos += PULSE_SHIFT;

      //Phase 3
      for(uint8_t i = 0; i < PULSE_PHASE_3; i++){
        //first halfwave 
        for(uint16_t ii = 0; ii < SHOT_DMA_CYCLES_PER_PERIODE/2; ii++){
          handle[j].gpio_data[ii + cur_buffer_pos] = handle[j].gpio_data[ii + cur_buffer_pos] | PULSE_P2[ii] * handle[j].TR_H;
        }
        cur_buffer_pos += SHOT_DMA_CYCLES_PER_PERIODE/2;
        //second halfwave
        for(uint16_t ii = 0; ii < SHOT_DMA_CYCLES_PER_PERIODE/2; ii++){
          handle[j].gpio_data[ii + cur_buffer_pos] = handle[j].gpio_data[ii + cur_buffer_pos] | PULSE_P2[ii] * handle[j].TR_L;
        }
        cur_buffer_pos += SHOT_DMA_CYCLES_PER_PERIODE/2;
      }
    }
    return EXIT_SUCCESS;
}
