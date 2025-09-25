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

#include <stdio.h>
#include "Unittest.h"
#include "gpio.h"
#include "Shared.h"
#include "debug.h"
#include "measurement.h"
#include "Sensor_config.h"



uint8_t TRIGGER_EVENT_FLAG = 0;
uint8_t TRIGGER_MODE_FLAG = 0;
uint8_t ASCII_FLAG = 0;
float vel[2];

extern gpio_pin_t LED_0;
extern gpio_pin_t LED_1;
extern uart_t USART;
extern uint32_t trigger_timeout_value;
extern TIM_HandleTypeDef htim2;
 


/*********************************************************************
*
*       main()
* 
*  Function description
*   Application entry point.
*/
int main(void) {

  unit_startup();

  __NOP();

  #ifdef KEY_TEST
    debug_standalone_key_test();
  #endif

  TRIGGER_EVENT_FLAG = 0;

  if(TRIGGER_MODE_FLAG == 1){
    while(TRIGGER_EVENT_FLAG == 0 && TRIGGER_MODE_FLAG == 1){
      __NOP();
      unit_usb();
    }
  }

  while(1 == 1){

    if(TRIGGER_MODE_FLAG == 1){

      // Raises Error LED if tigger is too early
      if(TRIGGER_EVENT_FLAG == 1){
        gpio_pin_write(LED_1, 1);
      }
      else {
        gpio_pin_write(LED_1, 0);
      }

      while(TRIGGER_EVENT_FLAG == 0){ // Waiting for Trigger
          __NOP();
          if(trigger_timeout_value < __HAL_TIM_GET_COUNTER(&htim2)){ // Trigger time out
              DAC_write(0, 0, 1);
           }
      }

      TRIGGER_EVENT_FLAG = 0;
      DAC_write(vel[0] * (9.0/45.0), vel[1] * (9.0/45.0), 1);

      measurement_US_interleaved(vel);
    }
    else if (TRIGGER_MODE_FLAG == 2) { // Calibration mode, all measurements are triggered by via UART
      __NOP();
    }
    else{
      measurement_US_interleaved(vel);
    }


    if(ASCII_FLAG == 1){
      char txt_buffer[50];
      uint8_t tr_length = sprintf(txt_buffer,"%f, %f\n",vel[0],vel[1]);
      uart_transmit(&USART,txt_buffer,tr_length);
      
    }
    else if (ASCII_FLAG == 2) { // One time output
      ASCII_FLAG = 0;
      char txt_buffer[50];
      uint8_t tr_length = sprintf(txt_buffer,"%f, %f\n",vel[0],vel[1]);
      uart_transmit(&USART,txt_buffer,tr_length);   
    }


    // Look if there is data on the Uart -> shortest command posible 2 char
    if(uart_bytesReadable(&USART) > 1){
      unit_usb();
    }
  }


}

/*************************** End of file ****************************/
