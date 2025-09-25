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
#include "packet_transfer.h"
#include "uart.h"
#include "stdlib.h"
#include "stdio.h"


#define TIME_OUT 0xFFFF

//Uart Handles
uart_init_t USART_ini;
uart_t USART;
CRC_HandleTypeDef crc_handle;


uint8_t packet_Init();

uint8_t packet_send(uint8_t *data, uint16_t size);
uint8_t packet_send_unsave(uint8_t *data, uint16_t size);  //Does not wate for Acknowledgment

uint8_t packet_read(uint8_t *data, uint16_t size);

//Intere Func
uint32_t CRC_calculate (uint8_t *data, uint32_t size);
void CRC_reset();
uint32_t revbit(uint32_t data);


//Interne Var 
static union {
    uint8_t u8[4];
    uint32_t u32;
} crc_buffer;
static uint32_t crc_buffer_counter = 0;





uint8_t packet_send(uint8_t *data, uint16_t size){

  //Do the CRC32 Calc
  CRC_reset();
  uint32_t crc = CRC_calculate(data,size);

  //Start of Transmission
  uart_transmit(&USART,data,size);
  uart_transmit(&USART,&crc,4);
  
  //Wait for ACK of the receiver
  uint16_t time_out=0;
  while(time_out < 2000){
     HAL_Delay(1);
    time_out ++;
    if(uart_bytesReadable(&USART) >= 3){
      char ret[3];
      uart_receive(&USART,ret,3);
      if(ret[0] != 'T' ||ret[1] != 'R' ||ret[2] != 'U'){ //If not read correctly resend ! 
          printf("%s\n",ret);
          uint32_t crc = CRC_calculate(data,size);
          uart_transmit(&USART,data,size);
          uart_transmit(&USART,&crc,4);
      }
      else{
        return EXIT_SUCCESS;
      }
    }
  }
  return EXIT_FAILURE;
  
}

uint8_t packet_send_unsave(uint8_t *data, uint16_t size){
  uint32_t crc = CRC_calculate(data,size);
  uart_transmit(&USART,data,size);
  uart_transmit(&USART,&crc,4);
}


uint8_t packet_read(uint8_t*data, uint16_t size){
  uint16_t time_out;
  uint32_t crc;
  while(time_out < 0xFFFF){
    time_out++,
    HAL_Delay(1);
    if(uart_bytesReadable(&USART) >= size +4){
      uart_receive(&USART, data, size);
      uart_receive(&USART, &crc, 4);
      if(crc != CRC_calculate(data,size)){
        uart_transmit(&USART,"FAL\n",4);
      }
      else{
        uart_transmit(&USART,"TRU\n",4);
        return EXIT_SUCCESS;
      }
    }


  }
  return EXIT_FAILURE;
}




//Inter Func
uint8_t packet_Init(void){

  //Starting UART
  uart_init_t USART_ini;
  USART_ini.baudrate = 1000000;
  USART_ini.pin_rx.bank = BANK_B;
  USART_ini.pin_rx.num = 11;
  USART_ini.pin_rx.num = 11;
  USART_ini.pin_tx.bank = BANK_B;
  USART_ini.pin_tx.num = 10;
  USART_ini.uart_instance = UART_3; 
  uart_init(&USART_ini,&USART);

  //Start CRC
    __HAL_RCC_CRC_CLK_ENABLE();
    __NOP(); 
    crc_handle.Instance = CRC;

    if (HAL_CRC_Init(&crc_handle) != HAL_OK) {
        while (1);
    }

}


uint32_t CRC_calculate (uint8_t *data, uint32_t size){
    uint8_t const *src = data;
    while (size > 0) {
        if (crc_buffer_counter == 0) {
            while (size >= 4) {
                uint32_t w = *(uint32_t const *)src;

                w = revbit(w);

                crc_handle.Instance->DR = w;
                size -= 4;
                src += 4;
            }
        }
        
        while (size > 0) {
            crc_buffer.u8[crc_buffer_counter] = *src;
            
            src++;
            crc_buffer_counter++;
            size--;
            
            if (crc_buffer_counter == 4) {
                uint32_t w = crc_buffer.u32;
                w = revbit(w);
                crc_handle.Instance->DR = w;
                crc_buffer_counter = 0;
                
                break;
            }
        }
    }
    uint32_t crc = crc_handle.Instance->DR;

    crc = revbit(crc);

    if (crc_buffer_counter > 0) {
        uint32_t buffer = crc_buffer.u32;
        switch (crc_buffer_counter) {
            case 1:
                buffer &= 0xff;
                break;
            case 2:
                buffer &= 0xffff;
                break;
            case 3:
            default:
                buffer &= 0xffffff;
                break;
        }
        crc ^= buffer;

        for (uint8_t j = 0; j < 8 * crc_buffer_counter; j++) {
            if (crc & 1) {
                crc = (crc >> 1) ^ 0xEDB88320;
            } else {
                crc >>= 1;
            }
        }
    }
    crc_buffer_counter = 0;

    return ~crc;
}

void CRC_reset(void){
    __HAL_CRC_DR_RESET(&crc_handle);
    __NOP();
    crc_buffer_counter = 0;
}


// Helper function to reverse bits in an uint32.
uint32_t revbit(uint32_t data){
    __asm("rbit %0,%0" : "+r" (data));
    return data;
}
  