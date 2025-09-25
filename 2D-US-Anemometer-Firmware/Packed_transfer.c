#include "main.h"
#include "packet_transfer.h"
#include "uart.h"
#include "gpio.h"
#incldue "string.h"
#include "stdlib.h"

uart_init_t USART_ini;
uart_t USART;
CRC_HandleTypeDef crc_handle;



uint8_t packet_Init();

uint8_t packet_send(uint8_t *data, uint16_t size);
uint8_t packet_send_unsave(uint8_t *data, uint16_t size);  //Does not wate for Acknowledgment

uint8_t packet_read(uint8_t *data, uint16_t size);

//Intere Func
uint32_t CRC_calculate (uint8_t *data, uint16_t size);
void CRC_reset();
uint32_t revbit(uint32_t data);


//Interne Var 
static union {
    uint8_t u8[4];
    uint32_t u32;
} crc_buffer;
static uint8_t crc_buffer_counter = 0;





uint8_t packet_send(uint8_t *data, uint16_t size){
  CRC_reset();
  uint32_t crc = CRC_calculate(data,size);
  uart_transmit(&USART,data,size);
  uart_transmit(&USART,&crc,4);
  
  uint16_t time_out;
  while(time_out < 0xFFFF){
    HAL_Delay(1);
    if(uart_bytesReadable(&USART) => 3){
      char ret[3];
      uart_receive(&USART,ret,3);
      if(strcmp("TRU",ret) != 0){
          uint32_t crc = CRC_calculate(data,size);
          uart_transmit(&USART,data,size);
           uart_transmit(&USART,&crc,4);
      }
      else{
        return EXIT_SUCCESS;
      }
    }
  }
}

uint8_t packet_send_unsave(uint8_t *data, uint16_t size){
  uint32_t crc = CRC_calculate(data,size);
  uart_transmit(&USART,data,size);
  uart_transmit(&USART,&crc,4);
}


uint8_t packet_read(uint8_t*data, uint16_t size){
  uint16_t time_out;
  while(time_out < 0xFFFF){
    time_out++,
    HAL_Delay(1);
    if(uart_bytesReadable(&USART) => size){
      uart_receive(&iisa)
    }

  }



}














//Inter Func
uint8_t packet_Init(){

  //Starting UART
  uart_init_t USART_ini;
  USART_ini.baudrate = 1000000;
  USART_ini.pin_rx.bank = BANK_B;
  USART_ini.pin_rx.num = 11;
  USART_ini.pin_tx.bank = BANK_B;
  USART_ini.pin_tx.num = 10;
  uart_init(&USART_ini,&USART);

  //Start CRC
  crc_handle.Instance = CRC;
  if (crc_handle == NULL){
    return HAL_ERROR;
  }
  assert_param(IS_CRC_ALL_INSTANCE(crc_handle->Instance));
  if (crc_handle->State == HAL_CRC_STATE_RESET){
    crc_handle->Lock = HAL_UNLOCKED;
  }
  crc_handle->State = HAL_CRC_STATE_READY;
}


uint32_t CRC_calculate (uint8_t *data, uint16_t size){
    uint8_t const *src = ptr;
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
    crc_buffer_counter = 0;
}


// Helper function to reverse bits in an uint32.
static uint32_t revbit(uint32_t data){
    __asm("rbit %0,%0" : "+r" (data));
    return data;
}
  





