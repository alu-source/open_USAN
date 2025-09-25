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
#include "SRAM_IS66.h"
#include "Shot.h"
#include "Shared.h"
#include "stdio.h"
#include "Unittest.h"
#include "gpio.h"
#include "Matched_Filter_r0.h"
#include "Filter_Fusion_r0.h"
#include "Fusion_eval_r0.h"
#include "Algo_helper.h"
#include "measurement.h"
#include "debug.h"
#include "Unittest.h"



/**
  Some of the functions are broken and will not be fixed !
    -> may be used to get an inside of how things were handelt for testing
*/




#define BULK_MESSURMENTS 512

#define BULK_TRANSMIT_SIZE 4

#define BULK_SIZE BULK_TRANSMIT_SIZE * 512 * 4 * 2  

#define CHECK_BIT(var,pos) (((var)>>(pos)) & 1)

uint16_t bulk_buffer[BULK_SIZE/2];
float key_score_buffer[2048];
float key_std_buffer[2048];


extern uart_t USART;
extern TIM_HandleTypeDef htim2;
extern Transceiver_handle shot_handle[4];
extern Shot_data_struct shot_data[4];
extern gpio_pin_init_t gpio_pin_C9;
extern EEPROM_OBJ EEPROM_data; 


void debug_matlab_key_test(void);
void debug_matlab_data_transfer(void);
void debug_record_and_send(int freq, int n);
void debug_perf_test_benchmark();

/**
  Function to Transfer data From MCU to Matlab 
*/
void debug_matlab_data_transfer(void) {
  // Waiting for go Signal
  while (uart_bytesReadable(&USART) <= 5) {
    HAL_Delay(1);
  }
  HAL_Delay(5);
  char txt_buffer[20] = {0};
  char cmd[2];
  int freq = 0;
  //uint8_t trans_buffer[4069];
  int n_mess = 0; //Amount of Bulk Transmitts 

  uart_receive(&USART, txt_buffer, uart_bytesReadable(&USART));
  printf("%i \n",uart_bytesReadable(&USART));

  sscanf(&txt_buffer[0], "%s %i %i ", cmd, &freq, &n_mess);

  if ( txt_buffer[1] == 'U' && freq > 4 && freq < 300) {
    debug_record_and_send(freq,n_mess);
  }
}


/*
  Function to do the Bulk of the work
*/
void debug_record_and_send(int freq, int n){

    // Calc time Betwenn shots
    uint32_t time = 1000000 / (freq*4);
    uint32_t shot_start [4];
    shot_start[0] = time;
    shot_start[1] = time*2;
    shot_start[2] = time*3;
    shot_start[3] = time*4;

    //Calc n_mess 
    uint16_t n_mess = BULK_TRANSMIT_SIZE * n;


    // Doing the shooting        pew pew .....
    //Signal End of Messurment
    for(uint16_t i = 0; i < n_mess; i++){
      //Start of Messurement
      __HAL_TIM_SET_COUNTER(&htim2, 0);

       if(i != 0){
        Shot_unlock_data(&shot_data[3]);
      }
      // Shot 1
     
     
    
    // gpio_pin_write(gpio_pin_C9.pin, 1);
      Shot_fire(&shot_handle[0],&shot_data[0]);
      //HAL_Delay(1);
      //gpio_pin_write(gpio_pin_C9.pin, 0);
      //Save the Prev. Messurment to SRAM
      if(i != 0){
        SRAM_write_page((uint8_t*)&shot_data[3].data + SHOT_ADC_OFFSET*2, 4 * (i-1) + 3);
        //packet_send((uint8_t*)&shot_data[3].data, 1024);
      }
      //Wait for next start Time
      while(__HAL_TIM_GET_COUNTER(&htim2) < shot_start[0]);

      // Shot 2
      Shot_unlock_data(&shot_data[0]);
      Shot_fire(&shot_handle[1],&shot_data[1]);

      //Save the Prev. Messurment to SRAM
      SRAM_write_page((uint8_t*)&shot_data[0].data + SHOT_ADC_OFFSET*2, 4 * i);
      //packet_send((uint8_t*)&shot_data[0].data, 1024);

      //Wait for next start Time
      while(__HAL_TIM_GET_COUNTER(&htim2) < shot_start [1]);

      // Shot 3
      Shot_unlock_data(&shot_data[1]);
      //gpio_pin_write(gpio_pin_C9.pin, 1);
      Shot_fire(&shot_handle[2],&shot_data[2]);
      //HAL_Delay(1);
      //gpio_pin_write(gpio_pin_C9.pin, 0);


      //Save the Prev. Messurment to SRAM
      SRAM_write_page((uint8_t*)&shot_data[1].data + SHOT_ADC_OFFSET*2, 4 * i + 1);
      //packet_send((uint8_t*)&shot_data[1].data, 1024);

      //Wait for next start Time
      while(__HAL_TIM_GET_COUNTER(&htim2) < shot_start [2]);

      // Shot 4
      Shot_unlock_data(&shot_data[2]);
      Shot_fire(&shot_handle[3],&shot_data[3]);

      //Save the Prev. Messurment to SRAM
      SRAM_write_page((uint8_t*)&shot_data[2].data + SHOT_ADC_OFFSET*2, 4 * i + 2);
      //packet_send((uint8_t*)&shot_data[2].data, 1024);
      //Wait for next start Time
      while(__HAL_TIM_GET_COUNTER(&htim2) < shot_start [3]);

      if(i == n_mess - 1){
        HAL_Delay(10);
        Shot_unlock_data(&shot_data[3]);
        SRAM_write_page((uint8_t*)&shot_data[3].data + SHOT_ADC_OFFSET*2, 4 * i + 3);
        //packet_send((uint8_t*)&shot_data[3].data, 1024);
      }

    }

    //Signal End of Messurment
    //gpio_pin_write(gpio_pin_C9.pin, 0);
    
    // Send data Back to Matlab
    for(uint16_t i = 0; i < (n_mess/BULK_TRANSMIT_SIZE); i++){

      //Fill buffer
      for(uint16_t j = 0; j < BULK_TRANSMIT_SIZE; j++){
        for(uint8_t jj = 0; jj < 4; jj++){
          uint32_t page_addr = jj + j*4 + i * (BULK_TRANSMIT_SIZE * 4);
          uint32_t byte_offset = (jj + j*4) * 1024;
          while(SRAM_Status() != 0); //Wait for Transfer to finish 
          SRAM_read_page((uint8_t*)bulk_buffer + byte_offset, page_addr);
        }
      }

      // Write UART
      packet_send((uint8_t*)bulk_buffer,BULK_SIZE);

      HAL_Delay(10);
    }
    
}


/*
  Used for testing diffrent Keys and there Performance 
*/
void debug_matlab_key_test(void){
  int freq,key,n_mess;

  //Parse Arg
  while (uart_bytesReadable(&USART) <= 5) {
    HAL_Delay(1);
  }
  HAL_Delay(5);
  char txt_buffer[20] = {0};
  char cmd[2];

  uart_receive(&USART, txt_buffer, uart_bytesReadable(&USART));
  sscanf(&txt_buffer[0], "%s %i %i %i ", cmd, &freq, &n_mess, &key);

  if(txt_buffer[1]=='K' && freq < 300 && freq > 10 && n_mess < 250){
    uint16_t key_prototype = (uint16_t)key;
    uint8_t key_buffer[SHOT_KEY_LENGTH];

    //Convert key 
    for(uint8_t i = 0; i < SHOT_KEY_LENGTH; i++){
        key_buffer[i] = CHECK_BIT(key_prototype,i); 
    }

    //Generate the Pattern
    Shot_generate_gpio_from_Key(key_buffer, shot_handle);

    //Starting the Sending 
    debug_record_and_send( freq,n_mess);
  }
}


#define AMP_MAX 3900
#define AMP_MIN 2800
#define AMP_ABS 2000

//Var used for statistical eval
uint16_t key_amp_max_error = 0;
uint16_t key_amp_min_error = 0;
uint16_t key_std_min_error = 0;
uint16_t key_std_max_error = 0;
uint16_t key_vel_error = 0;

void debug_standalone_key_test(void) {
  uint16_t max_key_value = 2048;    // second halfe can be skiped, bc the key are only phase swapped
  uint16_t progress_increment = max_key_value / 100;    // Used to indicate progress of the measurment
  uint16_t progress_last_increment = 0;
  uint8_t key_buffer[SHOT_KEY_LENGTH];

  char txt_buffer[75];
  uint8_t len = sprintf(txt_buffer, "Key testing started, this will take a while...\n");
  uart_transmit(&USART, txt_buffer, len);
  len = sprintf(txt_buffer, "Overview: Progress, over amp, under amp, over std, under std, velocity\n");
  uart_transmit(&USART, txt_buffer, len);

  for (uint16_t key_prototype = 0; key_prototype < max_key_value; key_prototype++) {
    // Convert key to pattern
    for (uint8_t i = 0; i < SHOT_KEY_LENGTH; i++) {
      key_buffer[i] = CHECK_BIT(key_prototype, i);
    }

    // Generate pattern
    Shot_generate_gpio_from_Key(key_buffer, shot_handle);

    // Take a zero meas
    measurement_null(EEPROM_data.E_Filter);

    // Test if Amplitude is in range
    uint8_t amp_valid = 0;
    float amp_res_max, amp_res_min;
    uint32_t index_res;

    for (uint8_t i = 0; i < 4; i++) {
      arm_max_f32(EEPROM_data.E_Filter[i].match_raw, 512, &amp_res_max, &index_res);     
      arm_min_f32(EEPROM_data.E_Filter[i].match_raw, 512, &amp_res_min, &index_res);

      if (amp_res_max < AMP_MIN) {

        if(amp_valid == 0){
          key_amp_min_error++;
        }
        amp_valid++;
      } else if (amp_res_max > AMP_MAX) {

        if(amp_valid == 0){
          key_amp_max_error++;
        }
        amp_valid++;
      }

      if(amp_res_min < 4096 - AMP_MAX){
        if(amp_valid == 0){
                  key_amp_max_error++;
        }
        amp_valid++;
      }
      else if (amp_res_min > 4096 - AMP_MIN){
        if(amp_valid == 0){
          key_amp_min_error++;
        }
        amp_valid++;
      }
      if(amp_res_max - amp_res_min < AMP_ABS){
        if(amp_valid == 0){
          key_amp_max_error++;
        }
        amp_valid++;
      }
    }
    





    // Skip key testing if amp is out of range (amp_valid > 0)
    if (amp_valid == 0) {
      uint8_t key_valid = 0;

      // Run test
      float vel[2];
      float score[2];
      float std[2];
      measurement_US_interleaved_PPR(200, vel, score, std);

      // Check if std is in range !
      if (std[0] == 0.0 || std[1] == 0.0) {    // std unresanable small
        key_score_buffer[key_prototype] = 0;
        key_std_min_error++;
        key_valid++;
      } else if (std[0] > 1.0 || std[1] > 1.0) {    // std unresanable big
        key_score_buffer[key_prototype] = 0;
        key_std_max_error++;
        key_valid++;
      }

      // Check if vel is in range !
      if (fabsf(vel[0]) > 1 || fabsf(vel[1]) > 1) {
        key_score_buffer[key_prototype] = 0;
        key_vel_error++;
        key_valid++;
      }

      if (key_valid == 0) {
        if (score[0] > score[1]) {
          key_score_buffer[key_prototype] = score[1];
        } else {
          key_score_buffer[key_prototype] = score[0];
        }

        if (std[0] > std[1]) {
          key_std_buffer[key_prototype] = std[0];
        } else {
          key_std_buffer[key_prototype] = std[1];
        }

      } else {
        key_score_buffer[key_prototype] = 0;
        key_std_buffer[key_prototype]   = 0;
      }

    } else {
      key_score_buffer[key_prototype] = 0;
    }


    // Print out progress after each perecent
    if (progress_increment + progress_last_increment <= key_prototype) {
      uint8_t progress = progress_last_increment / progress_increment;
      progress_last_increment += progress_increment;
      char txt_buffer[75];
      uint8_t len = sprintf(txt_buffer,
                            "Currently %4.d %% done, errors: %4.d, %4.d, %4.d, %4.d, %4.d\n",
                            progress,
                            key_amp_max_error,
                            key_amp_min_error,
                            key_std_max_error,
                            key_std_min_error,
                            key_vel_error);
      uart_transmit(&USART, txt_buffer, len);
    }
  }

  // Add final print outs
  len = sprintf(txt_buffer, "Currently 100 %% done, 2048 keys tested\n");
  uart_transmit(&USART, txt_buffer, len);

  // Best key is the one with the highest score
  // finde the five key with the largest score
  uint16_t max_score_idx[5];
  float max_score[5];
  float max_score_std[5];

  for (uint8_t i = 0; i < 5; i++) {
    arm_max_f32(key_score_buffer, 2048, &max_score[i], (uint32_t*)&max_score_idx[i]);
    key_score_buffer[max_score_idx[i]] = 0;
    max_score_std[i]                   = key_std_buffer[max_score_idx[i]];
    len = sprintf(txt_buffer, "%d. Key %d, Score: %f, std: %f\n", i, max_score_idx[i], max_score[i], max_score_std[i]);
    uart_transmit(&USART, txt_buffer, len);
  }

  // Print error statistics
  len = sprintf(txt_buffer, "Amplitude Errors: over %d, under %d\n", key_amp_max_error, key_amp_min_error);
  uart_transmit(&USART, txt_buffer, len);
  len = sprintf(txt_buffer, "Standarddeviation: over %d, under %d\n", key_std_max_error, key_std_min_error);
  uart_transmit(&USART, txt_buffer, len);
  len = sprintf(txt_buffer, "Velocity: %d\n", key_vel_error);
  uart_transmit(&USART, txt_buffer, len);

  len = sprintf(txt_buffer, "Don't forget to add new key to the Sensor_config\n");
  uart_transmit(&USART, txt_buffer, len);
  }



extern Shot_data_struct shot_data[4];

void debug_send_shot_data(uint16_t KEY){
    uint8_t key_buffer[SHOT_KEY_LENGTH];

   // Convert key to pattern
    for (uint8_t i = 0; i < SHOT_KEY_LENGTH; i++) {
      key_buffer[i] = CHECK_BIT(KEY, i);
    }

    // Generate pattern
    Shot_generate_gpio_from_Key(key_buffer, shot_handle);

    // Take a zero meas
    measurement_null(EEPROM_data.E_Filter);

    float vel[2], score[2] ,std[2];
    measurement_US_interleaved_PPR(200, vel, score, std);
    measurement_US_interleaved_PPR(200, vel, score, std);


    // Copy to buffer for ease of use
    for(uint8_t i = 0; i < 4; i++){
      for(uint16_t j = 0; j < 512; j++) {
        bulk_buffer[i*512 + j] = shot_data[i].data[j];
      }
    }


    CRC_reset();
    uint32_t packet_crc = CRC_calculate((uint8_t*)&bulk_buffer, (4 * 512 * 2));
    uint16_t error = uart_transmit(&USART, (uint8_t*)&bulk_buffer,  (4 * 512 * 2));
    uart_transmit(&USART, (uint8_t*)&packet_crc, 4);
    //HAL_UART_Transmit(USART.uart_instance, const uint8_t *pData, uint16_t Size, uint32_t Timeout)


}


#define n_timing 100

uint32_t worst_buffer[8];
uint32_t avg_buffer[8];
uint16_t sub_meas_list[] = {2, 4, 8, 16, 32 ,64, 128, 256};
extern uint16_t sub_meas;

void debug_timing(){
  uint32_t avg = 0, worst = 0;
  float vel_buf[2];
  
  for(uint8_t j = 0; j < 8; j++){
    sub_meas = sub_meas_list[j];

    for(uint8_t i = 0; i < n_timing; i++){

      measurement_US_interleaved(vel_buf);
      uint32_t time_cnt = __HAL_TIM_GET_COUNTER(&htim2);
      if(time_cnt > worst){
        worst = time_cnt;
      }
      avg += time_cnt;
    }

    worst_buffer[j] = worst;
    avg_buffer[j] = avg/n_timing;
    char txt_buffer[50];
    uint8_t len = sprintf(txt_buffer, "Time for %d sub_meas avg: %d, worst %d\n", sub_meas_list[j], avg/n_timing, worst);
    uart_transmit(&USART, txt_buffer, len);
    
    worst = 0;
    avg = 0;
  }
}