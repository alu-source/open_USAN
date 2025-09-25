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
#include "DAC_AD5732.h"
#include "Shot.h"
#include "Shared.h"
#include "measurement.h"
#include "Filter_Fusion_r0.h"
#include "Matched_Filter_r0.h"
#include "Fusion_eval_r0.h"
#include "correction.h"
#include "uart.h"
#include "Unittest.h"
#include <stdio.h>
#include "Sensor_config.h"
#include "debug.h"

#define START_END_TIME 700 //in us
#define TIMING_ERROR_MARGINE 0 //in us
#define LOOP_TIME 0 // in us
#define MIN_MEAS_TIME 1e6 / (4 * SUB_MEASUREMENT_FREQ)  //in us
#define TRIGGER_TIMOUT_MARGINE 1000 // in us



void measurement_US(float * vel);

void measurement_init(EEPROM_Meas_OBJ * meas_data);

void measurement_US_interleafe(float * vel);

void measurement_null (EEPROM_Filter_OBJ * filter_data);




//Externe Var
extern Transceiver_handle shot_handle[4]; //Unit_test
extern Shot_data_struct shot_data[4]; //Unit test
extern TIM_HandleTypeDef htim2; //shared
extern uart_t USART;
extern uint8_t TRIGGER_EVENT_FLAG;
extern uint8_t TRIGGER_MODE_FLAG;
extern EEPROM_OBJ EEPROM_data; 


//Interne Var
uint8_t startup_FLAG = 0;
uint32_t time_stepp = 0;
uint32_t next_time = 0;
uint16_t dummy[512];
float a_ref = 345;
uint32_t trigger_timeout_value = 0xFFFFFFFF;

Filter_return_data Filter_ret_a[5] __attribute__((section(".CCM_VAR")));
Filter_return_data Filter_ret_b[5] __attribute__((section(".CCM_VAR")));
Filter_return_data Filter_ret_c[5] __attribute__((section(".CCM_VAR")));
Filter_return_data Filter_ret_d[5] __attribute__((section(".CCM_VAR")));
Fusion_return_candidate Fusion_ret_x[3] __attribute__((section(".CCM_VAR")));
Fusion_return_candidate Fusion_ret_y[3] __attribute__((section(".CCM_VAR")));

Filter_instance Filter_inst[4] __attribute__((section(".CCM_VAR")));

uint16_t sub_meas = 0;
uint32_t clock_increment = 0;
float total_time = 0;
float vel_buffer[2][600];

/*
  Functions
*/
void measurement_US_calibration(float * vel, uint16_t sub_meas, float max_error, uint8_t timing);
void measurement_init(EEPROM_Meas_OBJ * meas_data);
void measurement_US(float* vel);
void measurement_null (EEPROM_Filter_OBJ * filter_data);
void measurement_US_interleaved(float* vel);
void measurement_US_interleaved_PPR(uint8_t n_meas, float * vel, float * score, float * std);

void measurement_init(EEPROM_Meas_OBJ * meas_data){
  a_ref = meas_data->a_ref;

  if(meas_data->freq != 0){
      TRIGGER_MODE_FLAG = 1;
    if (meas_data->duty > 1) {
      // Total time between trigger events - the time bevore start and ending
        total_time            = (1.0 / meas_data->freq) * 1e6 - CLOCK_MARGINE;                             // in us
        trigger_timeout_value = total_time + TRIGGER_TIMOUT_MARGINE;                                       // Used to detect missing trigger
        sub_meas              = (uint16_t)floorf((total_time * meas_data->duty/100)/(4*MIN_MEAS_TIME));    // number off sub measurementa
        time_stepp            = MIN_MEAS_TIME;
    } 
    else {
      time_stepp = MIN_MEAS_TIME;
      sub_meas   = 2;
      total_time = START_END_TIME  +MIN_MEAS_TIME*4 + 1;
    }
  }else{
    // Total time between trigger events - the time bevore start and ending
    TRIGGER_MODE_FLAG = 0;                                                   // No trigger
    if(meas_data->duty <= 100){
          total_time            = (1.0 / meas_data->duty) * 1e6  - CLOCK_MARGINE;       
          sub_meas              = (uint16_t)floorf((total_time)/(4*MIN_MEAS_TIME));
          time_stepp            = MIN_MEAS_TIME;
        }
    else{
      sub_meas          = 5;                                                 // Default num of sub meas
    }

    time_stepp        = MIN_MEAS_TIME;
    total_time        = (sub_meas*time_stepp*4);

  }

  //Print
  char txt_buffer[100];
  uint8_t length = sprintf(txt_buffer,"Meas setup:\n");
  uart_transmit(&USART,txt_buffer,length);


  if(meas_data->freq != 0){
    // Requested
    length = sprintf(txt_buffer,"Requested:\n");
    uart_transmit(&USART,txt_buffer,length);
    length = sprintf(txt_buffer,"Trigger Freq -> %d\n",meas_data->freq);
    uart_transmit(&USART,txt_buffer,length);
    length = sprintf(txt_buffer,"Duty -> %d\n\n",meas_data->duty);
    uart_transmit(&USART,txt_buffer,length);

    //Actual
    length = sprintf(txt_buffer,"Actual:\n");
    uart_transmit(&USART,txt_buffer,length);
    length = sprintf(txt_buffer,"Sub measurements -> %d\n",sub_meas);
    uart_transmit(&USART,txt_buffer,length);
    length = sprintf(txt_buffer,"Resulting dutycycle -> %f %%\n",((sub_meas*time_stepp*4)/total_time) * 100);
    uart_transmit(&USART,txt_buffer,length);
  }
  else{
    // Requested
    length = sprintf(txt_buffer,"Requested:\n");
    uart_transmit(&USART,txt_buffer,length);
    length = sprintf(txt_buffer,"Measurement Freq -> %d\n",meas_data->duty);
    uart_transmit(&USART,txt_buffer,length);
    length = sprintf(txt_buffer,"Number of sub measurements -> %d\n",sub_meas);
    uart_transmit(&USART,txt_buffer,length);

  }

  //Init of the DSP components ! 
  Filter_Init(Filter_inst, EEPROM_data.E_Filter);
  Filter_Fusion_init(&EEPROM_data.E_Fusion);
  Fusion_eval_init(&EEPROM_data.E_Eval);
}
void measurement_US(float* vel) {
  time_stepp = MIN_MEAS_TIME;    // 690;

  // Start the measurement
  __HAL_TIM_SET_COUNTER(&htim2, 0);
  next_time = time_stepp;

  // Shot 1
  Shot_fire(&shot_handle[0], &shot_data[0]);
  while (__HAL_TIM_GET_COUNTER(&htim2) < next_time)
    ;

  // Shot 2
  Shot_fire(&shot_handle[1], &shot_data[1]);
  next_time += time_stepp;
  Shot_unlock_data(&shot_data[0]);
  Filter_run_r0(Filter_ret_a, &Filter_inst[0], shot_data[0].data);
  while (__HAL_TIM_GET_COUNTER(&htim2) < next_time)
    ;

  // Shot 3
  Shot_fire(&shot_handle[2], &shot_data[2]);
  next_time += time_stepp;
  Shot_unlock_data(&shot_data[1]);
  Filter_run_r0(Filter_ret_c, &Filter_inst[1], shot_data[1].data);
  while (__HAL_TIM_GET_COUNTER(&htim2) < next_time)
    ;

  // Shot 4
  Shot_fire(&shot_handle[3], &shot_data[3]);
  next_time += time_stepp;
  Shot_unlock_data(&shot_data[2]);
  Filter_run_r0(Filter_ret_b, &Filter_inst[2], shot_data[2].data);

  // Filter Fusion for x dir
  Filter_Fusion(Filter_ret_a, Filter_ret_b, Fusion_ret_x, a_ref);

  while (__HAL_TIM_GET_COUNTER(&htim2) < next_time)
    ;

  Shot_unlock_data(&shot_data[3]);
  Filter_run_r0(Filter_ret_d, &Filter_inst[3], shot_data[3].data);

  // Filter Fusion for y dir
  Filter_Fusion(Filter_ret_c, Filter_ret_d, Fusion_ret_y, a_ref);

  // Fusion eval
  Fusion_eval_r0(Fusion_ret_x, Fusion_ret_y, &a_ref, vel);

  // Correction
  correction_apply(vel, &EEPROM_data.E_Cali);
  DAC_write(vel[0] * (9.0 / 45.0), vel[1] * (9.0 / 45.0), 1);
}

float match_buffer[512];

void measurement_null (EEPROM_Filter_OBJ * filter_data){
  
  //ZERO_MEAS_SAMPLES Samples
  for(uint8_t i = 0; i < 4; i++){
    for(uint16_t j = 0; j < 512; j++){
      match_buffer[j] = 0;
    }
  
    for(uint8_t j = 0; j < ZERO_MEAS_SAMPLES; j++){
      HAL_Delay(2);
      Shot_fire(&shot_handle[i],&shot_data[i]);
      HAL_Delay(3);
      Shot_unlock_data(&shot_data[i]);
      for (uint16_t g = 0; g < ALGO_BUFFER_SIZE; g++){
        match_buffer[g] += (float)shot_data[i].data[g+ADC_OFFSET];
      }
    }
    for(uint16_t j = 0; j < ALGO_BUFFER_SIZE; j++){
      filter_data[i].match_raw[j] = (float)match_buffer[j]/(float)ZERO_MEAS_SAMPLES;
    }

    //Check if average is ok !
    double sum[4] = {0,0,0,0};
    for(uint8_t i = 0; i < 4; i++){
      for(uint16_t j = 0; j < ALGO_BUFFER_SIZE; j++){
        sum[i] += filter_data[i].match_raw[j];
      }
      if(sum[i]/(ALGO_BUFFER_SIZE) > 3600 || sum[i]/(ALGO_BUFFER_SIZE) < 500){
          char txt_buffer[75];
          uint8_t len = sprintf(txt_buffer, "Error creating zero measurement !\n\r");
          uart_transmit(&USART, txt_buffer, len);
          filter_data[i].match_raw[0] = 0xFFFF;
      }
    }


  }

  //Testing the Match 
  Filter_Init(Filter_inst, filter_data);
  Filter_Fusion_init(&EEPROM_data.E_Fusion);


 #ifdef  KEY_TEST
 #else
  float h1[2];
  double sum[2] = {0,0};

  EEPROM_data.E_Meas.v_of[0] = 0;
  EEPROM_data.E_Meas.v_of[1] = 0;
  for(uint16_t i = 0; i < ZERO_MEAS_SAMPLES*10; i++){
    measurement_US_calibration(h1, 5, MEASUREMENT_INTER_ERROR_CALIBRATION, ADAPTIV_TIMING);
  }

  for(uint8_t i = 0; i < ZERO_MEAS_SAMPLES; i++){
    measurement_US_calibration(h1, 5, MEASUREMENT_INTER_ERROR_CALIBRATION, ADAPTIV_TIMING);
    vel_buffer[0][i + 5] = h1[0];
    vel_buffer[1][i + 5] = h1[1];
  }

  for(uint8_t i = 0; i < ZERO_MEAS_SAMPLES; i++){
    sum[0] += (double)vel_buffer[0][i + 5];
    sum[1] += (double)vel_buffer[1][i + 5]; 
  }

  EEPROM_data.E_Meas.v_of[0] = sum[0]/(float)ZERO_MEAS_SAMPLES;
  EEPROM_data.E_Meas.v_of[1] = sum[1]/(float)ZERO_MEAS_SAMPLES;
#endif
}



void measurement_US_interleaved(float* vel) {
  time_stepp = MIN_MEAS_TIME - (uint32_t)((700.0 + sub_meas/8.0)/(4.0 * sub_meas));
  
  HAL_NVIC_DisableIRQ(SysTick_IRQn);

  // Start the measurement
  __HAL_TIM_SET_COUNTER(&htim2, 0);
  next_time = time_stepp;

  // Shot 1
  Shot_fire(&shot_handle[0], &shot_data[0]);
  while (__HAL_TIM_GET_COUNTER(&htim2) < next_time)
    ;
  for (uint16_t i = 0; i < sub_meas - 1; i++) {
    // Shot 2
    Shot_unlock_data(&shot_data[0]);
    Shot_fire(&shot_handle[1], &shot_data[1]);
    next_time += time_stepp;
    Filter_run_r0(Filter_ret_a, &Filter_inst[0], shot_data[0].data);
    if (i > 0) {
      Fusion_eval_r0(Fusion_ret_x, Fusion_ret_y, &a_ref, vel);

      vel_buffer[0][i - 1] = vel[0];
      vel_buffer[1][i - 1] = vel[1];
    }

    while (__HAL_TIM_GET_COUNTER(&htim2) < next_time)
      ;

    // Shot 3
    Shot_unlock_data(&shot_data[1]);
    Shot_fire(&shot_handle[2], &shot_data[2]);
    next_time += time_stepp;
    Filter_run_r0(Filter_ret_c, &Filter_inst[1], shot_data[1].data);

    while (__HAL_TIM_GET_COUNTER(&htim2) < next_time)
      ;

    // Shot 4
    Shot_unlock_data(&shot_data[2]);
    Shot_fire(&shot_handle[3], &shot_data[3]);
    next_time += time_stepp;
    Filter_run_r0(Filter_ret_b, &Filter_inst[2], shot_data[2].data);

    // Filter Fusion for x dir
    Filter_Fusion(Filter_ret_a, Filter_ret_b, Fusion_ret_x, a_ref);

    while (__HAL_TIM_GET_COUNTER(&htim2) < next_time)
      ;

    // Shot 1
    Shot_unlock_data(&shot_data[3]);
    Shot_fire(&shot_handle[0], &shot_data[0]);
    next_time += time_stepp;
    Filter_run_r0(Filter_ret_d, &Filter_inst[3], shot_data[3].data);

    // Filter Fusion for y dir
    Filter_Fusion(Filter_ret_c, Filter_ret_d, Fusion_ret_y, a_ref);

    while (__HAL_TIM_GET_COUNTER(&htim2) < next_time)
      ;
  }

  // Shot 2
  Shot_unlock_data(&shot_data[0]);
  Shot_fire(&shot_handle[1], &shot_data[1]);
  next_time += time_stepp;
  Filter_run_r0(Filter_ret_a, &Filter_inst[0], shot_data[0].data);

  Fusion_eval_r0(Fusion_ret_x, Fusion_ret_y, &a_ref, vel);

  vel_buffer[0][sub_meas - 2] = vel[0];
  vel_buffer[1][sub_meas - 2] = vel[1];

  while (__HAL_TIM_GET_COUNTER(&htim2) < next_time)
    ;

  // Shot 3
  Shot_unlock_data(&shot_data[1]);
  Shot_fire(&shot_handle[2], &shot_data[2]);
  next_time += time_stepp;
  Filter_run_r0(Filter_ret_c, &Filter_inst[1], shot_data[1].data);
  while (__HAL_TIM_GET_COUNTER(&htim2) < next_time)
    ;

  // Shot 4
  Shot_unlock_data(&shot_data[2]);
  Shot_fire(&shot_handle[3], &shot_data[3]);
  next_time += time_stepp;
  Filter_run_r0(Filter_ret_b, &Filter_inst[2], shot_data[2].data);

  // Filter Fusion for x dir
  Filter_Fusion(Filter_ret_a, Filter_ret_b, Fusion_ret_x, a_ref);

  while (__HAL_TIM_GET_COUNTER(&htim2) < next_time)
    ;

  Shot_unlock_data(&shot_data[3]);
  Filter_run_r0(Filter_ret_d, &Filter_inst[3], shot_data[3].data);


  // Filter Fusion for y dir
  Filter_Fusion(Filter_ret_c, Filter_ret_d, Fusion_ret_y, a_ref);

  // Fusion eval
  Fusion_eval_r0(Fusion_ret_x, Fusion_ret_y, &a_ref, vel);

  vel_buffer[0][sub_meas - 1] = vel[0];
  vel_buffer[1][sub_meas - 1] = vel[1];

  // Mean filter with error detection
  // Values >= 9999 are error values
  // Custom code can go here!
  float sum0 = 0, sum1 = 0;
  uint16_t diff = sub_meas;

  for (uint16_t i = 0; i < sub_meas; i++) {
    if (vel_buffer[0][i] < 9999 && vel_buffer[1][i] < 9999) {
      sum0 += vel_buffer[0][i];
      sum1 += vel_buffer[1][i];
    } else {
      diff--;
    }
  }

  vel[0] = (sum0 / diff);
  vel[1] = (sum1 / diff);


  // Diff filter, sort out val with to big diff to mean -> probably error val
  diff = sub_meas;
  sum0 = 0;
  sum1 = 0;

  for (uint16_t i = 0; i < sub_meas; i++) {
    if (fabsf(vel_buffer[0][i] - vel[0]) < MEASUREMENT_INTER_ERROR || fabsf(vel_buffer[1][i] - vel[1]) < MEASUREMENT_INTER_ERROR) {
      sum0 += vel_buffer[0][i];
      sum1 += vel_buffer[1][i];
    } else {
      diff--;
    }
  }

  vel[0] = (sum0 / diff) - EEPROM_data.E_Meas.v_of[0];
  vel[1] = (sum1 / diff) - EEPROM_data.E_Meas.v_of[1];

  // Correction
  correction_apply(vel, &EEPROM_data.E_Cali);

  // Flipping and inverting axis as needed
  #if SWAP_AXIS == true
    float h1 = vel[0];
    vel[0] = vel[1];
    vel[1] = h1;
  #endif

  #if INVERT_X == true
    vel[0] = -vel[0];
  #endif

  #if INVERT_Y == true
    vel[1] = -vel[1];
  #endif

  HAL_NVIC_EnableIRQ(SysTick_IRQn);
}

void measurement_US_calibration(float * vel, uint16_t n_sub_meas, float max_error, uint8_t timing){

// Switches timing modes
if(timing == ADAPTIV_TIMING){
  time_stepp = MIN_MEAS_TIME - (uint32_t)((700.0 + n_sub_meas/8.0)/(4.0 * n_sub_meas));
}else if (timing == LAGACY_ADAPTIV_TIMING) {
 time_stepp = MIN_MEAS_TIME - (uint32_t)((700.0 + n_sub_meas*4.0)/(4.0 * n_sub_meas));
}
else{
  time_stepp = MIN_MEAS_TIME;
}

//Start the measurement
__HAL_TIM_SET_COUNTER(&htim2, 0);
next_time = time_stepp;

//Shot 1
Shot_fire(&shot_handle[0],&shot_data[0]);
while(__HAL_TIM_GET_COUNTER(&htim2) < next_time);

for (uint16_t i = 0; i < n_sub_meas-1; i++) {
  // Shot 2
  Shot_unlock_data(&shot_data[0]);
  Shot_fire(&shot_handle[1], &shot_data[1]);
  next_time += time_stepp;
  Filter_run_r0(Filter_ret_a, &Filter_inst[0], shot_data[0].data);
  if(i > 0){
    //Complete prev measurment 
    Fusion_eval_r0(Fusion_ret_x, Fusion_ret_y, &a_ref, vel);

    //Save result in to buffer
    vel_buffer[0][i-1] = vel[0];
    vel_buffer[1][i-1] = vel[1];
    
  }


  while (__HAL_TIM_GET_COUNTER(&htim2) < next_time)
    ;

  // Shot 3
  Shot_unlock_data(&shot_data[1]);
  Shot_fire(&shot_handle[2], &shot_data[2]);
  next_time += time_stepp;
  Filter_run_r0(Filter_ret_c, &Filter_inst[1], shot_data[1].data);

  while (__HAL_TIM_GET_COUNTER(&htim2) < next_time)
    ;

  // Shot 4
  Shot_unlock_data(&shot_data[2]);
  Shot_fire(&shot_handle[3], &shot_data[3]);
  next_time += time_stepp;
  Filter_run_r0(Filter_ret_b, &Filter_inst[2], shot_data[2].data);

  // Filter Fusion for x dir
  Filter_Fusion(Filter_ret_a, Filter_ret_b, Fusion_ret_x, a_ref);

  while (__HAL_TIM_GET_COUNTER(&htim2) < next_time)
    ;

  // Shot 1
  Shot_unlock_data(&shot_data[3]);
  Shot_fire(&shot_handle[0], &shot_data[0]);
  next_time += time_stepp;
  Filter_run_r0(Filter_ret_d, &Filter_inst[3], shot_data[3].data);

  // Filter Fusion for y dir
  Filter_Fusion(Filter_ret_c, Filter_ret_d, Fusion_ret_y, a_ref);

  while (__HAL_TIM_GET_COUNTER(&htim2) < next_time)
  ;
}
// Shot 2
Shot_unlock_data(&shot_data[0]);
Shot_fire(&shot_handle[1], &shot_data[1]);
next_time += time_stepp;
Filter_run_r0(Filter_ret_a, &Filter_inst[0], shot_data[0].data);

Fusion_eval_r0(Fusion_ret_x, Fusion_ret_y, &a_ref, vel);

vel_buffer[0][n_sub_meas - 2] = vel[0];
vel_buffer[1][n_sub_meas - 2] = vel[1];

while (__HAL_TIM_GET_COUNTER(&htim2) < next_time)
  ;

// Shot 3
Shot_unlock_data(&shot_data[1]);
Shot_fire(&shot_handle[2], &shot_data[2]);
next_time += time_stepp;
Filter_run_r0(Filter_ret_c, &Filter_inst[1], shot_data[1].data);
while (__HAL_TIM_GET_COUNTER(&htim2) < next_time)
  ;

// Shot 4
Shot_unlock_data(&shot_data[2]);
Shot_fire(&shot_handle[3], &shot_data[3]);
next_time += time_stepp;
Filter_run_r0(Filter_ret_b, &Filter_inst[2], shot_data[2].data);

// Filter Fusion for x dir
Filter_Fusion(Filter_ret_a, Filter_ret_b, Fusion_ret_x, a_ref);

while (__HAL_TIM_GET_COUNTER(&htim2) < next_time)
  ;

Shot_unlock_data(&shot_data[3]);
Filter_run_r0(Filter_ret_d, &Filter_inst[3], shot_data[3].data);

// Filter Fusion for y dir
Filter_Fusion(Filter_ret_c, Filter_ret_d, Fusion_ret_y, a_ref);

// Fusion eval
Fusion_eval_r0(Fusion_ret_x, Fusion_ret_y, &a_ref, vel);

vel_buffer[0][n_sub_meas - 1] = vel[0];
vel_buffer[1][n_sub_meas - 1] = vel[1];


//Mean filter with error detection
double sum0 = 0, sum1 = 0;
uint16_t diff = n_sub_meas;

for(uint16_t i = 0; i < n_sub_meas; i++){
  if(vel_buffer[0][i] < 9999 && vel_buffer[1][i] < 9999){
    sum0 += vel_buffer[0][i];
    sum1 += vel_buffer[1][i];
  }
  else {
   diff--;
  }
}


vel[0] = (sum0/diff);
vel[1] = (sum1/diff);

//Diff filter
diff = n_sub_meas;
sum0 = 0;
sum1 = 0;

for(uint16_t i = 0; i < n_sub_meas; i++){
  if(fabsf(vel_buffer[0][i] - vel[0]) < max_error || fabsf(vel_buffer[1][i] - vel[1]) < max_error){
    sum0 += vel_buffer[0][i];
    sum1 += vel_buffer[1][i];
  }
  else {
   diff--;
  }
}

vel[0] = (sum0/diff) - EEPROM_data.E_Meas.v_of[0];
vel[1] = (sum1/diff) - EEPROM_data.E_Meas.v_of[1];
}





float score_buffer[2][600];

void measurement_US_interleaved_PPR(uint8_t n_meas, float * vel, float * score, float * std) {
  time_stepp = MIN_MEAS_TIME;    // 900;//1250;//900;//690;
  sub_meas   = n_meas;

  // Start the measurement
  __HAL_TIM_SET_COUNTER(&htim2, 0);
  next_time = time_stepp;

  // Shot 1
  Shot_fire(&shot_handle[0], &shot_data[0]);
  while (__HAL_TIM_GET_COUNTER(&htim2) < next_time)
    ;

  // uint32_t p1 = __HAL_TIM_GET_COUNTER(&htim2);
  for (uint16_t i = 0; i < sub_meas - 1; i++) {
    // Shot 2
    Shot_unlock_data(&shot_data[0]);
    Shot_fire(&shot_handle[1], &shot_data[1]);
    next_time += time_stepp;
    Filter_run_r0(Filter_ret_a, &Filter_inst[0], shot_data[0].data);

    //Save smallest Peak score value in buffer
    if (Filter_ret_a[1].score < Filter_ret_a[3].score){
      score_buffer[0][i] = Filter_ret_a[1].score;
    }else {
      score_buffer[0][i] = Filter_ret_a[3].score;
     }


    if (i > 0) {
      Fusion_eval_r0(Fusion_ret_x, Fusion_ret_y, &a_ref, vel);

      vel_buffer[0][i - 1] = vel[0];
      vel_buffer[1][i - 1] = vel[1];
    }

    while (__HAL_TIM_GET_COUNTER(&htim2) < next_time)
      ;

    // Shot 3
    Shot_unlock_data(&shot_data[1]);
    Shot_fire(&shot_handle[2], &shot_data[2]);
    next_time += time_stepp;
    Filter_run_r0(Filter_ret_c, &Filter_inst[1], shot_data[1].data);

    score_buffer[1][i] = 0;

    while (__HAL_TIM_GET_COUNTER(&htim2) < next_time)
      ;

    // Shot 4
    Shot_unlock_data(&shot_data[2]);
    Shot_fire(&shot_handle[3], &shot_data[3]);
    next_time += time_stepp;
    Filter_run_r0(Filter_ret_b, &Filter_inst[2], shot_data[2].data);

    //Save smallest Peak score value in buffer
    if (Filter_ret_b[1].score < score_buffer[0][i]){
      score_buffer[0][i] = Filter_ret_b[1].score;
    }else if (Filter_ret_b[3].score < score_buffer[0][i]){
      score_buffer[0][i] = Filter_ret_b[3].score;
     }

    // Filter Fusion for x dir
    Filter_Fusion(Filter_ret_a, Filter_ret_b, Fusion_ret_x, a_ref);

    while (__HAL_TIM_GET_COUNTER(&htim2) < next_time)
      ;

    // Shot 1
    Shot_unlock_data(&shot_data[3]);
    Shot_fire(&shot_handle[0], &shot_data[0]);
    next_time += time_stepp;
    Filter_run_r0(Filter_ret_d, &Filter_inst[3], shot_data[3].data);

    //Save smallest Peak score value in buffer
    if (Filter_ret_d[1].score < score_buffer[1][i]){
      score_buffer[1][i] = Filter_ret_d[1].score;
    }else if (Filter_ret_d[3].score < score_buffer[0][i]){
      score_buffer[1][i] = Filter_ret_d[3].score;
     }

    // Filter Fusion for y dir
    Filter_Fusion(Filter_ret_c, Filter_ret_d, Fusion_ret_y, a_ref);

    while (__HAL_TIM_GET_COUNTER(&htim2) < next_time)
      ;
  }

  // Shot 2
  Shot_unlock_data(&shot_data[0]);
  Shot_fire(&shot_handle[1], &shot_data[1]);
  next_time += time_stepp;
  Filter_run_r0(Filter_ret_a, &Filter_inst[0], shot_data[0].data);

  //Save smallest Peak score value in buffer
    if (Filter_ret_a[1].score < Filter_ret_a[3].score){
      score_buffer[0][n_meas] = Filter_ret_a[1].score;
    }else {
      score_buffer[0][n_meas] = Filter_ret_a[3].score;
     }

  Fusion_eval_r0(Fusion_ret_x, Fusion_ret_y, &a_ref, vel);

  vel_buffer[0][sub_meas - 2] = vel[0];
  vel_buffer[1][sub_meas - 2] = vel[1];

  while (__HAL_TIM_GET_COUNTER(&htim2) < next_time)
    ;

  // Shot 3
  Shot_unlock_data(&shot_data[1]);
  Shot_fire(&shot_handle[2], &shot_data[2]);
  next_time += time_stepp;
  Filter_run_r0(Filter_ret_c, &Filter_inst[1], shot_data[1].data);

  //Save smallest Peak score value in buffer
    if (Filter_ret_c[1].score < Filter_ret_c[3].score){
      score_buffer[1][n_meas] = Filter_ret_c[1].score;
    }else {
      score_buffer[1][n_meas] = Filter_ret_c[3].score;
     }

  while (__HAL_TIM_GET_COUNTER(&htim2) < next_time)
    ;

  // Shot 4
  Shot_unlock_data(&shot_data[2]);
  Shot_fire(&shot_handle[3], &shot_data[3]);
  next_time += time_stepp;
  Filter_run_r0(Filter_ret_b, &Filter_inst[2], shot_data[2].data);

  
  //Save smallest Peak score value in buffer
    if (Filter_ret_b[1].score < score_buffer[0][n_meas]){
      score_buffer[0][n_meas - 1] = Filter_ret_b[1].score;
    }else if (Filter_ret_b[3].score < score_buffer[0][n_meas]){
      score_buffer[0][n_meas - 1] = Filter_ret_b[3].score;
     }

  // Filter Fusion for x dir
  Filter_Fusion(Filter_ret_a, Filter_ret_b, Fusion_ret_x, a_ref);

  while (__HAL_TIM_GET_COUNTER(&htim2) < next_time)
    ;

  Shot_unlock_data(&shot_data[3]);
  Filter_run_r0(Filter_ret_d, &Filter_inst[3], shot_data[3].data);

      //Save smallest Peak score value in buffer
    if (Filter_ret_d[2].score < score_buffer[1][n_meas]){
      score_buffer[1][n_meas - 1] = Filter_ret_d[2].score;
    }else if (Filter_ret_d[4].score < score_buffer[0][n_meas]){
      score_buffer[1][n_meas - 1] = Filter_ret_d[4].score;
     }


  // Filter Fusion for y dir
  Filter_Fusion(Filter_ret_c, Filter_ret_d, Fusion_ret_y, a_ref);

  // Fusion eval
  Fusion_eval_r0(Fusion_ret_x, Fusion_ret_y, &a_ref, vel);

  vel_buffer[0][sub_meas - 1] = vel[0];
  vel_buffer[1][sub_meas - 1] = vel[1];

  // Mean filters
  float sum0 = 0, sum1 = 0;
  uint16_t diff = sub_meas;

  for (uint16_t i = 0; i < sub_meas; i++) {
      sum0 += vel_buffer[0][i];
      sum1 += vel_buffer[1][i];    
  }

  vel[0] = (sum0 / diff);
  vel[1] = (sum1 / diff);


  sum0 = 0;
  sum1 = 0;
  for (uint16_t i = 0; i < sub_meas; i++) {
      sum0 += score_buffer[0][i];
      sum1 += score_buffer[1][i];    
  }

  score[0] = (sum0 / diff);
  score[1] = (sum1 / diff);

  
  //Evaluate Velocity empirical standard deviation this should be small << 1
  std[0] = 0;
  std[1] = 0;

    for (uint16_t i = 0; i < sub_meas; i++) {
      std[0] += fabsf(vel_buffer[0][i] - vel[0]);
      std[1] += fabsf(vel_buffer[1][i] - vel[1]);
  }
  std[0] = 1.0/(n_meas - 1) * std[0];
  std[1] = 1.0/(n_meas - 1) * std[1]; 
}
