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
#include "Algo_helper.h"
#include "stdlib.h"
#include "main.h"
#include "math.h"
#include "Matched_Filter_r0.h"
#include "Filter_Fusion_r0.h"
#include "Shared.h"
#include "Sensor_config.h"

float t_0;
float l_corr_a, l_corr_b, l_corr_c;

//List for speed of sound Combinations 
uint8_t a_combination_list_a[] __attribute__((section(".CCM_VAR"))) = {0,0,1,1,2,2,3,3,4};
uint8_t a_combination_list_b[] __attribute__((section(".CCM_VAR"))) = {0,1,1,2,2,3,3,4,4};
float a_vector[9] __attribute__((section(".CCM_VAR")));


//List for Score Combinations
uint8_t s_combination_list_a[] __attribute__((section(".CCM_VAR"))) = {0,0,0,0,0,1,2,3,4};
uint8_t s_combination_list_b[] __attribute__((section(".CCM_VAR"))) = {0,1,2,3,4,4,4,4,4};
uint8_t s_combination_list_length[] __attribute__((section(".CCM_VAR"))) = {1,2,3,4,5,4,3,2,1};


 
uint8_t Filter_Fusion(Filter_return_data * filter_a, Filter_return_data * filter_b,Fusion_return_candidate * fusion, float a_ref);

uint8_t Filter_Fusion_init(EEPROM_Fusion_OBJ * data){

  //Checking range of t_0
  if(data->t_0 < d_0/400 || data->t_0 > d_0/300){
    if (t_0 <= 0 || t_0 > 1) {
      Error_Handler(5);
      return EXIT_FAILURE;
    }
    Warning_Handler(5);
  }
  t_0 = data->t_0;

  // Checking range of l_corr_a
  if (data->l_corr_a < 0.9 || data->l_corr_a > 1.1) {
    if (data->l_corr_a <= 0 || data->l_corr_a > 2) {
      Error_Handler(5);
      return EXIT_FAILURE;
    }
    Warning_Handler(5);
  }
  l_corr_a = data->l_corr_a;

  // Checking range of l_corr_b
  if (data->l_corr_b < -0.1 || data->l_corr_b > 0.1) {
    if (data->l_corr_b < -1 || data->l_corr_b > 1) {
      Error_Handler(5);
      return EXIT_FAILURE;
    }
    Warning_Handler(5);
  }
  l_corr_b = data->l_corr_b;

  // Checking range of l_corr_c
  if (data->l_corr_c < -0.01 || data->l_corr_c > 0.01) {
    if (data->l_corr_c < -0.1 || data->l_corr_c > 0.1) {
      Error_Handler(5);
      return EXIT_FAILURE;
    }
    Warning_Handler(5);
  }
  l_corr_c = data->l_corr_c;

  return EXIT_SUCCESS;
}

 __attribute__((always_inline)) inline uint8_t Filter_Fusion(Filter_return_data * filter_a, Filter_return_data * filter_b, Fusion_return_candidate * fusion, float a_ref){

  float a_diff_vector[9];

  //Calc of meaning full speed of sound diff combinations
  for(uint8_t i = 0; i < 9; i++){
    //Mean time of transmission
    float h_time = t_0 + (filter_a[a_combination_list_a[i]].delay + filter_b[a_combination_list_b[i]].delay)/2;
    //Calc speed of sound
    float h_a = d_0/h_time;
    a_diff_vector[i] = fabsf(a_ref - h_a);
  }

  //Get the three val with the smallest diff
  uint8_t min_idx[3];
  float min_val[3];
  min_k_f32(a_diff_vector, 9, 3, min_idx, min_val);

  //Buffer for debug
  float vel[5];

  //Search for min score along the diagonal and paste to data struct
  for(uint8_t i = 0; i < 3; i++){

    uint8_t length = s_combination_list_length[min_idx[i]];
    uint8_t start_a = s_combination_list_a[min_idx[i]];
    uint8_t start_b = s_combination_list_b[min_idx[i]];
    float score_buffer[length];
    
    for(uint8_t j = 0; j < length; j++){
      //printf("Score %f\n", score_buffer[j]);

      //Maybe set fixed value ?! -> Should be inbetween
      //IF vel is out of range
      float h1 = 1/(t_0 + filter_a[start_a + j].delay); 
      float h2 = 1/(t_0 + filter_b[start_b - j].delay); 
      vel[j] = d_0/2 * (h1 - h2);

      // Experimental, if filter is very ambiguous, choose peak based on feasbility
      // If the filter is very ambiguous, the air speed will be high 
      // The air speed will not be larger than 60 m/s and smaller than 15 m/s
      // If jumping a peak the shift in speed will be about 50m/s 
      #ifdef EXPERIMENTAL_HIGH_SPEED_PEAK_SELCTION
      score_buffer[j] = filter_a[start_a + j].score + filter_b[start_b - j].score;
      if(score_buffer[j] < 2 + EXPERIMENTAL_HIGH_SPEED_PEAK_SELCTION){
        if(fabsf(vel[j]) < 15.0 || 60.0 < fabsf(vel[j])){
          score_buffer[j] = 9.0e20;
        }

      }else{
        if(vel[j] > 60.0 || vel[j] < -60.0){
          score_buffer[j] = 9.0e20;
        }
      }

      #else 
      // If vel is out of range mark with error value else copy score
      if(vel[j] > 60.0 || vel[j] < -60.0){
          score_buffer[j] = 9.0e20;
          }
      else{
        score_buffer[j] = filter_a[start_a + j].score + filter_b[start_b - j].score;
      }
      #endif
    }

    uint32_t min_idx_diagonal;
    arm_min_f32(&score_buffer[0], length, &fusion[i].score, &min_idx_diagonal);

    //Calc V
    uint8_t idx_a = start_a + (min_idx_diagonal);
    uint8_t idx_b = start_b - (min_idx_diagonal);
    float h1 = 1.0/(t_0 + filter_a[idx_a].delay); 
    float h2 = 1.0/(t_0 + filter_b[idx_b].delay); 

    float vel_2 = d_0/2.0 * (h1 - h2);
    fusion[i].vel = vel_2;

    //Calc l_corr
    float d_corr = l_corr_c * (vel_2*vel_2) + l_corr_b * fabs(vel_2) + l_corr_a;
    fusion[i].d_cor_factor = d_corr;

    //Calc speed of sound 
    fusion[i].a = d_0 /(((1/h1 + 1/h2)/2));

  }

  return EXIT_SUCCESS;
}







