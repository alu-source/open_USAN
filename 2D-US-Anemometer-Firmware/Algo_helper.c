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
#include "Algo_helper.h"
#include "Shot.h"
#include "Matched_Filter_r0.h"
#include "arm_math.h"


void helper_Init();
float helper_interpolate_Peak(float *data, peak_data_struct * max_index);
void helper_get_Peaks(peak_data_struct* Peak_list, uint8_t n_peak, float* data);
int16_t helper_linear_to_circular_index(uint16_t index);
uint16_t helper_circular_to_linear_index(int16_t index);
void min_k_f32(float * input, uint8_t input_size, uint8_t k, uint8_t * idx, float * val);

//Matrix def for interpolation
float pA[21] = {9,-3,1,4,-2,1,1,-1,1,0,0,1,1,1,1,4,2,1,9,3,1};
arm_matrix_instance_f32 A = {7, 3, pA};
float pAt[21];
arm_matrix_instance_f32 At = {3, 7,pAt};
float pAtA[9];
arm_matrix_instance_f32 AtA = {3, 3,pAtA};
float pAtAinv[9];
arm_matrix_instance_f32 AtAinv = {3, 3,pAtAinv};
float pAusg[21];
arm_matrix_instance_f32 Ausg = {3, 7,pAusg};
float pb[7];
arm_matrix_instance_f32 b = {7, 1,pb};
float pabc[7];
arm_matrix_instance_f32 abc = {3, 1,pabc};


void helper_Init(void){
  arm_mat_trans_f32(&A, &At);
  arm_mat_mult_f32(&At, &A, &AtA);
  arm_mat_inverse_f32(&AtA, &AtAinv);
  arm_mat_mult_f32(&AtAinv, &At, &Ausg);
}


 __attribute__((always_inline)) inline float helper_interpolate_Peak(float *data, peak_data_struct * max_index){
  float flt_index = 0;

  //Filling b Vector with Peak data 
  for(int8_t i = 0; i < 7; i++){
    pb[i] = data[helper_circular_to_linear_index(max_index->cir_index + i -3)];
  }
  //Calc Interp Peak as Float index 
  arm_mat_mult_f32(&Ausg, &b, &abc);      
  flt_index = (float)max_index->cir_index - pabc[1]/2/pabc[0];
  return flt_index;
}



 __attribute__((always_inline)) inline void helper_get_Peaks(peak_data_struct *Peak_list, uint8_t n_peak, float *data) {

  // Get first Peak
  arm_max_f32(data, ALGO_BUFFER_SIZE, &Peak_list[n_peak].peak_hight,(uint32_t*)&Peak_list[2].lin_index);

  /**
    Every other peak will be to the left or right of the main peak by SHOT_DMA_CYCLES_PER_PERIOD
    due to sine wave nature of the signal
  */
  int16_t m_peak = (int16_t)Peak_list[2].lin_index;
  if(m_peak > 512/2){
    m_peak = m_peak - 512;
  }

  for (int8_t i = -n_peak; i < n_peak+1; i++) {
    int16_t index_val[5]; //holds the index of the viewed Val two left two right
    index_val[0] = m_peak + SHOT_DMA_CYCLES_PER_PERIODE * i - 2;
    index_val[1] = m_peak + SHOT_DMA_CYCLES_PER_PERIODE * i - 1;
    index_val[2] = m_peak + SHOT_DMA_CYCLES_PER_PERIODE * i + 0;
    index_val[3] = m_peak + SHOT_DMA_CYCLES_PER_PERIODE * i + 1;
    index_val[4] = m_peak + SHOT_DMA_CYCLES_PER_PERIODE * i + 2;

    uint8_t index = 0;
    float max_hight = data[helper_circular_to_linear_index(index_val[0])];

    for (uint8_t j = 1; j < 5; j++) {
      float hight = data[helper_circular_to_linear_index(index_val[j])];
      if (max_hight < hight) {
        index = j;
        max_hight = hight;
      }
    }

    //Save to Struct
    Peak_list[n_peak + i].peak_hight = max_hight;
    Peak_list[n_peak + i].cir_index = index_val[index];
    Peak_list[n_peak + i].lin_index = helper_circular_to_linear_index(index_val[index]);
  }
}

__attribute__((always_inline)) inline int16_t helper_linear_to_circular_index(uint16_t index){
  if(index >= ALGO_BUFFER_SIZE/2){
    return index - ALGO_BUFFER_SIZE;
  }
  else{
    return index;
  }

}

__attribute__((always_inline)) inline uint16_t helper_circular_to_linear_index(int16_t index){
  if(index < 0){
    return index + ALGO_BUFFER_SIZE;
  }
  else if (index >= ALGO_BUFFER_SIZE) {
    return index - ALGO_BUFFER_SIZE;
  }
  else{
    return index;
  }
}



 __attribute__((always_inline)) inline void min_k_f32(float * input, uint8_t input_size, uint8_t k, uint8_t * idx, float * val){
  float buffer[input_size];
  for (uint8_t i = 0; i < input_size; i++){
    buffer[i] = input[i];
  }

  for (uint8_t j = 0; j < k; j++) {
    idx[j] = 0;
    val[j] = buffer[0];
    for (uint8_t i = 0; i < input_size; i++) {
      if (val[j] > buffer[i]) {
        idx[j] = i;
        val[j] = buffer[i];
      }
    }
    buffer[idx[j]] = 9999999;
  }
}
