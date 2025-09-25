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
#include "stdlib.h"
#include "main.h"
#include "Filter_Fusion_r0.h"
#include "stdlib.h"
#include "Sensor_config.h"
#include "correction.h"


#define W 0.005
#define F_a_ref 0.001


uint8_t Fusion_eval_r0(Fusion_return_candidate * Fusion_a,Fusion_return_candidate * Fusion_b,float * a_ref, float * v);


uint8_t Fusion_eval_init(EEPROM_Eval_OBJ * data){
  return EXIT_SUCCESS;
}

 __attribute__((always_inline)) inline uint8_t Fusion_eval_r0(Fusion_return_candidate * Fusion_a,Fusion_return_candidate * Fusion_b,float * a_ref, float * v){

  //Calc new matix with corrected speed of sound  
  float a_new_a[3][3];
  float a_new_b[3][3];

  for(uint8_t i = 0; i < 3; i++){
    for(uint8_t j = 0; j < 3; j++){
      a_new_a[i][j] = Fusion_a[i].a * Fusion_b[j].d_cor_factor;
      a_new_b[i][j] = Fusion_b[j].a * Fusion_a[i].d_cor_factor;
    }
  }

  //Score Calc
  float score [3][3];
  for(uint8_t i = 0; i < 3; i++){
    for(uint8_t j = 0; j < 3; j++){
      float a_score = fabsf(a_new_a[i][j] - a_new_b[i][j]) + fabsf((a_new_a[i][j] + a_new_b[i][j])/2 - *a_ref);
      float m_score = Fusion_a[i].score + Fusion_b[j].score;
      score[i][j] = a_score * W + m_score; 
    }
  }

  ////Get minimum
  float min_score = 999999;
  uint8_t index_i = 0;
  uint8_t index_j = 0;

  for(uint8_t i = 0; i < 3; i++){
    for(uint8_t j = 0; j < 3; j++){
      if(score[i][j] < min_score){
        min_score = score[i][j];
        index_i = i;
        index_j = j;
      }
    }
  }


  //Check for sanety if not sane output error val, so it can be later discared
  if(min_score > 100){
    v[0] = 99999;
    v[1] = 99999;
    return EXIT_FAILURE;
  }

  //Calc 
  v[0] = Fusion_a[index_i].vel * Fusion_b[index_j].d_cor_factor;
  v[1] = Fusion_b[index_j].vel * Fusion_a[index_i].d_cor_factor;


  if(Correction_max_v < v[0] || v[0] < - Correction_max_v){
    v[0] = 99999;
    v[1] = 99999;

    return EXIT_FAILURE;
  }else if (Correction_max_v < v[1] || v[1] < - Correction_max_v) {
    v[0] = 99999;
    v[1] = 99999;
    return EXIT_FAILURE;
  }
  else {
   *a_ref = (a_new_a[index_i][index_j] + a_new_b[index_i][index_j])/2 * F_a_ref + (1 - F_a_ref) * a_ref[0];
  }

  return EXIT_SUCCESS;
}

