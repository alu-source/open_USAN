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
#pragma  once
#include "main.h"


typedef struct{
  uint16_t lin_index;
  int16_t cir_index;
  float peak_hight;
}peak_data_struct;

/**
  Init for Matrix and Vectors
*/                                                             
void helper_Init();

/**
  Returns an "in between" float Index
*/          
float helper_interpolate_Peak(float *data, peak_data_struct * max_index);

/**
  Returns fills the peak list with n_peaks left and n_peaks right of the highest Peak
  Peak_list must be of size 2*n_peaks + 1 
*/ 
void helper_get_Peaks(peak_data_struct* Peak_list, uint8_t n_peak, float* data);

/**
  Every Index over ALGO_BUFFER_SIZE/2 will be offset by -ALGO_BUFFER_SIZE
*/
int16_t helper_linear_to_circular_index(uint16_t index);

/**
  Every Index < 0 will be offset by + ALGO_BUFFER_SIZE
*/
uint16_t helper_circular_to_linear_index(int16_t index);

/**
  Searches for k smallest val in the array, only efficent for small k values
*/
void min_k_f32(float * input, uint8_t input_size, uint8_t k, uint8_t * idx, float * val);
