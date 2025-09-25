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
#include "Matched_Filter_r0.h"
#include "Shot.h"
#include "Sensor_config.h"
#include "Unittest.h"
#include "packet_transfer.h"
#include "arm_math.h"

#define MAX_PEAKS 5      
#define MIN_DELAY -1 //in s
#define MAX_DELAY 1 //in s


//Interne Var
arm_rfft_fast_instance_f32 fftinstance __attribute__((section(".CCM_VAR")));  //Caution: CCM_VAR are not accessible by DMA but better for BUS allocation
float rfft_buffer[ALGO_BUFFER_SIZE] __attribute__((section(".CCM_VAR")));
float raw_buffer[ALGO_BUFFER_SIZE] __attribute__((section(".CCM_VAR")));
float filter_fft_buffer[ALGO_BUFFER_SIZE] __attribute__((section(".CCM_VAR")));
float filter_buffer[ALGO_BUFFER_SIZE] __attribute__((section(".CCM_VAR")));
 
peak_data_struct peak_buffer[MAX_PEAKS] __attribute__((section(".CCM_VAR")));

//Interne Func
void packed_complex_multiplication(float *pSrcA, float *pSrcB,float *pDst, uint32_t numSamples);

void Filter_Init(Filter_instance * filter, EEPROM_Filter_OBJ * e_data){
  helper_Init();
  arm_rfft_fast_init_f32(&fftinstance, ALGO_BUFFER_SIZE);
  for(uint8_t i = 0; i < 4; i++){
      //Flipp it
      for(uint16_t j = 0; j < ALGO_BUFFER_SIZE; j++){
        filter[i].Filter_match_raw[j] = (float)e_data[i].match_raw[ALGO_BUFFER_SIZE - 1 - j];
      }


      //Make avg = 0
      double avg = 0;

      for(uint16_t j = 0; j < ALGO_BUFFER_SIZE; j++){
          avg += filter[i].Filter_match_raw[j];
        }
        avg = avg / ALGO_BUFFER_SIZE;
        for(uint16_t j = 0; j < ALGO_BUFFER_SIZE; j++){
          filter[i].Filter_match_raw[j] -= avg;
        }

      //Pre FFT it !
      arm_rfft_fast_f32(&fftinstance, filter[i].Filter_match_raw, filter[i].Filter_match, 0);  

  }
}





 __attribute__((always_inline)) inline uint8_t Filter_run_r0(Filter_return_data* data, Filter_instance * filter, uint16_t *raw_data) {

  /*
    Filter data with matched filter
  */

  //Copy to the buffer
  for(uint16_t i = 0; i < ALGO_BUFFER_SIZE; i++){
    raw_buffer[i] = (float)raw_data[i + SHOT_ADC_OFFSET];
  }

  //transfer to frequency domain 
  arm_rfft_fast_f32(&fftinstance,raw_buffer,rfft_buffer, 0);  

  //Multiply
  packed_complex_multiplication(rfft_buffer, filter->Filter_match, filter_fft_buffer, ALGO_BUFFER_SIZE/2); 

  //transfer back to time domain    
  arm_rfft_fast_f32(&fftinstance,filter_fft_buffer, filter_buffer, 1);  
  
  //Get Peaks 
  helper_get_Peaks(peak_buffer, 2, filter_buffer);

  //Interpolate, Calc Delay and Score !!! 
  uint8_t error = 0;

  for(uint8_t i = 0; i < 5; i++){
    data[i].delay = helper_interpolate_Peak(filter_buffer, &peak_buffer[i]) / (SHOT_FREQ);
    if( MAX_DELAY < data->delay || data->delay < MIN_DELAY){
      error++;
    }

    float h1 =  (peak_buffer[2].peak_hight / peak_buffer[i].peak_hight);
    data[i].score = h1 * h1 * h1;

  }

  if(peak_buffer[2].cir_index > 36 || peak_buffer[2].cir_index < -36){
    error++;
  }
  return EXIT_SUCCESS;
}



 __attribute__((always_inline)) inline void packed_complex_multiplication(float * pSrcA, float * pSrcB,float * pDst, uint32_t numSamples)
{
  arm_cmplx_mult_cmplx_f32(pSrcA, pSrcB, pDst, numSamples);
}