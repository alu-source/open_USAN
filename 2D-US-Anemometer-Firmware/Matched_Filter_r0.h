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
#pragma once

#include "main.h"
#include "Unittest.h"

#define ALGO_BUFFER_SIZE 512 //Must be of power of 2

typedef struct{
  float delay;
  float score;
}Filter_return_data;



typedef struct{
  uint16_t DMA_buffer_offset;
  uint8_t n_candidates; 
  float null_delay;
  float Filter_match[ALGO_BUFFER_SIZE]; 
  float Filter_match_raw[ALGO_BUFFER_SIZE];
}Filter_instance;


/**
  Init for the Filter and Filterrelated stuff
  Filter musst be array of 4
*/
void Filter_Init(Filter_instance * filter, EEPROM_Filter_OBJ * e_data);

/**
  Runs one instance of the Matchedfilter !!!
*/
uint8_t Filter_run_r0(Filter_return_data * data, Filter_instance * filter, uint16_t *raw_data)__attribute__ ((optimize(3)));
