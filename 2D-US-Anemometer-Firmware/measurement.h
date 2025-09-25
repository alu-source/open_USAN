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
#include "Shared.h"
#include "Unittest.h"


#define NO_ADAPTIV_TIMING     0
#define ADAPTIV_TIMING        1 
#define LAGACY_ADAPTIV_TIMING 2


/**
  Initiates the measurements from the storage object. 
    ->Must be called after each change to the storage object.
*/
void measurement_init(EEPROM_Meas_OBJ * meas_data);

/**
  Single measurement 
  not used, but useful for understanding the process
  */
void measurement_US(float * vel);

/**
  Takes zero measurement must be taken under no wind condition 
*/
void measurement_null (EEPROM_Filter_OBJ * filter_data);

/**
  Interleaved measurement for higher frequency measurements
*/
void measurement_US_interleaved(float * vel);

/**
  Same as interleaf just with out compensation and calibration used
  for Calibration
  -> Samples at 200 Hz
*/
void measurement_US_calibration(float * vel, uint16_t sub_meas, float max_error, uint8_t timing);

/**
  Used for exporting/testing interresting data for key testing 
  n_meas -> number of interleafed meas
  vel, score, std -> retuns 2D Vektor of said values
    std should be smaller than one other wise the key is bad !
    test under no wind speed condition ! 
*/
void measurement_US_interleaved_PPR(uint8_t n_meas, float * vel, float * score, float * std);
