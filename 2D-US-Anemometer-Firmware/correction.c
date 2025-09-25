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
#include "correction.h"
#include "math.h"
#include "Sensor_config.h"


void correction_apply(float * v, EEPROM_Cali_OBJ * cali);


 __attribute__((always_inline)) inline void correction_apply(float * v, EEPROM_Cali_OBJ * cali){
  uint8_t flip   = 0;
  int8_t sign[2] = {1, 1};
  float v_x, v_y;
  
    v_x = fabsf(v[0]);
    v_y = fabsf(v[1]);

    if (v[0] < 0) {
      sign[0] = -1;
    }
    if (v[1] < 0) {
      sign[1] = -1;
    }

    // Get upper and lower points in the matrix !
    int16_t x_upper = ceilf(v_x / V_stepp) * sign[0];
    int16_t x_lower = floorf(v_x / V_stepp) * sign[0];
    int16_t y_upper = ceilf(v_y / V_stepp) * sign[1];
    int16_t y_lower = floorf(v_y / V_stepp) * sign[1];

    // To protect from out of bounds accsess
    if (x_upper > 90) {
      x_upper = 90;
    } else if (x_upper < -90) {
      x_upper = -90;
    }

    if (x_lower > 90) {
      x_lower = 90;
    } else if (x_lower < -90) {
      x_lower = -90;
    }

    if (y_upper > 90) {
      y_upper = 90;
    } else if (y_upper < -90) {
      y_upper = -90;
    }

    if (y_lower > 90) {
      y_lower = 90;
    } else if (y_lower < -90) {
      y_lower = -90;
    }


    // lets interpolate in the matix
    float v_x_c;
    float v_y_c;

    float x_y_upper = cali->corr_x_matrix[90 + x_lower][90 + y_upper] +
        (v[0] - x_lower * V_stepp) * (cali->corr_x_matrix[90 + x_upper][90 + y_upper] - cali->corr_x_matrix[90 + x_lower][90 + y_upper]) / ((float)V_stepp*sign[0]);
    float x_y_lower = cali->corr_x_matrix[90 + x_lower][90 + y_lower] +
        (v[0] - x_lower * V_stepp) * (cali->corr_x_matrix[90 + x_upper][90 + y_lower] - cali->corr_x_matrix[90 + x_lower][90 + y_lower]) / ((float)V_stepp*sign[0]);

    v_x_c = x_y_lower + (v[1] - y_lower * V_stepp) * ((x_y_upper - x_y_lower) / ((float)V_stepp*sign[1]));

    float y_y_upper = cali->corr_y_matrix[90 + x_lower][90 + y_upper] +
        (v[0] - x_lower  * V_stepp) * (cali->corr_y_matrix[90 + x_upper][90 + y_upper] - cali->corr_y_matrix[90 + x_lower][90 + y_upper]) / ((float)V_stepp*sign[0]);
    float y_y_lower = cali->corr_y_matrix[90 + x_lower][90 + y_lower] +
        (v[0] - x_lower * V_stepp) * (cali->corr_y_matrix[90 + x_upper][90 + y_lower] - cali->corr_y_matrix[90 + x_lower][90 + y_lower]) / ((float)V_stepp*sign[0]);

    v_y_c = y_y_lower + (v[1] / V_stepp - y_lower) * ((y_y_upper - y_y_lower) / ((float)V_stepp*sign[1]));

    v[0] = v_x_c;
    v[1] = v_y_c;
}
