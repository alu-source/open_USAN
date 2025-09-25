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
#include "DAC_AD5732.h"
#include "EEPROM.h"
#include "packet_transfer.h"
#include "Shot.h"
#include "SRAM_IS66.h"
#include "correction.h"



typedef struct {
float match_raw[512];
}EEPROM_Filter_OBJ;


typedef struct {
  float t_0, l_corr_a, l_corr_b, l_corr_c;
}EEPROM_Fusion_OBJ;

typedef struct {
  float c1p,c1n,c2p,c2n;
}EEPROM_Eval_OBJ;

typedef struct {
  uint8_t mode;
  uint16_t freq;
  uint8_t duty;
  float a_ref;
  float v_of[2];
}EEPROM_Meas_OBJ;




typedef struct{
  EEPROM_Filter_OBJ E_Filter[4];
  EEPROM_Fusion_OBJ E_Fusion;
  EEPROM_Eval_OBJ E_Eval;
  EEPROM_Meas_OBJ E_Meas;
  EEPROM_Cali_OBJ E_Cali;
}EEPROM_OBJ;



/**
  Startup of the sensor
*/
uint8_t unit_startup(void);


/**
  Handles the usb interface
*/
uint8_t unit_usb(void);
