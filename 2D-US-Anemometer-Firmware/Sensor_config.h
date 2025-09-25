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



#define d_0 0.0632 //Geom length of the measurement path
//if the length is changed the ADC offset has to be changed the fomular is as follows
//        (d_0/343)/(1/SHOT_FREQ) - 20 
#define ADC_OFFSET 120

#define SUB_MEASUREMENT_FREQ 200 //Up to 250Hz
#define CLOCK_MARGINE 0 // in us -> increase if clock drift is to large

//Key  and Calibration
#define ZERO_MEAS_SAMPLES 50 // Number of samples of the zero meas while testing 
//#define KEY_TEST // activates the KEY test part of the programm
#define KEY_DEFAULT 1926 //1814-> key for 14mm TR //1926 -> Key for 10mm TR //Here goes the chosen key of the test
#define CALIB_SAMPLES 600 // count of samples over which is averaged, for Calibration
#define VALID_SAMPLES 5 // count of samples over which is averaged, for  Validation

// Meas Paramter
#define MEASUREMENT_INTER_ERROR_CALIBRATION 10.0    //Allowed velocity diviation from mean, until sample is discarded, at calibration
#define MEASUREMENT_INTER_ERROR 25.0                //Allowed velocity diviation from mean, until sample is discarded, at normal meas
//#define EXPERIMENTAL_HIGH_SPEED_PEAK_SELCTION 0.01  //Allows the Algorithem to choose a peak based on feasbilty if blow threshhold, comment out to deactivate

// AXIS settings, changes are applied in this order!
#define SWAP_AXIS false // Swaps the X and Y outputs of the sensor
#define INVERT_X false // Invert X-axis
#define INVERT_Y true // Invert Y-axis
