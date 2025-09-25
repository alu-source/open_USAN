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
#include "DAC_AD5732.h"
#include "EEPROM.h"
#include "packet_transfer.h"
#include "Shot.h"
#include "SRAM_IS66.h"
#include "string.h"
#include "arm_math.h"
#include "Shared.h"
#include "uart.h"
#include <stdio.h>
#include "gpio.h"
#include "Unittest.h"
#include "measurement.h"
#include "Matched_Filter_r0.h"
#include "Sensor_config.h"
#include "OPT.h"
#include "correction.h"
#include "debug.h"
#include "Sensor_config.h"


#define CHECK_BIT(var,pos) (((var)>>(pos)) & 1)



//Randome  Junk
uint8_t Buffer[1024];
extern uart_t USART;
uint8_t key[SHOT_KEY_LENGTH];
gpio_pin_init_t gpio_pin_C9;
extern gpio_pin_t PE10;

extern uint8_t TRIGGER_EVENT_FLAG;
extern uint8_t TRIGGER_MODE_FLAG;
extern uint8_t ASCII_FLAG;
uint8_t EEPROM_FLAG = 0; // 0 if ram contense is the same as the eeprom content, and 1 if no longer valid !


//LEDs
extern gpio_pin_t LED_0,LED_1;

//Filter
extern Filter_instance Filter_inst[4];

//EEPROM 
EEPROM_OBJ EEPROM_data; 

//Later used for Measurements may move to Main 
Transceiver_handle shot_handle[4];
Shot_data_struct shot_data[4];


// Shortterm buffer
float c_buffer[2][50];

uint32_t crc;

uint8_t unit_test_fast(void);
uint8_t unit_test_full(void);
uint8_t unit_usb(void);


uint8_t unit_startup(){
  uint8_t error = 0;
  uint8_t crit_error = 0;

  //Clock Init
  System_clock_Init();
  HAL_Delay(100);
  packet_Init();
  HAL_Delay(100);
  //SRAM_Init();
  Shot_Init(shot_handle);

  //Convert key 
  for(uint8_t i = 0; i < SHOT_KEY_LENGTH; i++){
        key[i] = CHECK_BIT(KEY_DEFAULT,i); 
  }
  Shot_generate_gpio_from_Key(key, shot_handle);


  //Load EEPROM
  uint32_t crc_eeprom;
  if (EEPROM_read_data(4, (uint32_t*)&EEPROM_data, sizeof(EEPROM_data)/4) == EXIT_FAILURE || EEPROM_read_data(0,&crc_eeprom , 4/4) == EXIT_FAILURE) {
    char txt_buffer [70];
    uint8_t str_ln = sprintf(txt_buffer,"Could not read EEPROM!\n");
    uart_transmit(&USART,txt_buffer,str_ln);
  }

  //Check for CRC
  CRC_reset();
  crc = CRC_calculate((uint8_t*)&EEPROM_data,sizeof(EEPROM_data));

  if(crc != crc_eeprom){

    char txt_buffer [164];
    uint8_t str_ln = sprintf(txt_buffer,"EEPROAM is Corrupted!\n Sensor has to undergo initial setup!\n Nessesary stepps:\n\t->Zero measuremnt (-n)\n\t->Trigger config (-t)\n\t->Save to EEPROM (-w)\n\t->Exit (-e)\n\n");
    uart_transmit(&USART,txt_buffer,str_ln);
    EEPROM_FLAG = 1;
    
    // Init the  crossflow correction parameters -> have to be changed if sensor geom. changes
    float la = 0.984;
    float lb = 0.0003;
    float lc = 900e-10;

    EEPROM_data.E_Fusion.l_corr_a = la;
    EEPROM_data.E_Fusion.l_corr_b = lb;
    EEPROM_data.E_Fusion.l_corr_c = lc;

   gpio_pin_write(LED_1,1);
   gpio_pin_write(LED_0,0);

   while (42 != unit_usb()){
      //Disko machen !!!
      HAL_Delay(50);
      DAC_write(-4.5, -4.5, 1);
      gpio_pin_write(LED_1,0);
      gpio_pin_write(LED_0,1);
      HAL_Delay(50);
      DAC_write(4.5, 4.5, 1);
      gpio_pin_write(LED_1,1);
      gpio_pin_write(LED_0,0);
   }

  }


  //Init of the components
  measurement_init(&EEPROM_data.E_Meas);

  //DAC Init and calibration rountine, with smal in between stepps to limit overshoot 
  DAC_Init();
  for(uint8_t i = 0; i < 5; i++){
    DAC_write(-9.0, -9.0, 1);
    HAL_Delay(330);
    DAC_write(-4.5, -4.5, 1);
    HAL_Delay(3);
    DAC_write(0, 0, 1);
    HAL_Delay(330);
    DAC_write(4.5, 4.5, 1);
    HAL_Delay(3);
    DAC_write(9.0, 9.0, 1);
    HAL_Delay(330);
    DAC_write(4.5, 4.5, 1);
    HAL_Delay(2);
    DAC_write(-4.5, -4.5, 1);
    HAL_Delay(1);
  }

  //Test the TR 
  // Shot test...
  uint16_t index[4];
  uint16_t max[4];
  for (uint8_t i = 0; i < 4; i++) {
    HAL_Delay(20);
    Shot_fire(&shot_handle[i], &shot_data[i]);
    HAL_Delay(20);
    if (Shot_unlock_data(&shot_data[i]) != EXIT_SUCCESS) {
    }
    arm_max_q15((q15_t*)&shot_data[i].data, SHOT_DMA_LENGTH,(q15_t*)&max[i], (uint32_t*)&index[i]);
  }
  HAL_Delay(20);
  Shot_fire(&shot_handle[0], &shot_data[0]);
  HAL_Delay(20);
  if (Shot_unlock_data(&shot_data[0]) != EXIT_SUCCESS) {
  }
  arm_max_q15((q15_t*)&shot_data[0].data, SHOT_DMA_LENGTH,(q15_t*)&max[0], (uint32_t*)&index[0]);



  //Analyse 
  for(uint8_t i = 0; i < 4; i++){
    char txt_buffer [60];
    uint8_t str_ln = sprintf(txt_buffer,"Signal of Transceiver %d is %d.\n\n", i, max[i]);
    uart_transmit(&USART,txt_buffer,str_ln);

    if(max[i] < 2800){
    char txt_buffer [60];
    uint8_t str_ln = sprintf(txt_buffer,"Signal of Transceiver %d weak, check connections !\n\n", i);
    uart_transmit(&USART,txt_buffer,str_ln);
    crit_error ++;
    }
  }


  return EXIT_SUCCESS;
}

uint8_t unit_usb(void){
  //Did data arrive ?
  if(uart_bytesReadable(&USART) > 0){

    //Wait for transmittion to end
    HAL_Delay(10);

    uint16_t n_byte = uart_bytesReadable(&USART);
    uart_receive(&USART, Buffer, n_byte);

    //Null byte to end sting
    Buffer[n_byte +1] = '\0';

    //Get cmd start
    uint16_t cmd_start = 0;
    while(cmd_start < n_byte){
      if('-' == Buffer[cmd_start]){
        break;
      }
      cmd_start++;
    }

    uint8_t arg = 0;
    uint8_t exit_flag = 0;

    switch (Buffer[cmd_start + 1]) {
      case 'A':
      case 'a':
        int mode = 0;
        arg = sscanf((char*)Buffer + cmd_start +2 ,"%d",&mode);

        if(arg == 0){
          uint8_t tr_len;
          char txt_buffer[110];
          tr_len = sprintf(txt_buffer,"Mode missing, 0 to leave ASCII mode, 1 to enter continues ASCII mode, 2 to do a one time ASCII return\n\n");
          uart_transmit(&USART,txt_buffer,tr_len);
        }

        if(arg == 1){
          if(mode == 0){
            ASCII_FLAG = 0;
          }
          else if (mode == 1) {
            ASCII_FLAG = 1;
          }
          else if (mode == 2) {
            ASCII_FLAG = 2;
          }
          else{
            uint8_t tr_len;
            char txt_buffer[50];
            tr_len = sprintf(txt_buffer,"Mode not non\n");
            uart_transmit(&USART,txt_buffer,tr_len);
          }
        }
      break;


      case 'H' :
      case 'h':
        char txt_buffer[] =
        {"List of CMD:\n-h -> Help\n-s -> Current settings\n-t -> trigger [trigger_freq, duty in %]\n-s -> get current config\n-e -> exit menu\n-w -> write/save to EEPROM \n-n -> Zeromeasurement [temp in C], has to be taken under no wind speed condition\n-a -> ASCII mode\n\n"};
        uart_transmit(&USART,txt_buffer,sizeof(txt_buffer));
      break;

      case 'C':
      case 'c':
        mode = 0;
        arg = sscanf((char*)Buffer + cmd_start +2 ,"%d",&mode);
        if (mode == 0){ // RAW meas
          if(TRIGGER_MODE_FLAG != 2){
            TRIGGER_MODE_FLAG = 2;
            uint8_t length = sprintf(txt_buffer,"Cali_mode\n");
            uart_transmit(&USART,txt_buffer, length);
          }
          else {
            float vel[2];
            measurement_US_calibration(vel, CALIB_SAMPLES, MEASUREMENT_INTER_ERROR_CALIBRATION, ADAPTIV_TIMING);

            // For ASCII output of the Velocity -> can be ploted in the Arduino IDE
            //char txt_buffer[50];
            //uint8_t tr_length = sprintf(txt_buffer,"%f, %f\n",vel[0],vel[1]);
            //uart_transmit(&USART,txt_buffer,tr_length);

            // DEBUG_POINT_0
            CRC_reset();
            crc = CRC_calculate((uint8_t*)vel,8);
            uart_transmit(&USART,vel,8);
            uart_transmit(&USART,&crc,4);

             // Flipping and inverting axis as needed
            #if SWAP_AXIS == true
              float h1 = vel[0];
              vel[0] = vel[1];
              vel[1] = h1;
            #endif

            #if INVERT_X == true
              vel[0] = -vel[0];
            #endif

            #if INVERT_Y == true
              vel[1] = -vel[1];
            #endif

            DAC_write(vel[0] * (9.0/45.0),vel[1] * (9.0/45.0), 1);
          }
        }
        else if (mode == 1){ // Get calibration

            uint8_t length = sprintf(txt_buffer,"Listening\n");
            uart_transmit(&USART,txt_buffer, length);
            
            //Receive one line at a time 
            uint16_t time_out = 0; 
            uint8_t packet_retry = 0;

            //For X_corr
            for(uint8_t i = 0; i < Table_size; i++){

              time_out = 0;
              while (uart_bytesReadable(&USART) < Table_size * 4 + 4 && time_out < 1001) { //About 1s before timeout !
                time_out ++;
                HAL_Delay(1);
              }

              if(time_out >= 1000){ //When timed out about for loop
                i = 0xFF;
              }else{
                //Read data and check crc32
                uart_receive(&USART, &EEPROM_data.E_Cali.corr_x_matrix[i][0],Table_size * 4);
                uart_receive(&USART, &crc,4);

                CRC_reset();
                uint32_t packet_crc = CRC_calculate((uint8_t*)&EEPROM_data.E_Cali.corr_x_matrix[i][0], Table_size * 4);

                //Invalid crc
                if(packet_crc != crc){ 
                  i--; //Re do the packet !
                  packet_retry ++;
                  if(packet_retry > 20){
                    i = 0xFF;
                  }
                  uart_transmit(&USART,"FALSE\n", 6);
                }
                else {
                  uart_transmit(&USART,"TRUE\n", 5);
                }
              }
            }

            // For Y_corr
            for(uint8_t i = 0; i < Table_size; i++){

              time_out = 0;
              while (uart_bytesReadable(&USART) < Table_size * 4 + 4 && time_out < 1001) { //About 1s before timeout !
                time_out ++;
                HAL_Delay(1);
              }

              if(time_out >= 1000){ //When timed out about for loop
                i = 0xFF;
              }else{
                //Read data and check crc32
                uart_receive(&USART, &EEPROM_data.E_Cali.corr_y_matrix[i][0],Table_size * 4);
                uart_receive(&USART, &crc,4);

                CRC_reset();
                uint32_t packet_crc = CRC_calculate((uint8_t*)&EEPROM_data.E_Cali.corr_y_matrix[i][0], Table_size * 4);

                //Invalid crc
                if(packet_crc != crc){ 
                  i--; //Re do the packet !
                  packet_retry ++;
                  if(packet_retry > 20){
                    i = 0xFF;
                  }
                  uart_transmit(&USART,"FALSE\n", 6);
                }
                else {
                  uart_transmit(&USART,"TRUE\n", 5);
                }
              }
            }

        }
        else if (mode == 4){  // Get 100 raw and corrected values from the sensor -> trigger is used !

          //Wait for trigger sync
          TRIGGER_EVENT_FLAG = 0;
          while (TRIGGER_EVENT_FLAG == 0) {
            __NOP();
          }
          float vel_raw[2] = {0};
          float vel_corr[2] = {0};
          float v_transmit[4] = {0};

          for (uint8_t i = 0; i < 100; i++) { //About 4s
            

            // Wait for next trigger
            while(TRIGGER_EVENT_FLAG == 0){ //Waiting for trigger
              __NOP();
            }

            if(i > 0){
              DAC_write(vel_corr[0] * (9.0/45.0),vel_corr[1] * (9.0/45.0), 1);
            }

            TRIGGER_EVENT_FLAG = 0;

            measurement_US_calibration(vel_raw, VALID_SAMPLES, MEASUREMENT_INTER_ERROR, ADAPTIV_TIMING);
            vel_corr[0] = vel_raw[0];
            vel_corr[1] =  vel_raw[1];
            correction_apply(vel_corr, &EEPROM_data.E_Cali);
            v_transmit[0] = vel_raw[0];
            v_transmit[1] = vel_raw[1];
            v_transmit[2] = vel_corr[0];
            v_transmit[3] = vel_corr[1];

            CRC_reset();
            crc = CRC_calculate((uint8_t *)v_transmit, 4 * 4);
            uart_transmit(&USART, v_transmit, 4*4);
            uart_transmit(&USART, &crc, 4);
          }

          // Wait for next trigger
          while(TRIGGER_EVENT_FLAG == 0){ //Waiting for trigger
            __NOP();
          }
          DAC_write(vel_corr[0] * (9.0/45.0),vel_corr[1] * (9.0/45.0), 1);

        }
        else if (mode == 11) { // Exit Cali mode, enter nomrale operations again
          TRIGGER_MODE_FLAG = 1;
        } else if(mode == 12) {
          TRIGGER_MODE_FLAG = 0;
        }
        else if (mode == 7) { // Meas under old Calib condition
            float c_vel[2];
            measurement_US_calibration(c_vel, CALIB_SAMPLES, MEASUREMENT_INTER_ERROR_CALIBRATION, NO_ADAPTIV_TIMING);

            CRC_reset();
            crc = CRC_calculate((uint8_t*)c_vel,8);
            uart_transmit(&USART,c_vel,8);
            uart_transmit(&USART,&crc,4);
        }
        else if (mode == 8) { // Meas under old Valid condition
            double sum[2] = {0,0};
            float c_vel[2];

            for(uint8_t i = 0; i < 50; i++){ // To get the Voltage drop
              measurement_US_calibration(c_vel, VALID_SAMPLES, MEASUREMENT_INTER_ERROR, NO_ADAPTIV_TIMING);
              c_buffer[0][i] = c_vel[0];
              c_buffer[1][i] = c_vel[1];
            }
            for(uint8_t i = 0; i < 50; i++){
              measurement_US_calibration(c_vel, VALID_SAMPLES, MEASUREMENT_INTER_ERROR, NO_ADAPTIV_TIMING);
              c_buffer[0][i] = c_vel[0];
              c_buffer[1][i] = c_vel[1];
            }

            for(uint8_t i = 0; i < 50; i++){
              sum[0] = sum[0] + c_buffer[0][i];              
              sum[1] = sum[1] + c_buffer[1][i];
            }
            c_vel[0] = sum[0]/50.0;
            c_vel[1] = sum[1]/50.0;

            CRC_reset();
            crc = CRC_calculate((uint8_t*)c_vel,8);
            uart_transmit(&USART,c_vel,8);
            uart_transmit(&USART,&crc,4);
        }
        else if (mode == 9) { // Meas under old Meas condition
            double sum[2] = {0,0};
            float c_vel[2];

            for(uint8_t i = 0; i < 50; i++){
              measurement_US_calibration(c_vel, VALID_SAMPLES, MEASUREMENT_INTER_ERROR, LAGACY_ADAPTIV_TIMING);
              c_buffer[0][i] = c_vel[0];
              c_buffer[1][i] = c_vel[1];
            }
            for(uint8_t i = 0; i < 50; i++){
              measurement_US_calibration(c_vel, VALID_SAMPLES, MEASUREMENT_INTER_ERROR, LAGACY_ADAPTIV_TIMING);
              c_buffer[0][i] = c_vel[0];
              c_buffer[1][i] = c_vel[1];
            }

            for(uint8_t i = 0; i < 50; i++){
              sum[0] = sum[0] + c_buffer[0][i];              
              sum[1] = sum[1] + c_buffer[1][i];
            }
            c_vel[0] = sum[0]/50;
            c_vel[1] = sum[1]/50;

            CRC_reset();
            crc = CRC_calculate((uint8_t*)c_vel,8);
            uart_transmit(&USART,c_vel,8);
            uart_transmit(&USART,&crc,4);
        }
        else{
          uart_transmit(&USART,"Arg not known\n", 14);
        }

      break;

      case 'I':
      case 'i': 
        //ID of the Sensor
        if(Buffer[cmd_start + 2] == '?'){ //Get ID
          uint16_t ID;
          OPT_read(0, (uint8_t*)&ID, 2);
          uint8_t length = sprintf(txt_buffer,"SensorID:%i\n",ID);
          uart_transmit(&USART,txt_buffer, length);
        }
        else if (Buffer[cmd_start + 2] == '!') { //Set ID
          uint16_t ID;
          int h1;
          sscanf((char*)&Buffer[cmd_start + 3], "%d",&h1);
          ID = h1;
          uint8_t length = sprintf(txt_buffer,"Do you want to programm the ID to %d ?\n You can only programm it ONCE !\n",(uint16_t)h1);
          uart_transmit(&USART,txt_buffer, length);
          length = sprintf(txt_buffer,"Y/N\n");
          uart_transmit(&USART,txt_buffer, length);

          //Wait for confirmation
          while(uart_bytesReadable(&USART) < 1){
            HAL_Delay(1);
          }
          uint8_t h2;
          uart_receive(&USART, &h2, 1);
          if(h2 == 'Y' || h2 == 'y'){
            OPT_flash(0, (uint8_t*)&ID, 2);
            length = sprintf(txt_buffer,"Programmed ID\n");
            uart_transmit(&USART,txt_buffer, length);
          }
          else{
            length = sprintf(txt_buffer,"Programming aborted\n");
            uart_transmit(&USART,txt_buffer, length);
          }   
        }
      break;


      case 'T':
      case 't':
      int freq = 0;
      int duty = 100;

      arg = sscanf((char*)Buffer + cmd_start + 2,"%d %d",&freq , &duty);
      if(arg >= 1){ //freq was read
        if(freq >= 0.34 && freq <= 100){
          char txt_buffer[100];
          uint8_t length = sprintf(txt_buffer,"Meas. freq was set to: %d,",freq);
          EEPROM_data.E_Meas.freq = freq;
          uart_transmit(&USART,txt_buffer,length);
        }
        else if (freq == 0) { //Free Run 
          char txt_buffer[100];
          uint8_t length = sprintf(txt_buffer,"Triggering was turned off\n");
          EEPROM_data.E_Meas.freq = freq;
          uart_transmit(&USART,txt_buffer,length);
        
        }
        else if (freq > 100 || (freq < 1 && freq != 0)) { //out of range !
          char txt_buffer[100];
          uint8_t length = sprintf(txt_buffer,"Freq. was out of range !\n");
          uart_transmit(&USART,txt_buffer,length);
        
        }
        else {
          char txt_buffer[] = {"Error freq out off range !\n"};
          uart_transmit(&USART,txt_buffer,sizeof(txt_buffer));
        }

        if(arg == 2){
          char txt[100];
          if(freq != 0){
            if(duty <= 100 && duty >= 2){
              uint8_t length = sprintf(txt,"Dutycycle was set to %d %%\n",duty);
              EEPROM_data.E_Meas.duty = duty;
              uart_transmit(&USART,txt, length);
            } else if (duty == 1 ) {
              uint8_t length = sprintf(txt,"Meas was set to double shot\n");
              EEPROM_data.E_Meas.duty = 1;
              uart_transmit(&USART,txt, length);
            }
            else{
              uint8_t length = sprintf(txt,"Dutycycle was out of range and set to %d %%\n",100);
              EEPROM_data.E_Meas.duty = 100;
              uart_transmit(&USART,txt, length);
            }

          }else{
            if (freq == 0 && 1 <= duty && duty <= 100) {
              EEPROM_data.E_Meas.duty = duty; //here the dutycycle becomes the wanted frequency 
            }
            else {
              uint8_t length = sprintf(txt,"Freq was out of range and set to %d Hz\n",40);
              EEPROM_data.E_Meas.duty = 40;
              uart_transmit(&USART,txt,length);
            }
          }
        }
        else{
          if(freq != 0){
            char txt[100];
            uint8_t length = sprintf(txt,"Dutycycle set to default %d %%\n",100);
            EEPROM_data.E_Meas.duty = 100;
            uart_transmit(&USART,txt, length + 1);
          }else{
            char txt[100];
            uint8_t length = sprintf(txt,"Freq  set to default %d Hz\n",40);
            EEPROM_data.E_Meas.duty = 40;
            uart_transmit(&USART,txt,length);
          }
        }
        EEPROM_FLAG = 1;
      }

      //Config Meas. !
      measurement_init(&EEPROM_data.E_Meas);
      break;

      case 'S':
      case 's':
        uint16_t sensor_id;
        OPT_read(0,(uint8_t*)&sensor_id,2);
        uint16_t tr_len = sprintf(txt_buffer,"Current config of Sensor %d:\n",sensor_id);
        uart_transmit(&USART,txt_buffer,tr_len);
        tr_len = sprintf(txt_buffer,"Freq: %d Duty: %d\n",EEPROM_data.E_Meas.freq,EEPROM_data.E_Meas.duty);
        uart_transmit(&USART,txt_buffer,tr_len);
        tr_len = sprintf(txt_buffer,"Trigger mode: %d (1 -> Wait for trigger, 2 -> Calib mode, 0 -> free run)\n",TRIGGER_MODE_FLAG);
        uart_transmit(&USART,txt_buffer,tr_len);
        tr_len = sprintf(txt_buffer,"ASCII: %d (0 -> no ASCII output, 1 -> continues ASCII output, 2 -> one time ASCII output)\n",ASCII_FLAG);
        uart_transmit(&USART,txt_buffer,tr_len);
        tr_len = sprintf(txt_buffer,"L_corr: a = %f, b = %f, c = %f\n",EEPROM_data.E_Fusion.l_corr_a,EEPROM_data.E_Fusion.l_corr_b,EEPROM_data.E_Fusion.l_corr_c);
        uart_transmit(&USART,txt_buffer,tr_len);
        tr_len = sprintf(txt_buffer,"t_0: %f, a_0: %f\n",EEPROM_data.E_Fusion.t_0, EEPROM_data.E_Meas.a_ref);
        uart_transmit(&USART,txt_buffer,tr_len);
        tr_len = sprintf(txt_buffer,"Offsets: v_x = %f, v_y = %f\n",EEPROM_data.E_Meas.v_of[0], EEPROM_data.E_Meas.v_of[1]);
        uart_transmit(&USART,txt_buffer,tr_len);
        if(EEPROM_FLAG == 0){
          tr_len = sprintf(txt_buffer,"Eeprom is up to date\n");
          uart_transmit(&USART,txt_buffer,tr_len);
        }else{
          tr_len = sprintf(txt_buffer,"Eeprom is not up to date, consider writing updates the Eeprom with -w\n");
          uart_transmit(&USART,txt_buffer,tr_len);
        }

        tr_len = sprintf(txt_buffer,"Current status not smoking (hopefully)!\n\n");
        uart_transmit(&USART,txt_buffer,tr_len);
      break;

      case 'E':
      case 'e':
        if(EEPROM_FLAG == 1){
          uint16_t len = sprintf(txt_buffer,"Exiting... don't forget to save the changes with -w, otherwise the changes will be lost after powering down!\n\n");
          uart_transmit(&USART,txt_buffer,len);
        }else{
          uint16_t len = sprintf(txt_buffer,"Exiting...\n\n");
          uart_transmit(&USART,txt_buffer,len);
        }
        exit_flag = 42;
        return 42;
      break;

      /* //Used for config the cross flow parameters
      //case 'L':
      //case 'l':
      //  float la,lb,lc;
      //  arg = sscanf((char*)Buffer + cmd_start + 2,"%f %f %f", &la, &lb, &lc);

      //  if(arg == 3){
      //    sprintf(txt_buffer,"L cor factors set to: a = %f, b = %f, c = %f\n",la, lb , lc);
      //    tr_len = strlen(txt_buffer);
      //    uart_transmit(&USART,txt_buffer,tr_len);
      //    EEPROM_data.E_Fusion.l_corr_a = la;
      //    EEPROM_data.E_Fusion.l_corr_b = lb;
      //    EEPROM_data.E_Fusion.l_corr_c = lc;
      //  }
      //  else{
      //    //Default Val.
      //    la = 0.984;
      //    lb = 0.0003;
      //    lc = 900e-10;

      //    sprintf(txt_buffer,"L cor factors were missing set to default: a = %f, b = %f, c = %f\n",la, lb , lc);
      //    tr_len = strlen(txt_buffer);
      //    uart_transmit(&USART,txt_buffer,tr_len);
      //    EEPROM_data.E_Fusion.l_corr_a = la;
      //    EEPROM_data.E_Fusion.l_corr_b = lb;
      //    EEPROM_data.E_Fusion.l_corr_c = lc;
      //  }
      //break;
      */

      case 'W':
      case 'w':
          uart_transmit(&USART,"Start Saving\n",13);
          //Calc CRC
          CRC_reset();
          crc = CRC_calculate((uint8_t*)&EEPROM_data,sizeof(EEPROM_data));

          EEPROM_write_data(0, &crc, 1, 1);
          HAL_Delay(1);

          EEPROM_write_data(4, (uint32_t*)&EEPROM_data, sizeof(EEPROM_data)/4,0);
          uart_transmit(&USART,"Finished Saving\n",16);
          EEPROM_FLAG = 0;
      break;

      case 'N':
      case 'n':

        float temp = 0;
        arg = sscanf((char*)Buffer + cmd_start +2 ,"%f",&temp);

        if(arg == 1){
          EEPROM_data.E_Meas.a_ref = sqrt(1.4 * 287 * (273.15 + temp));
          EEPROM_data.E_Fusion.t_0 = d_0 / EEPROM_data.E_Meas.a_ref;
          measurement_null(EEPROM_data.E_Filter);

          sprintf(txt_buffer,"Zero measurment was recorded with: a_ref = %f m/s and t_0 = %f s\n",EEPROM_data.E_Meas.a_ref, EEPROM_data.E_Fusion.t_0);
          tr_len = strlen(txt_buffer);
          uart_transmit(&USART,txt_buffer,tr_len);
          Filter_Init(Filter_inst, EEPROM_data.E_Filter);
          EEPROM_FLAG = 1;
        }
        else{
          sprintf(txt_buffer,"Error temp is required!\n");
          tr_len = strlen(txt_buffer);
          uart_transmit(&USART,txt_buffer,tr_len);
        }
        
      break;

      case 'P':
      case 'p':
          sprintf(txt_buffer,"USAN\n");
          tr_len = strlen(txt_buffer);
          uart_transmit(&USART,txt_buffer,tr_len);

          //Output DAC pattern for 4s
          for(uint8_t i = 0; i < 4; i++){
            DAC_write(-9.0, -9.0, 1);
            HAL_Delay(330);
            DAC_write(-4.5, -4.5, 1);
            HAL_Delay(3);
            DAC_write(0, 0, 1);
            HAL_Delay(330);
            DAC_write(4.5, 4.5, 1);
            HAL_Delay(3);
            DAC_write(9.0, 9.0, 1);
            HAL_Delay(330);
            DAC_write(4.5, 4.5, 1);
            HAL_Delay(2);
            DAC_write(-4.5, -4.5, 1);
            HAL_Delay(1);
          }

      break;
    //case 'd':
    //case 'D':
      //int debug_key;
      //arg = sscanf((char*)Buffer + cmd_start + 2,"%d", &debug_key);
      
      //  //Use key debug function 
      //debug_send_shot_data((uint16_t)debug_key);
      //debug_timing();

      //break;
    
    }
    if(exit_flag == 42){
      return 42;
    }

  }
  return 0;  

}
