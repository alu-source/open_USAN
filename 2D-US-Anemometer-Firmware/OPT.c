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

#include "OPT.h"
#include "main.h"
#include "stdlib.h"
#include "stm32f4xx_hal_flash.h"




uint8_t OPT_flash(uint16_t addr, uint8_t * data, uint16_t size){
  
  //Write new data 
  if(addr + size > OPT_SIZE){
    return EXIT_FAILURE;
  }
  if(data == NULL){
    return EXIT_FAILURE;
  }
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPERR);

  HAL_FLASH_Unlock();

    uint32_t sofar=0;
    uint16_t numberofwords = size;
    uint32_t StartPageAddress = addr + OPT_BASE_ADR;

    	   while (sofar<numberofwords)
	   {
              uint32_t l_data = data[sofar];
	     if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, StartPageAddress, l_data) == HAL_OK)
	     {
	    	 StartPageAddress += 1;
	    	 sofar++;
	     }
	     else
	     {
	       /* Error occurred while writing data in Flash memory*/
	    	 return HAL_FLASH_GetError ();
	     }
	   }



    HAL_FLASH_Lock();
    return EXIT_SUCCESS;
}

uint8_t OPT_read(uint16_t addr, uint8_t * data, uint16_t size){
    if(addr + size > OPT_SIZE){
    return EXIT_FAILURE;
  }
  if(data == NULL){
      return EXIT_FAILURE;
  }

  uint32_t StartPageAddress = addr + OPT_BASE_ADR;
	while (1)
	{
          *data = *(__IO uint32_t *)StartPageAddress;
          StartPageAddress += 1;
          data++;
          if (!(size--)) break;
	}
  return EXIT_SUCCESS;




}