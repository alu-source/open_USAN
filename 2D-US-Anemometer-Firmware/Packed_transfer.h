
#pragma  once
#include "main.h"




uint8_t packet_Init();

uint8_t packet_send(uint8_t data, uint16_t size);
uint8_t packet_send_unsave(uint8_t data, uint16_t size);  //Does not wate for Acknowledgment

uint8_t packet_read(uint8_t data, uint16_t size);
uint8_t packet_send_read(uint8_t data, uint16_t size);  //Does not wate for Acknowledgment

