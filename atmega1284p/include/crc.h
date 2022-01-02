#ifndef __CRC8_H__
#define __CRC8_H__

#include <avr/pgmspace.h>
#include <avr/io.h>
#include <stdint.h>
#include <unistd.h>

uint8_t crc8(uint8_t *pcBlock, uint8_t len);
uint8_t crc8_with_init(uint8_t init_value, uint8_t *pcBlock, uint8_t len);
uint8_t crc8_byte(uint8_t old_crc, uint8_t byte);
#endif //__CRC8_H__
