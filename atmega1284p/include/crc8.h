#ifndef __CRC8_H__
#define __CRC8_H__
#include <stdint.h>
uint8_t crc8(const uint8_t *buffer, uint8_t count);
uint8_t crc8_256(const uint8_t *buffer, uint8_t start, uint8_t count);
#endif //__CRC8_H__
