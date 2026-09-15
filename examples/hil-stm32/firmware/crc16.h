#ifndef CRC16_H
#define CRC16_H
#include <stdint.h>
uint16_t crc16_modbus(const uint8_t *data, uint32_t len);
#endif
