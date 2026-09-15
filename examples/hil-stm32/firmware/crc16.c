/* CRC-16/MODBUS:poly 0x8005(反射 0xA001),init 0xFFFF,無最終 xor。
 * 「自己算 CRC 再驗自己」證明不了兩端算得一樣,橋接那邊有獨立實作與已知答案測試
 * ("123456789" -> 0x4B37)。 */
#include "crc16.h"

uint16_t crc16_modbus(const uint8_t *data, uint32_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint32_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++) {
            if (crc & 1) crc = (crc >> 1) ^ 0xA001;
            else         crc >>= 1;
        }
    }
    return crc;
}
