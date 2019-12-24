#ifndef __BSP_MODBUS_CRC16_H
#define __BSP_MODBUS_CRC16_H

#include "stm32f10x.h"

uint16_t CRC16_Modbus(uint8_t *_pBuf, uint16_t _usLen);

#endif

