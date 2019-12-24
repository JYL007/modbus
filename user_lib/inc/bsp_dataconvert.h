#ifndef __BSP_DATACONVERT_H
#define __BSP_DATACONVERT_H
#include "stm32f10x.h"

int16_t BEBufToInt16(uint8_t *_pBuf);
int16_t LEBufToInt16(uint8_t *_pBuf);
uint16_t BEBufToUint16(uint8_t *_pBuf);
uint16_t LEBufToUint16(uint8_t *_pBuf);

int32_t BEBufToInt32(uint8_t *_pBuf);
int32_t LEBufToInt32(uint8_t *_pBuf);
uint32_t BEBufToUint32(uint8_t *_pBuf);
uint32_t LEBufToUint32(uint8_t *_pBuf);


#endif
