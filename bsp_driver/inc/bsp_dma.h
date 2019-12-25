#ifndef __BSP_DMA_H
#define __BSP_DMA_H

#include "stm32f10x.h"
void dma1_channel2_init(uint32_t PeripheralBaseAddr, uint32_t MemoryBaseAddr, uint32_t BufferSize, uint32_t DIR);
void dma1_channel3_init(uint32_t PeripheralBaseAddr, uint32_t MemoryBaseAddr, uint32_t BufferSize, uint32_t DIR);
void dma2_channel3_init(uint32_t PeripheralBaseAddr, uint32_t MemoryBaseAddr, uint32_t BufferSize, uint32_t DIR);
void dma2_channel5_init(uint32_t PeripheralBaseAddr, uint32_t MemoryBaseAddr, uint32_t BufferSize, uint32_t DIR);
#endif

