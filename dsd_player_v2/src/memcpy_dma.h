#ifndef __DMA_MEMCPY_
#define __DMA_MEMCPY_

#include "stm32f0xx_conf.h"

void memcpy_dma(void *dest, void *src, uint32_t size_bytes);

void memcpy_dma_sync(void *dest, void *src, uint32_t size_bytes);

void memset_dma(void *mem, uint8_t data, uint32_t size_bytes);

void Memcpy_DMAInit();

#endif
