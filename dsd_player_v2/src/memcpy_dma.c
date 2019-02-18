
#include "stm32f0xx_conf.h"
#include "memcpy_dma.h"

void memcpy_dma(void *dest, void *src, uint32_t size_bytes)
{
    if(DMA1_Channel4->CMAR != 0)
        while(!(DMA1->ISR & DMA_ISR_TCIF4));

    DMA1_Channel4->CCR&=~DMA_CCR_EN ;
    DMA1->IFCR|=DMA_IFCR_CTCIF4 | DMA_IFCR_CHTIF4;
    DMA1_Channel4->CMAR = (uint32_t)dest;
    DMA1_Channel4->CPAR = (uint32_t)src;
    DMA1_Channel4->CNDTR = size_bytes;
    DMA1_Channel4->CCR |= DMA_CCR_EN ;

}

void memcpy_dma_sync(void *dest, void *src, uint32_t size_bytes)
{
    memcpy_dma(dest,src,size_bytes);
    while(!(DMA1->ISR & DMA_ISR_TCIF4));
}



void memset_dma(void *mem, uint8_t data, uint32_t size_bytes)
{
    *(uint8_t*)mem = data;
    memcpy_dma(mem+1,mem,size_bytes-1);
}

void Memcpy_DMAInit()
{
    /* Enable DMA1 clock */
    RCC->AHBENR|= RCC_AHBENR_DMA1EN;
    /* Configure DMA channel 1 */
    // PerSrc, MemInc, PerInc, PrioMiddle, M2M
    DMA1_Channel4->CCR = DMA_CCR_MINC | DMA_CCR_PINC | DMA_CCR_PL_0 | DMA_CCR_MEM2MEM;
}
