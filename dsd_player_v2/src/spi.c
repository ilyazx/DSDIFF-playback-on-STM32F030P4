
#include "spi.h"

//Tx DMA var
const uint32_t dma_FFFF=0xFFFF;
spiCallBack spi_current_callback_function;
volatile uint8_t spi_read_temp;

void SPI_Init_()
{
    /* Enable GPIO clock */
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
    /* PA5/PA6/PA7 = Mode_AF, SCK/MISO/MOSI */
    GPIOA->MODER |= GPIO_MODER_MODER5_1 | GPIO_MODER_MODER6_1 |
                    GPIO_MODER_MODER7_1;
    /* GPIO_Speed_50MHz */
    GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR5_1 | GPIO_OSPEEDER_OSPEEDR6_1 |
                   GPIO_OSPEEDER_OSPEEDR7_1 ;
    /* Pins is pulled up */
    GPIOA->PUPDR |=  GPIO_PUPDR_PUPDR5_0 | GPIO_PUPDR_PUPDR6_0 |
                   GPIO_PUPDR_PUPDR7_0 ;
    /* PB1 = Mode_Out, CS */
    GPIOB->MODER |= GPIO_MODER_MODER1_0;
    /* GPIO_Speed_50MHz */
    GPIOB->OSPEEDR|=GPIO_OSPEEDER_OSPEEDR1_1;
    /* Enable the SPI1 */
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN ;
    /* Data size 8 bit, DMA TX RX requests enable */
    SPI1->CR2 = (7<<8) | SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN;
    /* SPI FIFO 8 bit */
    SPI1->CR2 |= SPI_CR2_FRXTH;
    /* Master, internal slave select, Soft NSS, SPI enable */
    SPI1->CR1 |= SPI_CR1_MSTR | SPI_CR1_SSI | SPI_CR1_SSM | SPI_CR1_SPE;
}

void SPI_DMAInit(void)
{
    SPI_Init_();

    /* Enable DMA1 clock */
    RCC->AHBENR|= RCC_AHBENR_DMA1EN;
    /* SPI disable */
    SPI1->CR1 &= ~SPI_CR1_SPE;

    /* DMA channel 3 - SPI TX */
    /* PerDst, (PrioLow) */
    DMA1_Channel3->CCR = DMA_CCR_DIR  ;
    DMA1_Channel3->CPAR =(uint32_t)&(SPI1->DR);
    DMA1_Channel3->CMAR=(uint32_t)&dma_FFFF;

    /* DMA channel 2 - SPI RX */
    /* MemInc, TC int en. (PrioLow) */
    DMA1_Channel2->CCR = DMA_CCR_MINC | DMA_CCR_TCIE ;
    DMA1_Channel2->CPAR =(uint32_t)&(SPI1->DR);

    /* Enable SPI DMA requests */
    SPI1->CR2 |= SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN;
    /* SPI Enable*/
    SPI1->CR1 |= SPI_CR1_SPE;

    /* IRQ priority */
    NVIC_SetPriority(DMA1_Channel2_3_IRQn,2);
    /* IRQ enable */
    NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);
}

/* Wait while SPI is busy */
void SPI_WaitWhileBusy()
{
    while ((SPI1->SR & SPI_SR_BSY));
}

/* Send a byte via SPI */
void SPI_Send (uint8_t data)
{
    /* Wait if TX buffer not empty */
    while(!(SPI1->SR & SPI_SR_TXE));
    /* Send byte */
    *(uint8_t*)&SPI1->DR=data;
}

/* Read a byte via SPI */
uint8_t SPI_Read (void)
{
    uint8_t result;
    while ((SPI1->SR & SPI_SR_RXNE)) //Flush RX buffer
        result=*((volatile uint8_t*)&SPI1->DR);

    /* Send byte */
    *(uint8_t*)&SPI1->DR=0xFF;
    /* Wait for data receive */
    while (!(SPI1->SR & SPI_SR_RXNE));
    result=*((volatile uint8_t*)&SPI1->DR);
    return (result);
}

void SPI_ReadViaDMA(void *pBuf,uint16_t data_size, spiCallBack pCallBack)
{
    SPI_WaitWhileBusy();
    CS_ENABLE;
    /* DMA channels disable */
    DMA1_Channel2->CCR &= ~DMA_CCR_EN; //RX
    DMA1_Channel3->CCR &= ~DMA_CCR_EN; //TX
    spi_current_callback_function=pCallBack;
    DMA1_Channel2->CMAR = (uint32_t)pBuf;

    DMA1_Channel3->CMAR = (uint32_t)&dma_FFFF;
    DMA1_Channel2->CNDTR = data_size; //RX
    DMA1_Channel3->CNDTR = data_size; //tX
    /* DMA channels enable */
    DMA1_Channel2->CCR |= DMA_CCR_EN; //RX
    DMA1_Channel3->CCR |= DMA_CCR_EN; //TX
}

/* SPI read/write */
void DMA1_Channel2_3_IRQHandler(void)
{
    if(DMA1->ISR & DMA_ISR_TCIF2) //RX complete
    {
        /* Clear TC interrupt flag */
        DMA1->IFCR = DMA_IFCR_CTCIF2;
        spi_current_callback_function((void*)DMA1_Channel2->CMAR);
    }
}


