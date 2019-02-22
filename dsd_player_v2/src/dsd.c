
#include "stm32f0xx_conf.h"
#include "dsd.h"

/* Period=(48000000/2822400)-1, for DSD64 */
#define TIMERS_PERIOD 16
/* Buffer for data tranfer to GPIO via DMA */
#define DSD_DMA_BUFFER_SZIE 512
uint8_t dsd_dma_buffer[DSD_DMA_BUFFER_SZIE] __attribute__((aligned (4)));;
uint8_t *dsd_dma_buffer_low=dsd_dma_buffer;
uint8_t *dsd_dma_buffer_hi=&dsd_dma_buffer[DSD_DMA_BUFFER_SZIE/2];

uint8_t *dsd_read_buffer_low;
uint8_t *dsd_read_buffer_hi;
uint8_t *dsd_read_buffer_top;
uint8_t * volatile dsd_buffer_read_ptr;

const unsigned int dsd_ccr_table[]={
0x00000000,0xFF000000,0x00FF0000,0xFFFF0000,
0x0000FF00,0xFF00FF00,0x00FFFF00,0xFFFFFF00,
0x000000FF,0xFF0000FF,0x00FF00FF,0xFFFF00FF,
0x0000FFFF,0xFF00FFFF,0x00FFFFFF,0xFFFFFFFF
};

__attribute__((optimize("-O3"))) uint8_t *dsd_translate_ccr(uint8_t *src, uint32_t *dest)
{
    uint32_t i=0;

    while(i<DSD_DMA_BUFFER_SZIE/32)
    {
        /* Left channel | Right channel */
        *(dest+DSD_DMA_BUFFER_SZIE/8)=dsd_ccr_table[src[1]>>4 & 0x0F];
        *(dest++)=dsd_ccr_table[src[0]>>4 & 0x0F];
        *(dest+DSD_DMA_BUFFER_SZIE/8)=dsd_ccr_table[src[1] & 0x0F];
        *(dest++)=dsd_ccr_table[src[0] & 0x0F];
        src+=2;
        i++;
    }
    return(src);
}

/* Fill low part of DSD DMA buffer */
uint8_t *DSD_WriteLow_msb(uint8_t * buf)
{
    return (dsd_translate_ccr(buf, (uint32_t*)dsd_dma_buffer_low));
}

/* Fill high part of DSD DMA buffer */
uint8_t *DSD_WriteHigh_msb(uint8_t * buf)
{
    return (dsd_translate_ccr(buf, (uint32_t*)(dsd_dma_buffer_low+128)));
}

void DSD_Init(uint8_t *read_buffer, uint32_t read_buffer_size)
{
    /* GPIOA clock enable */
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
    /* PA9/PA10 = Mode_AF, TIM1_CH2/TIM1_CH3 */
    GPIOA->MODER |= GPIO_MODER_MODER9_1 | GPIO_MODER_MODER10_1;
    /* PA9/PA10 = Mode_AF_2 */
    GPIOA->AFR[1]  |= (GPIO_AF_2 << (4*(9-8))) | (GPIO_AF_2 << (4*(10-8)));
    /* PA9/PA10 GPIO_Speed_50MHz */
    GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR9_1 | GPIO_OSPEEDER_OSPEEDR10_1;
    /* Pins is pulled up */
    GPIOA->PUPDR |= GPIO_PUPDR_PUPDR4_0 | GPIO_PUPDR_PUPDR9_0 ;

    /* TIM17 clock enable */
    RCC->APB2ENR |= RCC_APB2ENR_TIM17EN;
    /* TIM17 period */
    TIM17->ARR=TIMERS_PERIOD;
    TIM17->CNT=0;
    /* 1: CCx DMA requests sent when update event occurs */
    TIM17->CR2|=TIM_CR2_CCDS;
    /** The OC1 output of the TIM17 is used to reset the TIM1
    for delay between the transactions DMA channels 1 and 5,
    otherwise the transactions can interfere with each other
    due to lack of system resources when shared access occurs.
    Reset occurs after two counts (2 clocks of the AHB clock in this case)
    of the TIM1 after the TIM17_OC1 rising edge occurs.*/
    TIM17->CCR1 = 2; //Delay 4 clocks
    /*  PWM mode 2 - Active = (TIMx_CNT>=TIMx_CCR1) */
    TIM17->CCMR1 |= TIM_CCMR1_OC1M ;
    /* CC1 output enable, active level is high (CC1P=0)*/
    TIM17->CCER |= TIM_CCER_CC1E ;
    /* Main output enable */
    TIM17->BDTR |= TIM_BDTR_MOE;
    /* Enable TIM17 DMA interface */
    TIM17->DIER =  TIM_DIER_UDE;
    /* Only counter overflow generates an update interrupt or DMA request. */
    TIM17->CR1 |= TIM_CR1_URS;

    /* DMA1 enable */
    RCC->AHBENR|= RCC_AHBENR_DMA1EN;
    /* DMA channel 1 configuration */
    DMA1_Channel1->CPAR =(uint32_t)&TIM1->CCR3; ;
    DMA1_Channel1->CMAR=(uint32_t)dsd_dma_buffer_hi;
    DMA1_Channel1->CNDTR=sizeof(dsd_dma_buffer)/2;
    // PerDst, MemInc, CircEn, PrioVH, PerSize = 16 bit
    DMA1_Channel1->CCR=DMA_CCR_DIR | DMA_CCR_MINC | DMA_CCR_CIRC | DMA_CCR_PL
                       | DMA_CCR_PSIZE_0 ;
    /* DMA channel 1 enable */
    DMA1_Channel1->CCR|=DMA_CCR_EN ;

    /* TIM1 clock enable */
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
    /* Configure DMA channel 5 */
    DMA1_Channel5->CPAR =(uint32_t)&TIM1->CCR2;
    DMA1_Channel5->CMAR=(uint32_t)dsd_dma_buffer_low;
    DMA1_Channel5->CNDTR=sizeof(dsd_dma_buffer)/2;
    // PerDst, MemInc, CircEn, PrioVH, HTIE,TCIE, PerSize = 16 bit
    DMA1_Channel5->CCR=DMA_CCR_DIR | DMA_CCR_MINC | DMA_CCR_CIRC | DMA_CCR_PL
                       | DMA_CCR_HTIE | DMA_CCR_TCIE | DMA_CCR_PSIZE_0 ;
    /* DMA channel 5 enable */
    DMA1_Channel5->CCR|=DMA_CCR_EN ;
    /* Enable DMA IRQ */
    NVIC_EnableIRQ(DMA1_Channel4_5_IRQn);
    /* IRQ priority VERY HIGH */
    NVIC_SetPriority(DMA1_Channel4_5_IRQn,0);

    /* TIM1 period */
    TIM1->ARR=TIMERS_PERIOD;
    TIM1->CNT=0;
    /*  PWM mode 1 - Active = (TIMx_CNT<TIMx_CCR1). Preload enable */
    TIM1->CCMR1 = TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2PE;
    /*  PWM mode 1 - Active = (TIMx_CNT<TIMx_CCR1) Preload enable */
    TIM1->CCMR2 = TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3PE;
    /* OC2,OC3 enable, CCxP = 0: OCx active high */
    TIM1->CCER = TIM_CCER_CC2E | TIM_CCER_CC3E ;
    /* Main output enable */
    TIM1->BDTR = TIM_BDTR_MOE;
    /* TIM1 DMA interface enable */
    TIM1->DIER =  TIM_DIER_UDE;
    /* TIM1 is slave, TS=ITDR3=TIM17_OC, SMS=reset mode */
    TIM1->SMCR=TIM_SMCR_TS_0 | TIM_SMCR_TS_1 | TIM_SMCR_SMS_2 ;

    /* TIM17 enable */
    TIM17->CR1 |= TIM_CR1_CEN;
    /* TIM1 enable*/
    TIM1->CR1 |= TIM_CR1_CEN;

    dsd_read_buffer_low=dsd_buffer_read_ptr=read_buffer;
    dsd_read_buffer_hi=read_buffer+read_buffer_size/2;
    dsd_read_buffer_top=read_buffer+read_buffer_size;
}

uint8_t DSD_Reconfigure(dsf_info_t *di)
{
    /* If the HSE is not active exit */
    if(!(RCC->CR | RCC_CR_HSERDY))return(1);
    /* Wait, if SPI busy*/
    while(SPI1->SR & SPI_SR_BSY);
    /* SPI disable */
    SPI1->CR1 &= ~SPI_CR1_SPE;
    /* Reset SPI divider to 4 */
    SPI1->CR1 &= ~(SPI_CR1_BR_0|SPI_CR1_BR_1|SPI_CR1_BR_2);
    /* Switch PCLK to HSI */
    RCC_SYSCLKConfig(RCC_SYSCLKSource_HSI);
    /* Disable PLL */
    RCC->CR &=~ RCC_CR_PLLON;
    if(di->sample_rate == 5644800)
    {
        /* DSD128 mode */
        /* Set SPI divider to 4 */
        //SPI1->CR1 |= SPI_CR1_BR_1;
        /* Overclock!!! */
        //RCC_PLLConfig(RCC_PLLSource_HSE, RCC_PLLMul_6);
    }
    else
        RCC_PLLConfig(RCC_PLLSource_HSE, RCC_PLLMul_6);

    /* Enable PLL: once the PLL is ready the PLLRDY */
    RCC->CR |= RCC_CR_PLLON;
    /* Wait the PLL */
    while (!(RCC->CR & RCC_CR_PLLRDY));
    /* SPI enable */
    SPI1->CR1 |= SPI_CR1_SPE;
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
    return(0);
}

uint8_t *DSD_GetCurrentBufferPosition()
{
    return(dsd_buffer_read_ptr);
}
__attribute__((optimize("-O3"))) void DMA1_Channel4_5_IRQHandler(void)
{
    /* Low part of DSD DMA buffer was played */
    if(DMA1->ISR & DMA_ISR_HTIF5)
    {
        /* Clear the HT interrupt flag */
        DMA1->IFCR = DMA_IFCR_CHTIF5;
        /* Fill low part of the DSD DMA buffer ~1300 ticks */
        dsd_buffer_read_ptr=DSD_WriteLow_msb(dsd_buffer_read_ptr);
    }
    else

        /* High part of DSD DMA buffer was played */
        if(DMA1->ISR & DMA_ISR_TCIF5)
        {
            /* Clear the TC interrupt flag */
            DMA1->IFCR = DMA_IFCR_CTCIF5;
            /* Fill high part of the DSD DMA buffer ~1300 ticks */
            dsd_buffer_read_ptr=DSD_WriteHigh_msb(dsd_buffer_read_ptr);

        }

    /* Go to begin of the buffer */
    if(dsd_buffer_read_ptr>=dsd_read_buffer_top)
        dsd_buffer_read_ptr=dsd_read_buffer_low;
}
