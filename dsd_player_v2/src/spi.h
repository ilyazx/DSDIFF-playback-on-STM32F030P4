#ifndef __SPI_H
#define __SPI_H

#include "stm32f0xx_conf.h"

void SPI_Init_();
void SPI_DMAInit(void);
void SPI_Send (uint8_t data);
uint8_t SPI_Read (void);
void SPI_WaitWhileBusy();
typedef void (*spiCallBack)(void *data_ptr);
void SPI_ReadViaDMA(void *pBuf,uint16_t data_size, spiCallBack pCallBack);

#define CS_ENABLE   GPIOB->BSRR=GPIO_BSRR_BR_1
#define CS_DISABLE  GPIOB->BSRR=GPIO_BSRR_BS_1


#endif
