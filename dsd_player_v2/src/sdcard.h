#ifndef __SD_CARD_H
#define __SD_CARD_H

#include "stm32f0xx_conf.h"
#include "spi.h"

uint8_t SD_Init(void);
typedef void (*tsdReadSectorCallBack)();
uint8_t SD_ReadSectorAsync(uint32_t nsector,uint8_t *buff,tsdReadSectorCallBack pCallBack);
uint8_t SD_ReadSectorSync(uint32_t nsector,uint8_t *buff,tsdReadSectorCallBack pCallBack);

/* SD card commands */
#define SD_GO_IDLE_STATE            0
#define SD_SEND_IF_COND             8
#define	SD_READ_SINGLE_BLOCK	    17
#define SD_WRITE_SINGLE_BLOCK       24
#define	SD_WRITE_MULTIPLE_BLOCK	    25
#define SD_SEND_OP_COND             41
#define SD_APP_CMD                  55
#define SD_READ_OCR                 58
#define	SD_READ_MULTIPLE_BLOCK	    18
#define	SD_STOP_TRANSMISSION	    12

#endif
