
#include "sdcard.h"
#include "memcpy_dma.h"

uint32_t sd_current_sector=-1;
uint32_t sd_data_wait_counter;
tsdReadSectorCallBack sd_rs_callback=0;
volatile uint8_t sdcard_busy=0;
//Type of the sd card
uint8_t  sdhc;

uint8_t SD_SendCmd(uint8_t cmd, uint32_t arg)
{
    uint8_t response, wait=0;

    if(!sdhc && (cmd == SD_READ_MULTIPLE_BLOCK ))
            arg = arg << 9; //Address fix for SD

    //Send command
    SPI_Send(cmd | 0x40);
    SPI_Send(arg>>24);
    SPI_Send(arg>>16);
    SPI_Send(arg>>8);
    SPI_Send(arg);

    //Send CRC
    if(cmd == SD_SEND_IF_COND)
        SPI_Send(0x87);
    else
        SPI_Send(0x95);

    //Wait for the responce
    while(((response = SPI_Read()) & 0x7f)==0x7f)
        if(wait++ > 64) break;

    //Check the responce if command is SD_READ_OCR
    if(response == 0x00 && cmd == SD_READ_OCR)
    {
        if(SPI_Read() & 0x40) sdhc = 1; //Card is SDHC
        else           sdhc = 0; //Card is SD

        SPI_Read();
        SPI_Read();
        SPI_Read();
    }

    SPI_Read();
    return response;
}

uint8_t SD_Init(void)
{
    uint8_t   i;
    uint8_t   response;
    uint8_t   SD_version = 2;
    uint16_t  retry = 0 ;

    CS_ENABLE;

    for(i=0; i<10; i++) SPI_Send(0xff);

    while(SD_SendCmd(SD_GO_IDLE_STATE, 0)!=0x01)
        if(retry++>0x20)
            return 1;

    retry = 0;
    while(SD_SendCmd(SD_SEND_IF_COND,0x000001AA)!=0x01)
    {
        if(retry++>64)
        {
            SD_version = 1;
            break;
        }
    }

    retry = 0;
    do
    {
        response = SD_SendCmd(SD_APP_CMD,0);
        if(response!=1) continue ;
        response = SD_SendCmd(SD_SEND_OP_COND,0x40000000);
        retry++;
        if(retry>64) return 1;
    }
    while(response != 0x00);

    retry = 0;
    sdhc = 0;
    if (SD_version == 2)
    {
        while(SD_SendCmd(SD_READ_OCR,0)!=0x00)
            if(retry++>64)  break;
    }

    CS_DISABLE;
    return 0;
}

void SPI_Callback_ReadSector(void *data_ptr)
{
    SPI_Read() ;
    SPI_Read() ;
    sd_current_sector++;
    CS_DISABLE;
    if(sd_rs_callback)
        sd_rs_callback();
    sd_rs_callback=0;
    sdcard_busy=0;
}

void SPI_CallBack_WaitForDataMarker(void *data_ptr)
{
    if((*(uint8_t*)DMA1_Channel2->CMAR)==0xFE)
        //The data is ready
        SPI_ReadViaDMA((void*)DMA1_Channel2->CMAR,512,SPI_Callback_ReadSector);
    else //wait
    {
        if(!(sd_data_wait_counter--)) NVIC_SystemReset();
        SPI_ReadViaDMA((void*)DMA1_Channel2->CMAR,1,SPI_CallBack_WaitForDataMarker);
    }

}

uint8_t SD_ReadSectorAsync(uint32_t nsector,uint8_t *buff,tsdReadSectorCallBack pCallBack)
{
    uint32_t retry=0;
    if(sdcard_busy)
        return(1);

    CS_ENABLE;
    sdcard_busy=1;

    if(nsector!=sd_current_sector)
    {
        if(sd_current_sector!=-1)
        {
            while(SD_SendCmd(SD_STOP_TRANSMISSION, 0))
                if(retry++>64)  NVIC_SystemReset();
        }

        while(SD_SendCmd(SD_READ_MULTIPLE_BLOCK, nsector))
            if(retry++>64)  NVIC_SystemReset();
        sd_current_sector=nsector;
        CS_ENABLE;
    }
    sd_rs_callback=pCallBack;
    sd_data_wait_counter=1024;
    SPI_ReadViaDMA(buff,1,SPI_CallBack_WaitForDataMarker);

    return(0);
}


uint8_t SD_ReadSectorSync(uint32_t nsector,uint8_t *buff,tsdReadSectorCallBack pCallBack)
{
    uint8_t result;

    result=SD_ReadSectorAsync(nsector,buff,pCallBack);
    while(sdcard_busy);
    return(result);
}


