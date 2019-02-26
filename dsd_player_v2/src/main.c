/*
**
**                           Main.c
**
**
**********************************************************************/

#include "spi.h"
#include "sdcard.h"
#include "dsd.h"
#include "memcpy_dma.h"
#include "pff\diskio.h"
#include "pff\pff.h"
#include "string.h"

#define PD_PLAY         0
#define PD_NEXT_FILE    1

FATFS fs;			/* File system object */
DIR dir;			/* Directory objects */
FILINFO fno;		/* File information */
UINT dwBytesRead=0;
#define PATH_SIZE (64)
char current_dir_path[PATH_SIZE];

/* Buffer for reading data from the SD card. Top of it is very close to the stack! */
#define SDCARD_READ_BUFFER_SIZE (1024+512+256)

uint8_t sdcard_read_buffer[SDCARD_READ_BUFFER_SIZE] __attribute__((aligned (4)));
uint8_t *sdcard_read_buffer_low=sdcard_read_buffer;
uint8_t *sdcard_read_buffer_hi=&sdcard_read_buffer[SDCARD_READ_BUFFER_SIZE/2];


void Init_GPIO()
{
    /* Enable the GPIOA clock */
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
    /* Pin PA10 is pulled up (button) */
    GPIOA->PUPDR |= GPIO_PUPDR_PUPDR4_0 | GPIO_PUPDR_PUPDR9_0 | GPIO_PUPDR_PUPDR10_1;
}

void Clocks_Init()
{
    uint32_t timeout=4096;
    /* Enable HSE */
    RCC->CR |= RCC_CR_HSEON;
    while(!(RCC->CR | RCC_CR_HSERDY) && timeout) timeout--;
    if(timeout) //HSE is ready
        RCC->CFGR |= RCC_CFGR_PLLSRC_HSE_PREDIV | RCC_CFGR_PLLMUL6;
    else //HSE is not ready
        RCC->CFGR |= RCC_CFGR_PLLSRC_HSI_PREDIV | RCC_CFGR_PLLMUL12;
    /* Enable PLL: once the PLL is ready the PLLRDY */
    RCC->CR |= RCC_CR_PLLON;
    /* Wait till PLL is ready */
    timeout=128;
    while (!(RCC->CR & RCC_CR_PLLRDY) && timeout) timeout--;
    if(timeout)
        /* Select PLL as system clock source */
        RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_PLL;
    else
        RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_HSI;
}

/* Reads dff file and writes data to dsf_info_t struct */
uint8_t Parse_DFF(dsf_info_t *di)
{
    uint32_t temp,file_pointer=16;
    uint32_t chSize; /* To do. Files large than 4GB not supported */
    pf_lseek(0);

    temp=0;
    pf_read(&temp,4,&dwBytesRead);
    if(temp != *(uint32_t*)"FRM8")return(1);
    pf_lseek(12);
    pf_read(&temp,4,&dwBytesRead);
    if(temp != *(uint32_t*)"DSD ")return(1);
    while(1)
    {
        /* Read chunk name */
        pf_read(&temp,4,&dwBytesRead);
        pf_lseek(file_pointer+8); ///Low dword of chSize
        pf_read(&chSize,4,&dwBytesRead);
        /* Big endian -> little endian */
        chSize=__builtin_bswap32(chSize);
        if(chSize % 2) chSize++;
        switch(temp)
        {
        case 0x504F5250: //PROP
            file_pointer+=12+4;
            pf_lseek(file_pointer);
            break;
        case 0x52504D43: //CMPR
            pf_read(&temp,4,&dwBytesRead);
            if(temp != *(uint32_t*)"DSD ") return(1); ///Error, data is compressed
            file_pointer+=chSize+12;
            pf_lseek(file_pointer);
            break;
        case 0x20445344: //DSD
            di->data_size = (uint32_t)chSize;
            di->dsd_data_offset = file_pointer+12;
            return(0);
            /* To do
            case "MLFT": //MLFT
            case "MRGT": //MRGT
            case "LS  ": //LS
            case "RS  ": //RS
            case "    ": //C
            case "LFE ": //LFE
            //Error, supported 2 channels stereo so.
            return(1);
            */
        case 0x20205346: //FS
            pf_read(&di->sample_rate,4,&dwBytesRead); //Sample rate
            di->sample_rate=__builtin_bswap32(di->sample_rate);
            file_pointer+=chSize+12;
            pf_lseek(file_pointer);
            break;
        default: //skip chunk
            file_pointer+=chSize+12;
            pf_lseek(file_pointer);
        }
    }

    return(0);
}

void Path_Append(char *path, char *data)
{
    if(path[0]==0)
        strcpy(path,data);
    else
    {
        strcat(path,"/");
        strcat(path,fno.fname);
    }
}

void Path_GoBack(char *path,char *removed_name)
{
    int i=strlen(path);
    while((path[i] != '/') && i) i--;
    if(removed_name)
        strcpy(removed_name,&path[(i==0) ? i : i+1]);
    path[i]=0;
}

uint8_t Process_Dirs()
{
    int result;
    dsf_info_t di;
    char dir_name[12];

    pf_readdir(&dir, &fno);

    if(!fno.fname[0]) //End of directory
    {
        //Return back or open root dir
        if(current_dir_path[0]==0)
            pf_opendir(&dir,current_dir_path); //open root dir
        else
        {
            /* Separate current dir name and path */
            Path_GoBack(current_dir_path,dir_name);
            pf_opendir(&dir,current_dir_path);
            /* Find directory */
            do pf_readdir(&dir, &fno);
            while(strcmp(fno.fname,dir_name));
        }
        return(PD_NEXT_FILE);
    }
    if(fno.fattrib == 16)
    {
        Path_Append(current_dir_path,fno.fname);
        pf_opendir(&dir,current_dir_path);
        return(PD_NEXT_FILE);
    }
    Path_Append(current_dir_path,fno.fname);
    result=pf_open(current_dir_path);
    Path_GoBack(current_dir_path,0);
        if(result)return(PD_NEXT_FILE); //Error open file, skip
    if(!Parse_DFF(&di))
        DSD_Reconfigure(&di);
    else //error or file isn't dff
        return(PD_NEXT_FILE);

    return(PD_PLAY);
}

int main(void)
{
    uint32_t result;

    /* Stop timers when the core is halted  */
    RCC->APB2ENR |= RCC_APB2ENR_DBGMCUEN;
    DBGMCU->APB2FZ |= DBGMCU_APB2_FZ_DBG_TIM1_STOP;
    DBGMCU->APB2FZ |= DBGMCU_APB2_FZ_DBG_TIM16_STOP;

    Clocks_Init();
    Init_GPIO();
    SPI_DMAInit();
    Memcpy_DMAInit();
    disk_initialize();
    result=pf_mount(&fs);
    if(result)NVIC_SystemReset();
    current_dir_path[0]=0;
    result=pf_opendir(&dir,current_dir_path);
    if(result)NVIC_SystemReset();
    DSD_Init(sdcard_read_buffer,SDCARD_READ_BUFFER_SIZE);

    dwBytesRead=0;
    while(1)
    {
        /* If no data or the next button is pressed play next file */
        if((!dwBytesRead) || !(GPIOA->IDR & GPIO_Pin_4))
        {
            //Silence
            memset_dma(sdcard_read_buffer,0xAA,SDCARD_READ_BUFFER_SIZE);
            /* Wait for button release */
            while(!(GPIOA->IDR & GPIO_Pin_4));
            /* Find next valid dff file */
            while(Process_Dirs());
        }

        while(DSD_GetCurrentBufferPosition()<sdcard_read_buffer_hi);
            __WFI();
        pf_read_async(sdcard_read_buffer_low,SDCARD_READ_BUFFER_SIZE/2,&dwBytesRead);
        if(dwBytesRead!=(SDCARD_READ_BUFFER_SIZE/2)) //Insert silence
            memset_dma(sdcard_read_buffer_low+dwBytesRead,0xAA, (SDCARD_READ_BUFFER_SIZE/2)-dwBytesRead);

        while(DSD_GetCurrentBufferPosition()>=sdcard_read_buffer_hi);
            __WFI();
        pf_read_async(sdcard_read_buffer_hi,SDCARD_READ_BUFFER_SIZE/2,&dwBytesRead);
        if(dwBytesRead!=(SDCARD_READ_BUFFER_SIZE/2)) //Insert silence
            memset_dma(sdcard_read_buffer_low+dwBytesRead, 0xAA, (SDCARD_READ_BUFFER_SIZE/2)-dwBytesRead);
    }

    return(0);
}


void HardFault_Handler()
{
    uint32_t *SP;
    //uint32_t FaultAddr;

    asm("mov %0,SP\n" : "=r" (SP));

    //FaultAddr=*(SP+6);
    asm("BKPT");
}

void SysTick_Handler()
{
    asm("bkpt");

}

