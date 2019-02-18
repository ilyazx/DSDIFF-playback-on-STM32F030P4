/*-----------------------------------------------------------------------*/
/* Low level disk I/O module skeleton for Petit FatFs (C)ChaN, 2014      */
/*-----------------------------------------------------------------------*/

#include "diskio.h"
#include "sdcard.h"
#include "stm32f0xx_conf.h"
#include "memcpy_dma.h"
#include <string.h>

uint8_t r_sector[512];
uint32_t current_r_sector=-1;

uint8_t r_fat_sector[512];
uint32_t current_r_fat_sector=-1;

/*-----------------------------------------------------------------------*/
/* Initialize Disk Drive                                                 */
/*-----------------------------------------------------------------------*/

DSTATUS disk_initialize (void)
{
    DSTATUS stat=0;

    while(SD_Init());
    // Put your code here
    return stat;
}



/*-----------------------------------------------------------------------*/
/* Read Partial Sector                                                   */
/*-----------------------------------------------------------------------*/

DRESULT disk_readp (
    BYTE* buff,		/* Pointer to the destination object */
    DWORD sector,	/* Sector number (LBA) */
    UINT offset,	/* Offset in the sector */
    UINT count		/* Byte count (bit15:destination) */
)
{
    // DRESULT res=0;
    // Put your code here
    if(sector!=current_r_fat_sector)SD_ReadSectorSync(sector,r_fat_sector,0);
    current_r_fat_sector=sector;
    memcpy(buff,&r_fat_sector[offset],count);
    return 0;
}

DRESULT disk_readp_data_async (
    BYTE* buff,		/* Pointer to the destination object */
    DWORD sector,	/* Sector number (LBA) */
    UINT offset,	/* Offset in the sector */
    UINT countt		/* Byte count (bit15:destination) */
)
{
    //DRESULT res=0;
    // Put your code here
    if(sector!=current_r_sector)
    if(SD_ReadSectorSync(sector,r_sector,0))
            return(1);
    current_r_sector=sector;
    memcpy_dma(buff,&r_sector[offset],countt);
    return 0;
}

/*-----------------------------------------------------------------------*/
/* Write Partial Sector                                                  */
/*-----------------------------------------------------------------------*/



DRESULT disk_writep (
    BYTE* buff,		// Pointer to the data to be written, NULL:Initiate/Finalize write operation
    DWORD sc		// Sector number (LBA) or Number of bytes to send
)
{
    DRESULT res=FR_DISK_ERR;


    if (!buff)
    {
        if (sc)
        {

            // Initiate write process

        }
        else
        {

            // Finalize write process

        }
    }
    else
    {

        // Send data to the disk

    }

    return res;
}

