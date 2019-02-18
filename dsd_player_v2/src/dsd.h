#ifndef __DSD_H
#define __DSD_H

typedef struct
{
    uint32_t sample_rate;
    uint32_t dsd_data_offset;
    uint32_t data_size;

} dsf_info_t;

uint8_t *dsd_write_low(uint8_t * buf);
uint8_t *dsd_write_hi(uint8_t * buf);
void DSD_Init();
uint8_t *DSD_GetCurrentBufferPosition();
uint8_t DSD_Reconfigure(dsf_info_t *di);

#endif
