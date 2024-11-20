#ifndef __SBUS_H__
#define __SBUS_H__
/********** Includes **********/
#include <stdint.h>

#pragma pack(push, 1) // cancel byte alignment

/*********** Defines **********/
#define RAW_DATA_LOWER_BOUND  272u // u for unsigned int
#define RAW_DATA_UPPER_BOUND  1712u
#define RAW_DATA_MEDIAN_VALUE 992u

/*********** Typedefs **********/

typedef struct
{
    uint16_t lx;  // left stick x, corresponds to J4
    uint16_t ly;  // left stick y, corresponds to J3
    uint16_t rx;  // right stick x, corresponds to J1
    uint16_t ry;  // right stick y, corresponds to J2
    uint8_t  SA;  // left up switch
    uint8_t  SB;  // right up switch
    uint8_t  SC;  // center switch
    uint8_t  A;   // button A
    uint8_t  B;   // button B
    uint8_t  C;   // button C
    uint8_t  D;   // button D
}RC_DataProcessed; // data that is literally used

typedef struct 
{
    uint16_t raw_data[16];
    RC_DataProcessed processed_data;
}RC_ChannelData; // Channel data of remote controller

/********** Function Prototypes **********/
void sbus_decode(uint8_t* rx_buf, RC_ChannelData* channel_data);
uint8_t* get_uart5_rx_buf();

#pragma pack(pop) // restore byte alignment

#endif // __SBUS_H__