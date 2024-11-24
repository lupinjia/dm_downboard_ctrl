#ifndef __SBUS_H__
#define __SBUS_H__
/********** Includes **********/
#include <stdint.h>
#include "stm32h7xx_hal.h"

#pragma pack(push, 1) // cancel byte alignment

/*********** Defines **********/
#define RAW_DATA_LOWER_BOUND    272u // u for unsigned int
#define RAW_DATA_UPPER_BOUND    1712u
#define RAW_DATA_MEDIAN_VALUE   992u
#define LEVER_DATA_MEDIAN_VALUE (RAW_DATA_MEDIAN_VALUE - RAW_DATA_LOWER_BOUND)

/*********** Typedefs **********/

// data that is literally used, 15 bytes
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
}RC_DataProcessed; 

// Channel data of remote controller, 47 bytes
typedef struct 
{
    uint16_t raw_data[16];
    RC_DataProcessed processed_data;
}RC_ChannelData; 

/********** Function Prototypes **********/

/**
 * @brief get uart5 rx buffer pointer
 * 
 * @return uint8_t* pointer to uart5 rx buffer
 */
uint8_t* sbus_get_uart5_rx_buf();
/**
 * @brief Get the rc channel data pointer
 * 
 * @attention This function is only used for debugging purposes.
 * @return RC_ChannelData* pointer to rc channel data
 */
RC_ChannelData* sbus_get_rc_channel_data();

double* sbus_get_motor_angle_increment();

uint8_t* sbus_get_selected_motor_id();
/**
 * @brief Initialize sbus module
 * 
 * @details give the variables initial values
 * @return None
 */
void sbus_init();

#pragma pack(pop) // restore byte alignment

#endif // __SBUS_H__