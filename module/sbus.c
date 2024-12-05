#include "sbus.h"
#include "param.h"
#include "string.h"
#include "usart.h"

static uint8_t uart5_rx_buf[25] = {0};
static RC_ChannelData rc_data;

static uint8_t convert_switch_value(uint16_t raw_value)
{
    if(raw_value == RAW_DATA_LOWER_BOUND) return 0;
    else if(raw_value == RAW_DATA_MEDIAN_VALUE) return 1;
    else if(raw_value == RAW_DATA_UPPER_BOUND) return 2;
    else return 1; // default value
}

static void sbus_decode(uint8_t* rx_buf, RC_ChannelData* channel_data)
{
    // decode raw channel data from SBUS buffer
    channel_data->raw_data[0] = (rx_buf[1] | (rx_buf[2] << 8)) & 0x07FF;
    channel_data->raw_data[1] = (rx_buf[2] >> 3 | (rx_buf[3] << 5)) & 0x07FF;
    channel_data->raw_data[2] = (rx_buf[3] >> 6 | (rx_buf[4] << 2)  | rx_buf[5] << 10) & 0x07FF;
    channel_data->raw_data[3] = (rx_buf[5] >> 1 | (rx_buf[6] << 7)) & 0x07FF;
    channel_data->raw_data[4] = (rx_buf[6] >> 4 | (rx_buf[7] << 4)) & 0x07FF;
    channel_data->raw_data[5] = (rx_buf[7] >> 7 | (rx_buf[8] << 1)  | rx_buf[9] <<  9) & 0x07FF;
    channel_data->raw_data[6] = (rx_buf[9] >> 2 | (rx_buf[10] << 6)) & 0x07FF;
    channel_data->raw_data[7] = (rx_buf[10] >> 5 | (rx_buf[11] << 3)) & 0x07FF;
    channel_data->raw_data[8] = (rx_buf[12] | (rx_buf[13] << 8 )) & 0x07FF;
    channel_data->raw_data[9] = (rx_buf[13] >> 3 | (rx_buf[14] << 5 )) & 0x07FF;
    channel_data->raw_data[10] = (rx_buf[14] >> 6 | (rx_buf[15] << 2 )  | rx_buf[16] << 10 ) & 0x07FF;
	channel_data->raw_data[11] = (rx_buf[16] >> 1 | (rx_buf[17] << 7 )) & 0x07FF;
	channel_data->raw_data[12] = (rx_buf[17] >> 4 | (rx_buf[18] << 4 )) & 0x07FF;
	channel_data->raw_data[13] = (rx_buf[18] >> 7 | (rx_buf[19] << 1 )  | rx_buf[20] <<  9 ) & 0x07FF;
	channel_data->raw_data[14] = (rx_buf[20] >> 2 | (rx_buf[21] << 6 )) & 0x07FF;
	channel_data->raw_data[15] = (rx_buf[21] >> 5 | (rx_buf[22] << 3 )) & 0x07FF;

    // process raw channel data
    // resultant left/right joystick value range: 0~1440
    // resultant switch value range: 0, 1, 2
    // resultant A/B/C/D button value range: 0, 1
    channel_data->processed_data.lx = channel_data->raw_data[3] - RAW_DATA_LOWER_BOUND;
    channel_data->processed_data.ly = channel_data->raw_data[2] - RAW_DATA_LOWER_BOUND;
    channel_data->processed_data.rx = channel_data->raw_data[0] - RAW_DATA_LOWER_BOUND;
    channel_data->processed_data.ry = channel_data->raw_data[1] - RAW_DATA_LOWER_BOUND;
    channel_data->processed_data.SA = convert_switch_value(channel_data->raw_data[4]);
    channel_data->processed_data.SB = convert_switch_value(channel_data->raw_data[5]);
    channel_data->processed_data.SC = convert_switch_value(channel_data->raw_data[6]);
    channel_data->processed_data.A = (channel_data->raw_data[7] == RAW_DATA_UPPER_BOUND) ? 1 : 0;
    channel_data->processed_data.B = (channel_data->raw_data[8] == RAW_DATA_UPPER_BOUND) ? 1 : 0;
    channel_data->processed_data.C = (channel_data->raw_data[9] == RAW_DATA_UPPER_BOUND) ? 1 : 0;
    channel_data->processed_data.D = (channel_data->raw_data[10] == RAW_DATA_UPPER_BOUND) ? 1 : 0;
}

uint8_t* sbus_get_uart5_rx_buf()
{
    return uart5_rx_buf;
}

const RC_DataProcessed* sbus_get_rc_data_processed()
{
    return &rc_data.processed_data;
}

void sbus_init()
{
   memset(uart5_rx_buf, 0, UART5_RX_BUFFER_SIZE);
   memset(&rc_data, 0, sizeof(RC_ChannelData));
}

void start_sbus_reception()
{
    HAL_UARTEx_ReceiveToIdle_DMA(&huart5, uart5_rx_buf, UART5_RX_BUFFER_SIZE*2);
}

/**
 * @brief UART5 IDLE event callback function.
 * @details receive sbus data from remote controller, decode it and update rc_data.
 * 
 * @param huart handle of UART
 * @param Size  size of received data
 */

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) // IDLE event callback
{
    if(huart->Instance == UART5)
    {
        if (Size <= UART5_RX_BUFFER_SIZE && uart5_rx_buf[0] == 0x0F) // success
	    {
		    sbus_decode(uart5_rx_buf, &rc_data);
	    }
	    else // failed
	    {
		    memset(uart5_rx_buf, 0, UART5_RX_BUFFER_SIZE);
	    }			
    }
}