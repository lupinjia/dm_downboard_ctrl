/**
 * @file    upper_comm.c
 * @author  lupinjia(jason_1120202397@163.com)
 * @brief   contains functions used to communicate with upper board(RDKX5) through SPI.
 * @version 1.0
 * @date    2024-12-08
 * 
 * @copyright HYGroup@BIT 2024
 * 
 */
#include <string.h>
#include "upper_comm.h"
#include "tim.h"

const static MotorFeedbacksLogical* motor_fbs_logical_ptr = NULL;
const static RC_DataProcessed*      rc_data_processed_ptr = NULL;
static float* motor_torque_des_ptr = NULL;
Lower2UpperMsg lower2upper_msg; // message sent to upper board
Upper2LowerMsg upper2lower_msg; // message received from upper board

void upper_comm_init(void)
{
  motor_fbs_logical_ptr = motor_get_feedbacks_logical();
  motor_torque_des_ptr = motor_get_torque_des();
  rc_data_processed_ptr = sbus_get_rc_data_processed();
  memset(&lower2upper_msg, 0, sizeof(Lower2UpperMsg));
  memset(&upper2lower_msg, 0, sizeof(Upper2LowerMsg));
}

/**
 * @brief  calculate crc32 of data
 * @param  data pointer to data
 * @param  len  length of data [Byte]
 * @note   refer to https://stackoverflow.com/questions/21001659/crc32-algorithm-implementation-in-c-without-a-look-up-table-and-with-a-public-li
 * @return uint32_t crc32 value
 */
static uint32_t calc_crc32(const uint8_t *data, uint8_t len) 
{
   int i, j;
   unsigned int byte, crc, mask;

   i = 0;
   crc = 0xFFFFFFFF;
   while (i < len) {
      byte = data[i];            // Get next byte.
      crc = crc ^ byte;
      for (j = 7; j >= 0; j--) {    // Do eight times.
         mask = -(crc & 1);
         crc = (crc >> 1) ^ (0xEDB88320 & mask);
      }
      i = i + 1;
   }
   return ~crc;
}

/**
 * @brief update lower2upper_msg with current motor feedbacks and rc data processed.
 * @note  this function should be called before sending lower2upper_msg to upper board.
 * 
 */
void update_lower2upper_msg(void)
{
  memcpy(&lower2upper_msg.motor_fbs_logical, motor_fbs_logical_ptr, sizeof(MotorFeedbacksLogical));
  memcpy(&lower2upper_msg.rc_data_processed, rc_data_processed_ptr, sizeof(RC_DataProcessed));
  lower2upper_msg.crc32 = calc_crc32((uint8_t*)&lower2upper_msg.motor_fbs_logical, sizeof(MotorFeedbacksLogical)+sizeof(RC_DataProcessed));
  lower2upper_msg.header1 = 0xAA;
  lower2upper_msg.header2 = 0xBB;
  lower2upper_msg.tail1 = 0xEE;
  lower2upper_msg.tail2 = 0xFF;
}

/**
 * @brief Callback function for SPI transmit and receive complete.
 * @note  This function is called when SPI transmit and receive complete, to process received data.
 * @param hspi handle of spi
 */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
	// process rx data
   for(int i = 0; i < MOTOR_NUM_TOTAL; i++)
   {
      motor_torque_des_ptr[i] = (float)(upper2lower_msg.torque_des[i] / 100000.0);
   }
	 
	 update_lower2upper_msg();
	 HAL_SPI_TransmitReceive_DMA(&hspi3, (uint8_t*)&lower2upper_msg, (uint8_t*)&upper2lower_msg, sizeof(Lower2UpperMsg));
}

void start_spi_comm()
{
	update_lower2upper_msg();
	HAL_SPI_TransmitReceive_DMA(&hspi3, (uint8_t*)&lower2upper_msg, (uint8_t*)&upper2lower_msg, sizeof(Lower2UpperMsg));
}