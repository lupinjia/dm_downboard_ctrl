#include <string.h>
#include "upper_comm.h"
#include "tim.h"

const static MotorFeedbacksLogical* motor_fbs_logical_ptr = NULL;
const static RC_DataProcessed*      rc_data_processed_ptr = NULL;
static int32_t* motor_current_des_ptr = NULL;
Lower2UpperMsg lower2upper_msg;
Upper2LowerMsg upper2lower_msg;

void upper_comm_init(void)
{
//  motor_fbs_logical_ptr = motor_get_feedbacks_logical();
//  motor_current_des_ptr = motor_get_current_des();
  rc_data_processed_ptr = sbus_get_rc_data_processed();
  memset(&lower2upper_msg, 0, sizeof(Lower2UpperMsg));
  memset(&upper2lower_msg, 0, sizeof(Upper2LowerMsg));
}

/**
 * @brief calculate crc32 of data
 * @param data pointer to data
 * @note  refer to https://stackoverflow.com/questions/21001659/crc32-algorithm-implementation-in-c-without-a-look-up-table-and-with-a-public-li
 * @return crc32 value
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

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
	// process rx data
   memcpy(motor_current_des_ptr, upper2lower_msg.current_des, sizeof(int32_t)*MOTOR_NUM_PER_LEG*2);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) // falling edge
{
	update_lower2upper_msg();
	HAL_SPI_TransmitReceive_DMA(&hspi3, (uint8_t*)&lower2upper_msg, (uint8_t*)&upper2lower_msg, sizeof(Lower2UpperMsg));
}