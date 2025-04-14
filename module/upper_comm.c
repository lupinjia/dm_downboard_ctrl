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
#include "usart.h"

const static MotorFeedbacksLogical* motor_fbs_logical_ptr = NULL;
static float* motor_torque_des_ptr = NULL;
Lower2UpperMsg lower2upper_msg; // message sent to upper board
Upper2LowerMsg upper2lower_msg; // message received from upper board
uint8_t rx_buff[UART_BUFF_SIZE];

void upper_comm_init(void)
{
  motor_fbs_logical_ptr = motor_get_feedbacks_logical();
  motor_torque_des_ptr = motor_get_torque_des();
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
static void update_lower2upper_msg(void)
{
  memcpy(&lower2upper_msg.motor_fbs_logical, motor_fbs_logical_ptr, sizeof(MotorFeedbacksLogical));
  lower2upper_msg.crc32 = calc_crc32((uint8_t*)&lower2upper_msg.motor_fbs_logical, sizeof(MotorFeedbacksLogical));
  lower2upper_msg.header1 = 0xAA;
  lower2upper_msg.header2 = 0xBB;
  lower2upper_msg.tail1 = 0xEE;
  lower2upper_msg.tail2 = 0xFF;
}

void send_low_state()
{
   update_lower2upper_msg();
   HAL_UART_Transmit_DMA(&huart7, (uint8_t*)&lower2upper_msg, sizeof(Lower2UpperMsg));
}

void start_rev_low_cmd()
{
	HAL_UARTEx_ReceiveToIdle_DMA(&huart7, rx_buff, UART_BUFF_SIZE*2);
}

static void parse_rx_msg()
{
   if (rx_buff[0] == 0xAA && rx_buff[1] == 0xBB && rx_buff[UART_BUFF_SIZE-2] == 0xEE && rx_buff[UART_BUFF_SIZE-1] == 0xFF)
   {
      memcpy(&upper2lower_msg, rx_buff, UART_BUFF_SIZE);
      uint32_t crc_verfication = calc_crc32((uint8_t*)upper2lower_msg.torque_des, sizeof(int32_t)*MOTOR_NUM_TOTAL);
      if (crc_verfication == upper2lower_msg.crc32)
      {
         for (int i = 0; i < MOTOR_NUM_TOTAL; i++)
         {
            motor_torque_des_ptr[i] = (float)(upper2lower_msg.torque_des[i] / 100000.0);
         }
      }
      else
      {
         memset(&upper2lower_msg, 0, UART_BUFF_SIZE);
      }
   }
   else
   {
      memset(&upper2lower_msg, 0, UART_BUFF_SIZE);
   }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef * huart, uint16_t Size)
{

	if(huart->Instance == UART7)
	{
		if (Size <= UART_BUFF_SIZE)
		{
			HAL_UARTEx_ReceiveToIdle_DMA(&huart7, rx_buff, UART_BUFF_SIZE*2);
			parse_rx_msg();
		}
		else
		{
			HAL_UARTEx_ReceiveToIdle_DMA(&huart7, rx_buff, UART_BUFF_SIZE*2);
			memset(&upper2lower_msg, 0, UART_BUFF_SIZE);
		}
	}
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef * huart)
{
	if(huart->Instance == UART7)
	{
		HAL_UARTEx_ReceiveToIdle_DMA(&huart7, rx_buff, UART_BUFF_SIZE*2);
		memset(rx_buff, 0, UART_BUFF_SIZE);	
	}
}
