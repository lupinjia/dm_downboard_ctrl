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
#include "usbd_cdc_if.h"
#include "math_ops.h"

const static MotorFeedbacksLogical* motor_fbs_logical_ptr = NULL;
static float* motor_pos_des_ptr = NULL;
static float* motor_kp_ptr = NULL;
static float* motor_kd_ptr = NULL;
Lower2UpperMsg lower2upper_msg; // message sent to upper board
Upper2LowerMsg upper2lower_msg; // message received from upper board
uint8_t rx_buff[UART_RX_BUFF_SIZE];
uint8_t low_cmd_rev_flag = 0; // flag to indicate whether low command is received or not
uint16_t low_cmd_rev_cnt = 0; // count of low command received
uint8_t* low_cmd_real_time_flag_ptr = NULL; // pointer to low command real time flag

void upper_comm_init(void)
{
  motor_fbs_logical_ptr = motor_get_feedbacks_logical();
  motor_pos_des_ptr = motor_get_pos_des();
  motor_kp_ptr = motor_get_kp();
  motor_kd_ptr = motor_get_kd();
  low_cmd_real_time_flag_ptr = motor_get_low_cmd_real_time_flag();
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
   CDC_Transmit_HS((uint8_t*)&lower2upper_msg, sizeof(lower2upper_msg));
}

void parse_rx_msg()
{
   if (rx_buff[0] == 0xAA && rx_buff[1] == 0xBB && rx_buff[UART_RX_BUFF_SIZE-2] == 0xEE && rx_buff[UART_RX_BUFF_SIZE-1] == 0xFF)
   {
      memcpy(&upper2lower_msg, rx_buff, UART_RX_BUFF_SIZE);
      uint32_t crc_verfication = calc_crc32((uint8_t*)upper2lower_msg.pos_des, sizeof(uint16_t)*MOTOR_NUM_TOTAL+4);
      if (crc_verfication == upper2lower_msg.crc32) // receive successfully
      {
         for (int i = 0; i < MOTOR_NUM_TOTAL; i++) // update motor torque des
         {
            motor_pos_des_ptr[i] = uint_to_float(upper2lower_msg.pos_des[i],POS_MIN,POS_MAX,16);
         }
         *motor_kp_ptr = uint_to_float(upper2lower_msg.kp, KP_MIN, KP_MAX, 16);
         *motor_kd_ptr = uint_to_float(upper2lower_msg.kd, KD_MIN, KD_MAX, 16);
         // low cmd real-time evaluation
         if(low_cmd_rev_flag == 0) // start timer count
         {
            low_cmd_rev_flag = 1;
            __HAL_TIM_SET_COUNTER(&htim13, 0);
            __HAL_TIM_ENABLE(&htim13);
         }
         else if(low_cmd_rev_flag == 1) // stop timer count
         {
            low_cmd_rev_flag = 0;
            low_cmd_rev_cnt = __HAL_TIM_GET_COUNTER(&htim13);
            if(low_cmd_rev_cnt > 20000) // 20ms timeout
            {
               *low_cmd_real_time_flag_ptr = 0; // low command not received in real time
            }
            else
            {
               *low_cmd_real_time_flag_ptr = 1; // low command received in real time
            }
            __HAL_TIM_DISABLE(&htim13);
         }
      }
      else
      {
         memset(&upper2lower_msg, 0, UART_RX_BUFF_SIZE);
      }
   }
   else
   {
      memset(&upper2lower_msg, 0, UART_RX_BUFF_SIZE);
   }
}
