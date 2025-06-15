/**
 * @file    upper_comm.h
 * @author  lupinjia(jason_1120202397@163.com)
 * @brief   contains functions used to communicate with upper board(RDKX5) through SPI.
 * @version 1.0
 * @date    2024-12-08
 * 
 * @copyright HYGroup@BIT 2024
 * 
 */
#ifndef __UPPER_COMM_H__
#define __UPPER_COMM_H__

#include "dvc_motor.h"

#pragma pack(push, 1) // cancel byte alignment

// Message transmitted to upper board
// 32 bytes
typedef struct 
{
  uint8_t header1;
  uint8_t header2;
  MotorFeedbacksLogical motor_fbs_logical; // 24 bytes
  uint32_t crc32;
  uint8_t tail1;
  uint8_t tail2;
}Lower2UpperMsg;

#define UART_TX_BUFF_SIZE 32

// Message received from upper board
// 24 bytes
typedef struct
{
  uint8_t header1;
  uint8_t header2;
  uint16_t pos_des[MOTOR_NUM_TOTAL];
  uint16_t kp;
  uint16_t kd;
  uint32_t crc32;
  uint8_t tail1;
  uint8_t tail2;
}Upper2LowerMsg;

#define UART_RX_BUFF_SIZE 24

void upper_comm_init(void);

void send_low_state(void);

void parse_rx_msg(void);

extern uint8_t rx_buff[UART_RX_BUFF_SIZE]; // buffer to receive data from upper board

#pragma pack(pop) // restore byte alignment

#endif //__UPPER_COMM_H__