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

#include "sbus.h"
#include "spi.h"
#include "dvc_motor.h"

#pragma pack(push, 1) // cancel byte alignment

// Message transmitted to upper board
// 71 bytes
typedef struct 
{
  uint8_t header1;
  uint8_t header2;
  MotorFeedbacksLogical motor_fbs_logical;
  RC_DataProcessed      rc_data_processed;
  uint32_t crc32;
  uint8_t tail1;
  uint8_t tail2;
}Lower2UpperMsg;

// Message received from upper board
// 71 bytes
typedef struct
{
  uint8_t header1;
  uint8_t header2;
  int32_t torque_des[MOTOR_NUM_TOTAL];
  uint8_t redundancy[39];
  uint32_t crc32;
  uint8_t tail1;
  uint8_t tail2;
}Upper2LowerMsg;


void upper_comm_init(void);

void update_lower2upper_msg(void);

void start_spi_comm(void);

#pragma pack(pop) // restore byte alignment

#endif //__UPPER_COMM_H__