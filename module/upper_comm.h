#ifndef __UPPER_COMM_H__
#define __UPPER_COMM_H__

#include "motor.h"
#include "sbus.h"
#include "spi.h"

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
  int32_t current_des[MOTOR_NUM_PER_LEG*2];
  uint8_t redundancy[39];
  uint32_t crc32;
  uint8_t tail1;
  uint8_t tail2;
}Upper2LowerMsg;


void upper_comm_init(void);

void update_lower2upper_msg(void);

#pragma pack(pop) // restore byte alignment

#endif //__UPPER_COMM_H__