#ifndef __UPPER_COMM_H__
#define __UPPER_COMM_H__

#include "motor.h"
#include "spi.h"

// Message transmitted to upper board
// 56 bytes
typedef struct 
{
  uint8_t header1;
  uint8_t header2;
  MotorFeedbacksLogical motor_fbs_logical;
  uint32_t crc32;
  uint8_t tail1;
  uint8_t tail2;

}Lower2UpperMsg;

// Message received from upper board
// 56 bytes
typedef struct
{
  uint8_t header1;
  uint8_t header2;
  uint8_t data[48];
  uint32_t crc32;
  uint8_t tail1;
  uint8_t tail2;
}Upper2LowerMsg;


void upper_comm_init(void);

void update_lower2upper_msg(void);

void upper_comm_transmit_receive(void);


#endif //__UPPER_COMM_H__