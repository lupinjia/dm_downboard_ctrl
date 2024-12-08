#ifndef __PARAM_H__
#define __PARAM_H__

#include "stm32h7xx_hal.h"

// Macros for Motor Control
#define MOTOR_NUM_PER_LEG     3
#define MOTOR_NUM_TOTAL       6

// Macros for SBUS
#define UART5_RX_BUFFER_SIZE  25   // [byte], uart5 for sbus receive

// Macros for SPI
#define CS_PORT GPIOE        // use PE14 as software CS for SPI
#define CS_PIN  GPIO_PIN_14

#endif