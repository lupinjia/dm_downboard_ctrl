#ifndef __PARAM_H__
#define __PARAM_H__

#include "stm32h7xx_hal.h"

// Macros for Motor Control
#define MOTOR_NUM_PER_LEG     3
#define MOTOR_NUM_TOTAL       6
#define MOTOR_REDUCTION_RATIO 18.0f
#define M_PI	              3.14159265358979323846f
#define MAX_CURRENT           20000 //[mA]
#define MAX_CURRENT_A         
#define MAX_TORQUE_NM         30.0f // [Nm]
// #define FP_KP                 1000.0f
// #define FP_KD                 90.0f
#define FP_DAMP_KD            10.0f
#define TORQUE_CONSTANT       1.5f // [Nm/A], to convert torque to current

// Macros for SBUS
#define UART5_RX_BUFFER_SIZE  25   // [byte], uart5 for sbus receive

#define CS_PORT GPIOE
#define CS_PIN  GPIO_PIN_14

// Type of leg
typedef enum
{
    LEFT_LEG,
    RIGHT_LEG
}LEG_TYPE;

#endif