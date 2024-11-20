#ifndef __PARAM_H__
#define __PARAM_H__

#define MOTOR_NUM             3
#define MOTOR_REDUCTION_RATIO 18.0f
#define M_PI	              3.14159265358979323846f
#define MAX_CURRENT           20000.0f //[mA]
// #define FP_KP                 1000.0f
// #define FP_KD                 90.0f
#define FP_DAMP_KD            10.0f
#define UART5_RX_BUFFER_SIZE  25   // [byte], uart5 for sbus receive
#define TORQUE_CONSTANT       1.5f // [Nm/A], to convert torque to current

#endif