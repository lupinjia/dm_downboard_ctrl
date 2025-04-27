/**
 * @file    dvc_motor.h
 * @author  lupinjia(jason_1120202397@163.com)
 * @brief   contains driver functions for EC-A8112-P1-18 motor from encos(因克斯电机), utility functions and some interrupt callback functions.
 * @version 1.0
 * @date    2024-12-08
 * 
 * @copyright HYGroup@BIT 2024
 * 
 */
#ifndef DVC_MOTOR_H
#define DVC_MOTOR_H

#define POS_MIN -12.5f
#define POS_MAX  12.5f
#define SPD_MIN -18.0f
#define SPD_MAX  18.0f
#define T_MIN -90.0f // max torque of EC-A8112-P1-18 is 90Nm
#define T_MAX  90.0f // max torque of EC-A8112-P1-18 is 90Nm
#define I_MIN -30.0f
#define I_MAX  30.0f

#define RPM2RAD 0.10471975511965977
#define RAD2RPM 9.549296585513721
#define DEG2RAD 0.017453292519943295
#define RAD2DEG 57.29577951308232

#include "stm32h7xx.h"
#include "can_bsp.h"
#include "param.h"

typedef struct 
{
	uint16_t angle_actual_int;
	uint16_t angle_desired_int;
	int16_t speed_actual_int;
	int16_t speed_desired_int;
	int16_t current_actual_int;
	int16_t current_desired_int;
	float 	speed_actual_rad;
	float 	speed_desired_rad;
	float 	angle_actual_rad;	
	float   angle_desired_rad;
	uint16_t	motor_id;
	uint8_t 	temperature;
	uint8_t		error;
	float     angle_actual_float;
	float 		speed_actual_float;
	float 		current_actual_float;
	float     angle_desired_float;
	float 		speed_desired_float;
	float 		current_desired_float;
	float			power;
	uint16_t	acceleration;
	uint16_t	linkage_KP;
	uint16_t 	speed_KI;
	uint16_t	feedback_KP;
	uint16_t	feedback_KD;
}Motor_Msg;

// motor feedback data processed into logical values with int32_t type
// transimitted to upper board
// 4 bytes
typedef struct
{
    int16_t angle;
    int16_t speed;
}MotorFeedbackLogical;

// Integration of MotorFeedbackLogical
// 24 bytes
typedef struct
{
    MotorFeedbackLogical left[MOTOR_NUM_PER_LEG];
    MotorFeedbackLogical right[MOTOR_NUM_PER_LEG];
}MotorFeedbacksLogical;

// motor feedback data processed into logical values with float type, for lower board demo
// this will be implemented on upper board
typedef struct
{
    float angle;   // [rad], angle of motor shaft
    float speed;   // [rad/s], speed of motor shaft
    float current; // [A], current of motor
}MotorFeedbackLogicalFloat;

typedef struct
{
    MotorFeedbackLogicalFloat left[MOTOR_NUM_PER_LEG];
    MotorFeedbackLogicalFloat right[MOTOR_NUM_PER_LEG];
}MotorFeedbacksLogicalFloat;

void SetMotorPosition(FDCAN_TxFrame_TypeDef *TxFrame, uint16_t motor_id, float position_des, float velocity_des);
void SetMotorVelocity(FDCAN_TxFrame_TypeDef *TxFrame, uint16_t motor_id, float speed_des);
void SetMotorTorque(FDCAN_TxFrame_TypeDef *TxFrame, uint16_t motor_id, float torque_des);
void MotorIDReading(FDCAN_TxFrame_TypeDef *TxFrame);
void MotorIDSetting(FDCAN_TxFrame_TypeDef *TxFrame, uint16_t motor_id, uint16_t motor_id_new);
void MotorCommModeReading(FDCAN_TxFrame_TypeDef *TxFrame, uint16_t motor_id);

const MotorFeedbacksLogical* motor_get_feedbacks_logical();
float* motor_get_torque_des();
uint8_t* motor_get_low_cmd_real_time_flag();
void send_motor_cmd();

#endif