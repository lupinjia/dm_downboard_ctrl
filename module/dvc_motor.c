/**
 * @file    dvc_motor.c
 * @author  lupinjia(jason_1120202397@163.com)
 * @brief   contains driver functions for EC-A8112-P1-18 motor from encos(因克斯电机), utility functions and some interrupt callback functions.
 * @version 1.0
 * @date    2024-12-08
 * 
 * @copyright HYGroup@BIT 2024
 * 
 */
#include "dvc_motor.h"
#include "math_ops.h"

// left leg and right leg are 2 buses, and the motor id on same position is the same
const static uint8_t motor_id[MOTOR_NUM_PER_LEG] = {0x01, 0x02, 0x03};
static uint8_t       can_send_flag = 1; // choose which id to send cmd

// to save initial pos of motors
static uint8_t can_first_rev_flag[MOTOR_NUM_TOTAL] = {1, 1, 1, 1, 1, 1};
static float   motor_physical_initial_angle[MOTOR_NUM_TOTAL] = {0, 0, 0, 0, 0, 0}; // record physical initial angle of motors, for converting physical pos to logical pos
const static float motor_logical_initial_angle[MOTOR_NUM_TOTAL] = {0.33, 1.3, -2.00, // [rad]
                                                                     -0.33, 1.3, -2.00};
const static int8_t motor_direction_mask[MOTOR_NUM_TOTAL] = {1, 1, 1, 1, -1, -1}; // direction mask, to convert physical pos to logical pos

float motor_torque_des[MOTOR_NUM_TOTAL] = {0, 0, 0, 0, 0, 0};

MotorFeedbacksLogical motor_fbs_logical;            // int type, data sent to upboard
MotorFeedbacksLogicalFloat motor_fbs_logical_float; // float type, for processing and debug

/**********************Motor Driver Functions **************************/

union TypeConvert
{
	float to_float;
	int to_int;
	unsigned int to_uint;
	uint8_t buf[4];
} type_convert;

/**
 * @brief Set the Motor Position
 * 
 * @param motor_id The ID of the motor
 * @param position_des The desired position of the motor, unit: rad.
 * @param velocity_des The desired velocity of the motor, unit: rad/s.
 */
void SetMotorPosition(FDCAN_TxFrame_TypeDef *TxFrame, uint16_t motor_id, float position_des, float velocity_des)
{
  uint16_t current_threshold = I_MAX * 10;
  uint8_t ack_status = 1;

  TxFrame->Header.Identifier = motor_id;
  TxFrame->Header.DataLength = FDCAN_DLC_BYTES_8;
  type_convert.to_float = position_des * RAD2DEG;
  TxFrame->Data[0] = 0x20 | (type_convert.buf[3] >> 3);
  TxFrame->Data[1] = (type_convert.buf[3] << 5) | (type_convert.buf[2] >> 3);
  TxFrame->Data[2] = (type_convert.buf[2] << 5) | (type_convert.buf[1] >> 3);
  TxFrame->Data[3] = (type_convert.buf[1] << 5) | (type_convert.buf[0] >> 3);

  if (velocity_des > SPD_MAX) 
  {
    velocity_des = SPD_MAX;
  }
  else if (velocity_des < SPD_MIN)
  {
    velocity_des = SPD_MIN;
  }

  uint16_t velocity_des_uint16 = (uint16_t)(velocity_des * RAD2RPM * 10);
  TxFrame->Data[4] = (type_convert.buf[0] << 5) | (velocity_des_uint16 >> 10);
  TxFrame->Data[5] = (velocity_des_uint16 & 0x3FC) >> 2;
  TxFrame->Data[6] = ((velocity_des_uint16 & 0x03) << 6) | (current_threshold >> 6);
  TxFrame->Data[7] = ((current_threshold & 0x3F) << 2) | ack_status;

  HAL_FDCAN_AddMessageToTxFifoQ(TxFrame->hcan, &TxFrame->Header, TxFrame->Data);
}

/**
 * @brief Set the Motor Velocity
 * 
 * @param motor_id The ID of the motor
 * @param velocity_des The desired velocity of the motor, unit: rad/s
 */
void SetMotorVelocity(FDCAN_TxFrame_TypeDef *TxFrame, uint16_t motor_id, float velocity_des)
{
  uint16_t current_threshold = I_MAX * 10;
  uint8_t ack_status = 1;

  TxFrame->Header.Identifier = motor_id;
  TxFrame->Header.DataLength = FDCAN_DLC_BYTES_7;
  TxFrame->Data[0] = 0x40 | ack_status;

  if (velocity_des > SPD_MAX) 
  {
    velocity_des = SPD_MAX;
  }
  else if (velocity_des < SPD_MIN)
  {
    velocity_des = SPD_MIN;
  }

  type_convert.to_float = velocity_des * RAD2RPM;
  TxFrame->Data[1] = type_convert.buf[3];
  TxFrame->Data[2] = type_convert.buf[2];
  TxFrame->Data[3] = type_convert.buf[1];
  TxFrame->Data[4] = type_convert.buf[0];
  TxFrame->Data[5] = current_threshold>>8;
  TxFrame->Data[6] = current_threshold&0xff;

  HAL_FDCAN_AddMessageToTxFifoQ(TxFrame->hcan, &TxFrame->Header, TxFrame->Data);
}

/**
 * @brief Set the Motor Torque
 * 
 * @param motor_id The ID of the motor
 * @param torque_des The desired torque of the motor, unit: N.m
 */

void SetMotorTorque(FDCAN_TxFrame_TypeDef *TxFrame, uint16_t motor_id, float torque_des)
{
  uint8_t ctrl_status = 1;
  uint8_t ack_status = 1;

  TxFrame->Header.Identifier = motor_id;
  TxFrame->Header.DataLength = FDCAN_DLC_BYTES_3;

  if (torque_des > T_MAX) 
  {
    torque_des = T_MAX;
  }
  else if (torque_des < T_MIN)
  {
    torque_des = T_MIN;
  }
  int16_t torque_des_int16 = (int16_t)(torque_des * 100);

  TxFrame->Data[0] = 0x60 | ctrl_status << 2 | ack_status;
  TxFrame->Data[1] = torque_des_int16 >> 8;
  TxFrame->Data[2] = torque_des_int16 & 0xff;

  HAL_FDCAN_AddMessageToTxFifoQ(TxFrame->hcan, &TxFrame->Header, TxFrame->Data);
}

/* Read motor ID */
void MotorIDReading(FDCAN_TxFrame_TypeDef *TxFrame)
{
  TxFrame->Header.Identifier = 0x7FF;
  TxFrame->Header.DataLength = FDCAN_DLC_BYTES_4;
  TxFrame->Data[0] = 0xFF;
  TxFrame->Data[1] = 0xFF;
  TxFrame->Data[2] = 0x00;
  TxFrame->Data[3] = 0x82;

  HAL_FDCAN_AddMessageToTxFifoQ(TxFrame->hcan, &TxFrame->Header, TxFrame->Data);
}

/* Set motor new ID */
void MotorIDSetting(FDCAN_TxFrame_TypeDef *TxFrame, uint16_t motor_id, uint16_t motor_id_new)
{
  TxFrame->Header.Identifier = 0x7FF;
  TxFrame->Header.DataLength = FDCAN_DLC_BYTES_6;
  TxFrame->Data[0] = motor_id>>8;
  TxFrame->Data[1] = motor_id&0xff;
  TxFrame->Data[2] = 0x00;
  TxFrame->Data[3] = 0x04;
  TxFrame->Data[4] = motor_id_new>>8;
  TxFrame->Data[5] = motor_id_new&0xff;

  HAL_FDCAN_AddMessageToTxFifoQ(TxFrame->hcan, &TxFrame->Header, TxFrame->Data);
}

/* Read motor communication mode */
void MotorCommModeReading(FDCAN_TxFrame_TypeDef *TxFrame, uint16_t motor_id)
{
  TxFrame->Header.Identifier = 0x7FF;
  TxFrame->Header.DataLength = FDCAN_DLC_BYTES_4;
  TxFrame->Data[0] = motor_id>>8;
  TxFrame->Data[1] = motor_id&0xff;
  TxFrame->Data[2] = 0x00;
  TxFrame->Data[3] = 0x81;

  HAL_FDCAN_AddMessageToTxFifoQ(TxFrame->hcan, &TxFrame->Header, TxFrame->Data);
}
/**********************Motor Driver Functions **************************/

/**********************Motor Utility Functions **************************/

/**
 * @brief   get motor logical feedbacks pointer
 * @details This function is called on initilization of upper_comm to get the pointer to motor_fbs_logical, which is then packed and sent to upboard.
 * @return  pointer to motor_fbs_logical
 */
const MotorFeedbacksLogical* motor_get_feedbacks_logical()
{
  return &motor_fbs_logical;
}
/**
 * @brief   get motor desired torque pointer
 * @details This function is called on initilization of upper_comm to get the pointer to motor_torque_des, which is updated with the data received from upboard.
 * @return  pointer to motor_torque_des
 */
float* motor_get_torque_des()
{
  return motor_torque_des;
}

/**********************Motor Utility Functions **************************/
/**
 * @brief   send command to motor, callback for TIM7
 * @details the frequency of this function is 300us, which means each motor will receive its cmd every 900us.
 * @return  none
 */
static void send_cmd()
{
  // left leg
  SetMotorTorque(&FDCAN1_TxFrame, motor_id[can_send_flag-1], motor_torque_des[can_send_flag-1] * motor_direction_mask[can_send_flag-1]);
  // right leg
  SetMotorTorque(&FDCAN2_TxFrame, motor_id[can_send_flag-1], motor_torque_des[can_send_flag-1+MOTOR_NUM_PER_LEG] * motor_direction_mask[can_send_flag-1+MOTOR_NUM_PER_LEG]);
  // SetMotorTorque(&FDCAN1_TxFrame, motor_id[can_send_flag-1], 0.0);
  // SetMotorTorque(&FDCAN2_TxFrame, motor_id[can_send_flag-1], 0.0);
  switch (can_send_flag) // change id of motor to be sent cmd
  {
    case 1:
      can_send_flag = 2;
      break;
    case 2:
      can_send_flag = 3;
      break;
    case 3:
      can_send_flag = 1;
      break;
    default:
      can_send_flag = 1;
      break;
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if(htim->Instance == TIM7) // 300us period, 900us command period for each motor
  {
    send_cmd();
  }
}

/**
 * @brief FDCAN1 RX FIFO0 Callback function.
 * 
 * @param hfdcan      handle of FDCAN
 * @param RxFifo0ITs  interrupt status of RX FIFO0
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
  int angle;
  int speed;
  int current;
  HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &FDCAN12_RxFrame.Header, FDCAN12_RxFrame.Data);

  switch(FDCAN12_RxFrame.Header.Identifier) {
    case 0x001:
    case 0x002:
    case 0x003:
      // Response frame type 1
      if (FDCAN12_RxFrame.Data[0] >> 5 == 1) {
        uint8_t motor_id_t = FDCAN12_RxFrame.Header.Identifier - 1; // motor ID - 1 = index in array
        // process motor feedback data
        angle = FDCAN12_RxFrame.Data[1]<<8|FDCAN12_RxFrame.Data[2];
        speed = FDCAN12_RxFrame.Data[3]<<4|(FDCAN12_RxFrame.Data[4]&0xF0)>>4;
        current = (FDCAN12_RxFrame.Data[4]&0x0F)<<8|FDCAN12_RxFrame.Data[5];
        // float data
        motor_fbs_logical_float.left[motor_id_t].angle = uint_to_float(angle,POS_MIN,POS_MAX,16);
        motor_fbs_logical_float.left[motor_id_t].speed = uint_to_float(speed,SPD_MIN,SPD_MAX,12);
        motor_fbs_logical_float.left[motor_id_t].current = uint_to_float(current,I_MIN,I_MAX,12);

        if(can_first_rev_flag[motor_id_t] == 1) // first receive, save initial pos
        {
          motor_physical_initial_angle[motor_id_t] = motor_fbs_logical_float.left[motor_id_t].angle;
          can_first_rev_flag[motor_id_t] = 0;
        }
        else // post process motor feedback data
        {
          // logical feedback, logicalization
          // motor_fbs_logical.left[motor_id_t].angle = (motor_fbs_raw.left[motor_id_t].angle
          //                                             - motor_physical_initial_angle_int[motor_id_t]) * motor_direction_mask[motor_id_t]
          //                                             + motor_logical_initial_angle_int[motor_id_t];
          // motor_fbs_logical.left[motor_id_t].speed = motor_fbs_raw.left[motor_id_t].speed * motor_direction_mask[motor_id_t]; // convert to logical speed
          // ! uint_to_float after logicalization will cause error

          // logicalize float value
          motor_fbs_logical_float.left[motor_id_t].angle = (motor_fbs_logical_float.left[motor_id_t].angle 
                                                            - motor_physical_initial_angle[motor_id_t]) * motor_direction_mask[motor_id_t]
                                                            + motor_logical_initial_angle[motor_id_t];
          motor_fbs_logical_float.left[motor_id_t].speed *= motor_direction_mask[motor_id_t]; // convert to logical speed

          // get integer value of logical feedback
          motor_fbs_logical.left[motor_id_t].angle = float_to_uint(motor_fbs_logical_float.left[motor_id_t].angle,POS_MIN,POS_MAX,16);
          motor_fbs_logical.left[motor_id_t].speed = float_to_uint(motor_fbs_logical_float.left[motor_id_t].speed,SPD_MIN,SPD_MAX,12);
        }
      }
      break;
  }
}

/**
 * @brief FDCAN2 RX FIFO1 Callback function.
 * 
 * @param hfdcan      handle of FDCAN
 * @param RxFifo0ITs  interrupt status of RX FIFO1
 */
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
  int angle;
  int speed;
  int current;
  HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, &FDCAN12_RxFrame.Header, FDCAN12_RxFrame.Data);

  switch(FDCAN12_RxFrame.Header.Identifier) { // judge the motor ID
    case 0x001:
    case 0x002:
    case 0x003:
      // Response frame type 1
      if (FDCAN12_RxFrame.Data[0] >> 5 == 1) {
        uint8_t motor_id_t = FDCAN12_RxFrame.Header.Identifier - 1; // motor ID - 1 = index in array
        // process motor feedback data
        angle = FDCAN12_RxFrame.Data[1]<<8|FDCAN12_RxFrame.Data[2];
        speed = FDCAN12_RxFrame.Data[3]<<4|(FDCAN12_RxFrame.Data[4]&0xF0)>>4;
        current = (FDCAN12_RxFrame.Data[4]&0x0F)<<8|FDCAN12_RxFrame.Data[5];
        // float data
        motor_fbs_logical_float.right[motor_id_t].angle = uint_to_float(angle,POS_MIN,POS_MAX,16);
        motor_fbs_logical_float.right[motor_id_t].speed = uint_to_float(speed,SPD_MIN,SPD_MAX,12);
        motor_fbs_logical_float.right[motor_id_t].current = uint_to_float(current,I_MIN,I_MAX,12);

        if(can_first_rev_flag[motor_id_t+MOTOR_NUM_PER_LEG] == 1) // first receive, save initial pos
        {
          motor_physical_initial_angle[motor_id_t+MOTOR_NUM_PER_LEG] = motor_fbs_logical_float.right[motor_id_t].angle;
          can_first_rev_flag[motor_id_t+MOTOR_NUM_PER_LEG] = 0;
        }
        else // post process motor feedback data
        {
          // logicalize float value
          motor_fbs_logical_float.right[motor_id_t].angle = (motor_fbs_logical_float.right[motor_id_t].angle 
                                                            - motor_physical_initial_angle[motor_id_t+MOTOR_NUM_PER_LEG]) * motor_direction_mask[motor_id_t+MOTOR_NUM_PER_LEG]
                                                            + motor_logical_initial_angle[motor_id_t+MOTOR_NUM_PER_LEG];
          motor_fbs_logical_float.right[motor_id_t].speed *= motor_direction_mask[motor_id_t+MOTOR_NUM_PER_LEG]; // convert to logical speed

          // get integer value of logical feedback
          motor_fbs_logical.right[motor_id_t].angle = float_to_uint(motor_fbs_logical_float.right[motor_id_t].angle,POS_MIN,POS_MAX,16);
          motor_fbs_logical.right[motor_id_t].speed = float_to_uint(motor_fbs_logical_float.right[motor_id_t].speed,SPD_MIN,SPD_MAX,12);
        }
      }
      break;
  }
}

