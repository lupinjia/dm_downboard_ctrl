#include "dvc_motor.h"
#include "math_ops.h"

// left leg and right leg are 2 buses, and the motor id on same position is the same
const static uint8_t motor_id[MOTOR_NUM_PER_LEG] = {0x01, 0x02, 0x03};
static uint8_t       can_send_flag = 1; // choose which id to send cmd
const static int8_t motor_direction_mask[MOTOR_NUM_PER_LEG*2] = {1, 1, 1, 1, -1, -1}; // direction mask, to convert physical pos to logical pos
// to save initial pos of motors
static uint8_t can_first_rev_flag[MOTOR_NUM_TOTAL] = {1, 1, 1, 1, 1, 1};
float   motor_physical_initial_pos[MOTOR_NUM_TOTAL] = {0, 0, 0, 0, 0, 0};

Motor_Msg motor_rx_msg[MOTOR_NUM_TOTAL]; 

uint8_t motor_id_rv = 0;
// uint8_t motor_comm_mode = 0;

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

/**
 * @brief   send command to motor, callback for TIM6
 * @details the frequency of this function is 300us, which means each motor will receive its cmd every 900us.
 * @return  none
 */
static void send_cmd()
{

//  motor_control_current(motor_id[can_send_flag-1], motor_current_des[can_send_flag-1]*motor_direction_mask[can_send_flag-1], // convert current to physical couterpart
//                        motor_current_des[can_send_flag-1+MOTOR_NUM_PER_LEG]*motor_direction_mask[can_send_flag-1+MOTOR_NUM_PER_LEG]); 
  // motor_control_current(motor_id[can_send_flag-1], 0, 0);
  SetMotorTorque(&FDCAN1_TxFrame, motor_id[can_send_flag-1], 0.0);
  SetMotorTorque(&FDCAN2_TxFrame, motor_id[can_send_flag-1], 0.0);
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

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) // 300us
{
  if(htim->Instance == TIM7) // 300us period, nearly 1ms for each motor
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
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) // fdcan1
{
  int pos_int=0;            
	int spd_int=0;
	int cur_int=0;
  HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &FDCAN12_RxFrame.Header, FDCAN12_RxFrame.Data);

  switch(FDCAN12_RxFrame.Header.Identifier) {
    case 0x001:
    case 0x002:
    case 0x003:
      // Response frame type 1
      if (FDCAN12_RxFrame.Data[0] >> 5 == 1) {
        uint8_t motor_id_t = FDCAN12_RxFrame.Header.Identifier - 1; // motor ID - 1 = index in array
        pos_int=FDCAN12_RxFrame.Data[1]<<8|FDCAN12_RxFrame.Data[2];
			  spd_int=FDCAN12_RxFrame.Data[3]<<4|(FDCAN12_RxFrame.Data[4]&0xF0)>>4;
			  cur_int=(FDCAN12_RxFrame.Data[4]&0x0F)<<8|FDCAN12_RxFrame.Data[5];
			
			  motor_rx_msg[motor_id_t].angle_actual_rad=uint_to_float(pos_int,POS_MIN,POS_MAX,16);
			  motor_rx_msg[motor_id_t].speed_actual_rad=uint_to_float(spd_int,SPD_MIN,SPD_MAX,12);
			  motor_rx_msg[motor_id_t].current_actual_float=uint_to_float(cur_int,I_MIN,I_MAX,12);
			  motor_rx_msg[motor_id_t].temperature=(FDCAN12_RxFrame.Data[6]-50)/2;

        if(can_first_rev_flag[motor_id_t] == 1) // first receive, save initial pos
        {
          motor_physical_initial_pos[motor_id_t] = motor_rx_msg[motor_id_t].angle_actual_rad;
          can_first_rev_flag[motor_id_t] = 0;
        }
        else
        {

        }
      }
      break;
  }
}

void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) // fdcan2
{
  int pos_int=0;            
	int spd_int=0;
	int cur_int=0;
  HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, &FDCAN12_RxFrame.Header, FDCAN12_RxFrame.Data);

  switch(FDCAN12_RxFrame.Header.Identifier) { // judge the motor ID
    case 0x001:
    case 0x002:
    case 0x003:
      // Response frame type 1
      if (FDCAN12_RxFrame.Data[0] >> 5 == 1) {
        uint8_t motor_id_t = FDCAN12_RxFrame.Header.Identifier - 1; // motor ID - 1 = index in array
        pos_int=FDCAN12_RxFrame.Data[1]<<8|FDCAN12_RxFrame.Data[2];
			  spd_int=FDCAN12_RxFrame.Data[3]<<4|(FDCAN12_RxFrame.Data[4]&0xF0)>>4;
			  cur_int=(FDCAN12_RxFrame.Data[4]&0x0F)<<8|FDCAN12_RxFrame.Data[5];
			
			  motor_rx_msg[motor_id_t+MOTOR_NUM_PER_LEG].angle_actual_rad=uint_to_float(pos_int,POS_MIN,POS_MAX,16);
			  motor_rx_msg[motor_id_t+MOTOR_NUM_PER_LEG].speed_actual_rad=uint_to_float(spd_int,SPD_MIN,SPD_MAX,12);
			  motor_rx_msg[motor_id_t+MOTOR_NUM_PER_LEG].current_actual_float=uint_to_float(cur_int,I_MIN,I_MAX,12);
			  motor_rx_msg[motor_id_t+MOTOR_NUM_PER_LEG].temperature=(FDCAN12_RxFrame.Data[6]-50)/2;
        if(can_first_rev_flag[motor_id_t+MOTOR_NUM_PER_LEG] == 1) // first receive, save initial pos
        {
          motor_physical_initial_pos[motor_id_t+MOTOR_NUM_PER_LEG] = motor_rx_msg[motor_id_t+MOTOR_NUM_PER_LEG].angle_actual_rad;
          can_first_rev_flag[motor_id_t+MOTOR_NUM_PER_LEG] = 0;
        }
        else
        {

        }
      }
      break;
  }
}