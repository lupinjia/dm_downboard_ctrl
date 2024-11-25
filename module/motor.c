/*******************************************************************
 * @file    motor.c
 * @brief   PID algorithm
 * 
 * @author  lupinjia
 * @date    20220930
 * @last_modified 20240904
 *********************************************************************/
#include "motor.h"
#include "force_position.h"

/********** Variables **********/
// left leg and right leg are 2 buses, and the motor id on same position is the same
const static uint8_t motor_id[MOTOR_NUM_PER_LEG] = {0x01, 0x02, 0x03};

//! buf不能小于48字节, 否则数组溢出会导致错误, 程序跑飞
static MotorRxMsg  can1_rx_buf; // left leg
static MotorRxMsg  can2_rx_buf; // right leg

// to save initial pos of motors
static uint8_t can_first_rev_flag[MOTOR_NUM_PER_LEG*2] = {1, 1, 1, 1, 1, 1};
// 0-left hip 1-left thigh 2-left calf
// 3-right hip 4-right thigh 5-right calf
static int32_t motor_physical_initial_cnt[MOTOR_NUM_PER_LEG*2] = {0, 0, 0, 0, 0, 0};
double   motor_physical_initial_pos[MOTOR_NUM_PER_LEG*2] = {0, 0, 0, 0, 0, 0}; // for lower board demo
const static double motor_logical_initial_pos[MOTOR_NUM_PER_LEG*2] = {0.33, 1.34904, -2.00, // [rad]
                                                                     -0.33, 1.29971, -2.00};
const static int32_t motor_logical_initial_cnt[MOTOR_NUM_PER_LEG*2] = {34033, 139129, -206264, 
                                                                -34033, 134042, -206264};
const static int8_t motor_direction_mask[MOTOR_NUM_PER_LEG*2] = {-1, -1, 1, 1, -1, -1}; // direction mask, to convert physical pos to logical pos
static MotorTxCmds motor_cmds;
MotorFeedbacksRaw motor_fbs_raw;
MotorFeedbacksLogical motor_fbs_logical;
MotorFeedbacksLogicalFloat motor_fbs_logical_float; // for lower board demo

/********** Function Implementations **********/

const uint8_t* motor_get_id_ptr()
{
    return motor_id;
}

const double*  motor_get_logical_initial_pos()
{
    return motor_logical_initial_pos;
}

const int8_t* motor_get_direction_mask()
{
    return motor_direction_mask;
}

const MotorFeedbacksLogical* motor_get_feedbacks_logical()
{
    return &motor_fbs_logical;
}

/**
 * @brief fill motor command struct
 *
 * @param motor_cmd
 * @param id
 * @param cmd
 * @param loop
 * @param des_value
 * @param kp
 * @param ki
 * @param kd
 * @param s
 * @param max_current
 * @param num_para 0 for changing kp, ki, kd, s; -1 for not changing
 * @note  The msg transmitted will get wrong if
 *        the constants are not re-filled each time before tx
 */
static void motor_fill_cmd(MotorTxCmd *motor_cmd, uint8_t id, uint8_t cmd, int8_t loop, int32_t des_value, 
                    double kp, double ki, double kd, double s, int32_t max_current, int8_t num_para)
{
    motor_cmd->header = 0xAA; // constant
    motor_cmd->id = id;
    motor_cmd->wr = 0x77;     // constant
    motor_cmd->cmd = cmd;     // 0 for enable or disable, 1 for normal loop, 2 for reset, 3 for break
    motor_cmd->value = des_value;
    motor_cmd->kp = (int32_t)(kp * 10000); // refer to manual
    motor_cmd->ki = (int32_t)(ki * 10000);
    motor_cmd->kd = (int32_t)(kd * 10000);
    motor_cmd->s = (int32_t)(s * 10000);   // not consistent with manual. but this one(10000 times) is correct.
    motor_cmd->l_max = max_current;
    motor_cmd->ff_current = 0;
    motor_cmd->loop = loop; // 0 for open-loop, 1 for current-loop, 2 for velocity-loop, 3 for position-loop, 4 for external perception loop
    motor_cmd->num_para = num_para;
    motor_cmd->end = 0x0D;    // constant
    motor_cmd->check = 0x0A;  // constant
    for(int i = 0; i < 12; i++)
    {
        motor_cmd->redundancy[i] = 0x00; // constant
    }
}

static void motor_update_feedback(LEG_TYPE leg_type, uint16_t can_reid)
{
    if(leg_type == LEFT_LEG)
    {
        // raw feedback
        motor_fbs_raw.left[can_reid-1].cnt_encoder = can1_rx_buf.cnt_encoder;
        motor_fbs_raw.left[can_reid-1].speed = can1_rx_buf.spd_motor;
        motor_fbs_raw.left[can_reid-1].current = can1_rx_buf.cur_motor;

        // logical feedback
        motor_fbs_logical.left[can_reid-1].cnt_encoder = can1_rx_buf.cnt_encoder;
        motor_fbs_logical.left[can_reid-1].speed = can1_rx_buf.spd_motor;

        // convert to float
        // these processes will be transferred to upper board
        // for lower board demo
        motor_fbs_logical_float.left[can_reid-1].angle = can1_rx_buf.cnt_encoder / (100.0 * MOTOR_REDUCTION_RATIO);
        motor_fbs_logical_float.left[can_reid-1].angle = motor_fbs_logical_float.left[can_reid-1].angle * M_PI / 180.0; // convert to rad
        motor_fbs_logical_float.left[can_reid-1].speed = can1_rx_buf.spd_motor / MOTOR_REDUCTION_RATIO; // convert to output axis[rpm]
        motor_fbs_logical_float.left[can_reid-1].speed = motor_fbs_logical_float.left[can_reid-1].speed * 2 * M_PI / 60.0; // convert to rad/s
        motor_fbs_logical_float.left[can_reid-1].current = can1_rx_buf.cur_motor / 1000.0; // convert to A
    }
    else if(leg_type == RIGHT_LEG)
    {
        // raw feedback
        motor_fbs_raw.right[can_reid-1].cnt_encoder = can2_rx_buf.cnt_encoder;
        motor_fbs_raw.right[can_reid-1].speed = can2_rx_buf.spd_motor;
        motor_fbs_raw.right[can_reid-1].current = can2_rx_buf.cur_motor;

        // logical feedback
        motor_fbs_logical.right[can_reid-1].cnt_encoder = can2_rx_buf.cnt_encoder;
        motor_fbs_logical.right[can_reid-1].speed = can2_rx_buf.spd_motor;

        // convert to float
        // these processes will be transferred to upper board
        // for lower board demo
        motor_fbs_logical_float.right[can_reid-1].angle = can2_rx_buf.cnt_encoder / (100.0 * MOTOR_REDUCTION_RATIO);
        motor_fbs_logical_float.right[can_reid-1].angle = motor_fbs_logical_float.right[can_reid-1].angle * M_PI / 180.0; // convert to rad
        motor_fbs_logical_float.right[can_reid-1].speed = can2_rx_buf.spd_motor / MOTOR_REDUCTION_RATIO; // convert to output axis[rpm]
        motor_fbs_logical_float.right[can_reid-1].speed = motor_fbs_logical_float.right[can_reid-1].speed * 2 * M_PI / 60.0; // convert to rad/s
        motor_fbs_logical_float.right[can_reid-1].current = can2_rx_buf.cur_motor / 1000.0; // convert to A
    }
}

static void motor_post_process_feedback(LEG_TYPE leg_type, uint16_t can_reid)
{
    if(leg_type == LEFT_LEG)
    {
        // logical feedback, logicalization
        motor_fbs_logical.left[can_reid-1].cnt_encoder = (motor_fbs_logical.left[can_reid-1].cnt_encoder 
                                                         - motor_physical_initial_cnt[can_reid-1]) * motor_direction_mask[can_reid-1] 
                                                         + motor_logical_initial_cnt[can_reid-1];
        motor_fbs_logical.left[can_reid-1].speed *= motor_direction_mask[can_reid-1]; // convert to logical speed
        
        // logicalize float values
        motor_fbs_logical_float.left[can_reid-1].angle = (motor_fbs_logical_float.left[can_reid-1].angle 
                                                    - motor_physical_initial_pos[can_reid-1]) * motor_direction_mask[can_reid-1] 
                                                    + motor_logical_initial_pos[can_reid-1]; // convert to logical pos, for lower board demo
        
		motor_fbs_logical_float.left[can_reid-1].speed *= motor_direction_mask[can_reid-1]; // convert to logical speed, for lower board demo
        motor_fbs_logical_float.left[can_reid-1].current *= motor_direction_mask[can_reid-1]; // convert to logical current
    }
    else if(leg_type == RIGHT_LEG)
    {
        // logical feedback, logicalization
        motor_fbs_logical.right[can_reid-1].cnt_encoder = (motor_fbs_logical.right[can_reid-1].cnt_encoder 
                                                         - motor_physical_initial_cnt[can_reid-1+MOTOR_NUM_PER_LEG]) * motor_direction_mask[can_reid-1+MOTOR_NUM_PER_LEG] 
                                                         + motor_logical_initial_cnt[can_reid-1+MOTOR_NUM_PER_LEG];
        motor_fbs_logical.right[can_reid-1].speed *= motor_direction_mask[can_reid-1+MOTOR_NUM_PER_LEG]; // convert to logical speed

        // logicalize float values
        motor_fbs_logical_float.right[can_reid-1].angle = (motor_fbs_logical_float.right[can_reid-1].angle 
                                                    - motor_physical_initial_pos[can_reid-1+MOTOR_NUM_PER_LEG]) * motor_direction_mask[can_reid-1+MOTOR_NUM_PER_LEG]
                                                    + motor_logical_initial_pos[can_reid-1+MOTOR_NUM_PER_LEG]; // convert to logical pos, for lower board demo
        
        motor_fbs_logical_float.right[can_reid-1].speed *= motor_direction_mask[can_reid-1+MOTOR_NUM_PER_LEG]; // convert to logical speed, for lower board demo
        motor_fbs_logical_float.right[can_reid-1].current *= motor_direction_mask[can_reid-1+MOTOR_NUM_PER_LEG]; // convert to logical current
    }
}

/**
 * @brief enable all motors
 * @note  Refer to manual for parameter setting
 */
void motor_enable()
{
    for(int i = 0; i < MOTOR_NUM_PER_LEG; i++)
    {
        // left leg
        motor_fill_cmd(&motor_cmds.left[i], motor_id[i], 0x00, 0xff, 0x01, 0, 0, 0, 0, 0, 0xff);
        fdcanx_send_data(&hfdcan1, motor_id[i], (uint8_t*)&motor_cmds.left[i], sizeof(MotorTxCmd), 1000);
        // right leg
        motor_fill_cmd(&motor_cmds.right[i], motor_id[i], 0x00, 0xff, 0x01, 0, 0, 0, 0, 0, 0xff);
        fdcanx_send_data(&hfdcan2, motor_id[i], (uint8_t*)&motor_cmds.right[i], sizeof(MotorTxCmd), 1000);
        HAL_Delay(0);
    }
}

/**
 * @brief disable all motors
 * @note  Refer to manual for parameter setting
 */
void motor_disable()
{
    for(int i = 0; i < MOTOR_NUM_PER_LEG; i++)
    {
        // left leg
        motor_fill_cmd(&motor_cmds.left[i], motor_id[i], 0x00, 0xff, 0, 0, 0, 0, 0, 0, 0xff);
        fdcanx_send_data(&hfdcan1, motor_id[i], (uint8_t*)&motor_cmds.left[i], sizeof(MotorTxCmd), 1000);
        // right leg
        motor_fill_cmd(&motor_cmds.right[i], motor_id[i], 0x00, 0xff, 0, 0, 0, 0, 0, 0, 0xff);
        fdcanx_send_data(&hfdcan2, motor_id[i], (uint8_t*)&motor_cmds.right[i], sizeof(MotorTxCmd), 1000);
        HAL_Delay(0);
    }
}
/**
 * @brief initialize motors, including first disable all motors, then set speed-loop parameters
 * 
 * @return MotorTxCmd* static array of motor commands
 */
void motor_init()
{
  memset(&can1_rx_buf, 0, sizeof(MotorRxMsg));
  memset(&can2_rx_buf, 0, sizeof(MotorRxMsg));
  memset(&motor_fbs_raw, 0, sizeof(MotorFeedbacksRaw));
  memset(&motor_fbs_logical, 0, sizeof(MotorFeedbacksLogical));
  memset(&motor_fbs_logical_float, 0, sizeof(MotorFeedbacksLogicalFloat));
  motor_disable();
  HAL_Delay(0);
}

void motor_msg_init(MotorRxMsg *motor_msg)
{
    motor_msg->label[0] = 0x00;
    motor_msg->label[1] = 0x00;
    motor_msg->label[2] = 0x00;
    motor_msg->cmd = 0x00;
    motor_msg->data = 0x00;
    motor_msg->ref = 0x00;
    motor_msg->spd_motor = 0x00;
    motor_msg->cur_motor = 0x00;
    motor_msg->cnt_encoder = 0x00;
    motor_msg->volt_source = 0x00;
    motor_msg->cur_source = 0x00;
    motor_msg->value_out1 = 0x00;
    motor_msg->value_out2 = 0x00;
    motor_msg->value_out3 = 0x00;
    motor_msg->value_out4 = 0x00;
    motor_msg->value_out5 = 0x00;
}

/**
 * @brief control motor current, send one cmd to both buses
 * 
 * @param id               motor id on two buses
 * @param des_value_left   desired current value for specified motor on left leg
 * @param des_value_right  desired current value for specified motor on right leg
 */
void motor_control_current(uint8_t id, int32_t des_value_left, int32_t des_value_right)
{
    // left leg
    motor_fill_cmd(&motor_cmds.left[id-1], id, 0x01, 0x01, des_value_left, 0, 0, 0, 0, MAX_CURRENT, -1); // use default current-loop param
    fdcanx_send_data(&hfdcan1, id, (uint8_t*)&motor_cmds.left[id-1], sizeof(MotorTxCmd), 1000);
    // right leg
    motor_fill_cmd(&motor_cmds.right[id-1], id, 0x01, 0x01, des_value_right, 0, 0, 0, 0, MAX_CURRENT, -1); // use default current-loop param
    fdcanx_send_data(&hfdcan2, id, (uint8_t*)&motor_cmds.right[id-1], sizeof(MotorTxCmd), 1000);
}

/**
 * @brief control motor speed, send one cmd to both buses
 * 
 * @param id 
 * @param des_value_left 
 * @param des_value_right 
 * @param kp 
 * @param ki 
 * @param kd 
 * @param s 
 * @param num_para 
 */
void motor_control_speed(uint8_t id, int32_t des_value_left, int32_t des_value_right, double kp, 
                         double ki, double kd, double s, int8_t num_para)
{
    // left leg
    motor_fill_cmd(&motor_cmds.left[id-1], id, 0x01, 0x02, des_value_left, kp, ki, kd, s, MAX_CURRENT, num_para);
    fdcanx_send_data(&hfdcan1, id, (uint8_t*)&motor_cmds.left[id-1], sizeof(MotorTxCmd), 1000);
    // right leg
    motor_fill_cmd(&motor_cmds.right[id-1], id, 0x01, 0x02, des_value_right, kp, ki, kd, s, MAX_CURRENT, num_para);
    fdcanx_send_data(&hfdcan2, id, (uint8_t*)&motor_cmds.right[id-1], sizeof(MotorTxCmd), 1000);
}

/**
 * @brief control motor position, send one cmd to both buses
 * 
 * @param id 
 * @param des_value_left 
 * @param des_value_right 
 * @param kp 
 * @param ki 
 * @param kd 
 * @param s 
 * @param num_para 
 */
void motor_control_position(uint8_t id, int32_t des_value_left, int32_t des_value_right, double kp, 
                            double ki, double kd, double s, int8_t num_para)
{
    // left leg
    motor_fill_cmd(&motor_cmds.left[id-1], id, 0x01, 0x03, des_value_left, kp, ki, kd, s, MAX_CURRENT, num_para);
    fdcanx_send_data(&hfdcan1, id, (uint8_t*)&motor_cmds.left[id-1], sizeof(MotorTxCmd), 1000);
    // right leg
    motor_fill_cmd(&motor_cmds.right[id-1], id, 0x01, 0x03, des_value_right, kp, ki, kd, s, MAX_CURRENT, num_para);
    fdcanx_send_data(&hfdcan2, id, (uint8_t*)&motor_cmds.right[id-1], sizeof(MotorTxCmd), 1000);
}

void motor_pd_update_real_value(LEG_TYPE leg_type, uint16_t can_reid)
{
    double pos_real;
    double vel_real;
    if(leg_type == LEFT_LEG)
    {
        pos_real = motor_fbs_logical_float.left[can_reid-1].angle;
        vel_real = motor_fbs_logical_float.left[can_reid-1].speed;
    }
    else if(leg_type == RIGHT_LEG)
    {
        pos_real = motor_fbs_logical_float.right[can_reid-1].angle;
        vel_real = motor_fbs_logical_float.right[can_reid-1].speed;
    }
    fp_update_real_value(pos_real, vel_real, leg_type, can_reid);
}

/**
 * @brief     callback function for FDCAN1 RX FIFO0(Left Leg)
 * @details   receive motor feedback, update pd error
 * @attention each callback function handles one motor with 1 id
 */
static void fdcan1_rx_callback(void)
{
	FDCAN_RxHeaderTypeDef pRxHeader;
	if(HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &pRxHeader, (uint8_t*)&can1_rx_buf)==HAL_OK)
	{
	}
	if(pRxHeader.Identifier == 0x789) // ID of response message is 0x789
	{
        uint16_t can_reid = can1_rx_buf.label[1];
        // memcpy(&motor_msgs.motor_msgs_left[can_reid-1], &can1_rx_buf, sizeof(MotorRxMsg));
        motor_update_feedback(LEFT_LEG, can_reid);
        if(can_first_rev_flag[can_reid-1] == 1) // first receive, save initial pos
        {
            motor_physical_initial_cnt[can_reid-1] = can1_rx_buf.cnt_encoder;
            motor_physical_initial_pos[can_reid-1] = motor_fbs_logical_float.left[can_reid-1].angle; // before logicalization, it's physical pos
            can_first_rev_flag[can_reid-1] = 0;
        }
        else
        {
            motor_post_process_feedback(LEFT_LEG, can_reid);
        }
        motor_pd_update_real_value(LEFT_LEG, can_reid);
	}
}

/**
 * @brief   callback function for FDCAN2 RX FIFO1
 * @details receive motor feedback, update pd error
 * 
 */
static void fdcan2_rx_callback(void)
{
	FDCAN_RxHeaderTypeDef pRxHeader;
	if(HAL_FDCAN_GetRxMessage(&hfdcan2, FDCAN_RX_FIFO1, &pRxHeader, (uint8_t*)&can2_rx_buf)==HAL_OK)
	{
	}
	if(pRxHeader.Identifier == 0x789) // ID of response message is 0x789
	{
        uint16_t can_reid = can2_rx_buf.label[1];
        // memcpy(&motor_msgs.motor_msgs_right[can_reid-1], &can2_rx_buf, sizeof(MotorRxMsg));
        motor_update_feedback(RIGHT_LEG, can_reid);
        if(can_first_rev_flag[can_reid-1+MOTOR_NUM_PER_LEG] == 1) // first receive, save initial pos
        {
            motor_physical_initial_cnt[can_reid-1+MOTOR_NUM_PER_LEG] = can2_rx_buf.cnt_encoder;
            motor_physical_initial_pos[can_reid-1+MOTOR_NUM_PER_LEG] = motor_fbs_logical_float.right[can_reid-1].angle; // before logicalization, it's physical pos
            can_first_rev_flag[can_reid-1+MOTOR_NUM_PER_LEG] = 0;
        }
        else
        {
            motor_post_process_feedback(RIGHT_LEG, can_reid);
        }
        motor_pd_update_real_value(RIGHT_LEG, can_reid);
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
    if(hfdcan == &hfdcan1) // left leg
    {
		fdcan1_rx_callback();
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
    if(hfdcan == &hfdcan2) // right leg
    {
        fdcan2_rx_callback();
    }
}