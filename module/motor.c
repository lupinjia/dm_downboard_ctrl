#include "motor.h"

static MotorTxCmd motor_cmd[MOTOR_NUM];
static uint8_t motor_id[MOTOR_NUM] = {0x01, 0x02, 0x03};
static MotorRxMsg motor_msg[MOTOR_NUM];
static MotorFeedback motor_fb[MOTOR_NUM];

uint8_t* motor_get_id_ptr()
{
    return motor_id;
}
MotorFeedback *motor_feedback_init(void)
{
    for(int i = 0; i < MOTOR_NUM; i++)
    {
        motor_fb[i].angle = 0;
        motor_fb[i].speed = 0;
        motor_fb[i].current = 0;
    }
    return motor_fb;
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
 * @param num_para
 * @note  The msg transmitted will get wrong if
 *        the constants are not re-filled each time before tx
 */
void motor_fill_cmd(MotorTxCmd *motor_cmd, uint8_t id, uint8_t cmd, int8_t loop, int32_t des_value, 
                    float kp, float ki, float kd, float s, int32_t max_current, int8_t num_para)
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
    motor_cmd->num_para = num_para; // 0 for changing kp, ki, kd, s; -1 for not changing
    motor_cmd->end = 0x0D;    // constant
    motor_cmd->check = 0x0A;  // constant
    for(int i = 0; i < 12; i++)
    {
        motor_cmd->redundancy[i] = 0x00; // constant
    }
}
void motor_update_feedback(MotorRxMsg *motor_msg, MotorFeedback *motor_fb)
{
    for(int i=0; i<MOTOR_NUM; i++)
    {
        motor_fb[i].angle = motor_msg[i].cnt_encoder / (100.0 * MOTOR_REDUCTION_RATIO);
        motor_fb[i].angle = motor_fb[i].angle * M_PI / 180.0; // convert to radian
        motor_fb[i].speed = motor_msg[i].spd_motor / MOTOR_REDUCTION_RATIO;
        motor_fb[i].current = motor_msg[i].cur_motor;
    }
}
/**
 * @brief disable all motors
 * @note  Refer to manual for parameter setting
 */
void motor_disable()
{
    for(int i = 0; i < MOTOR_NUM; i++)
    {
        motor_fill_cmd(&motor_cmd[i], motor_id[i], 0x00, 0xff, 0, 0, 0, 0, 0, 0, 0xff);
        fdcanx_send_data(&hfdcan1, motor_id[i], (uint8_t*)&motor_cmd[i], sizeof(motor_cmd[i]));
        HAL_Delay(0);
    }
}
/**
 * @brief initialize motors, including first disable all motors, then set speed-loop parameters
 * 
 * @return MotorTxCmd* static array of motor commands
 */
MotorTxCmd* motor_cmds_init()
{
    motor_disable();
    HAL_Delay(0);
    // set spd-loop param
    for(int i=0; i<MOTOR_NUM; i++)
    {
        motor_fill_cmd(&motor_cmd[i], motor_id[i], 0x01, 0x02, 0, 10, 50, 0, 3, 20000, 0);
        fdcanx_send_data(&hfdcan1, motor_id[i], (uint8_t*)&motor_cmd[i], sizeof(motor_cmd[i]));
        HAL_Delay(0);
    }
    //todo: set current pos as 0
    
    
    return motor_cmd;
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

MotorRxMsg* motor_msgs_init()
{
    for(int i=0; i<MOTOR_NUM; i++)
    {
        motor_msg_init(&motor_msg[i]);
    }
    return motor_msg;
}

void motor_enable()
{
    for(int i = 0; i < MOTOR_NUM; i++)
    {
        motor_fill_cmd(&motor_cmd[i], motor_id[i], 0x00, 0xff, 0x01, 0, 0, 0, 0, 0, 0xff);
        fdcanx_send_data(&hfdcan1, motor_id[i], (uint8_t*)&motor_cmd[i], sizeof(motor_cmd[i]));
        HAL_Delay(0);
    }
}

void motor_control_speed(MotorTxCmd *motor_cmd, uint8_t id, int32_t des_value, float kp, 
                         float ki, float kd, float s, int32_t max_current, int8_t num_para)
{
    motor_fill_cmd(motor_cmd, id, 0x01, 0x02, des_value, kp, ki, kd, s, max_current, num_para);
    fdcanx_send_data(&hfdcan1, motor_cmd->id, (uint8_t*)motor_cmd, sizeof(*motor_cmd));
}

void motor_control_position(MotorTxCmd *motor_cmd, uint8_t id, int32_t des_value, float kp, 
                            float ki, float kd, float s, int32_t max_current, int8_t num_para)
{
    motor_fill_cmd(motor_cmd, id, 0x01, 0x03, des_value, kp, ki, kd, s, max_current, num_para);
    fdcanx_send_data(&hfdcan1, motor_cmd->id, (uint8_t*)motor_cmd, sizeof(*motor_cmd));
}                            

