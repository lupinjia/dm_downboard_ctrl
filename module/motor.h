#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "stm32h7xx_hal.h"
#include "can_bsp.h"
#include "param.h"
#include "pd.h"

typedef struct __packed
{
    uint8_t header;
    uint8_t id;
    uint8_t wr;
    uint8_t cmd;
    int32_t value;
    int32_t kp;
    int32_t ki;
    int32_t kd;
    int32_t s;
    int32_t l_max;
    int32_t ff_current;
    int8_t loop;
    int8_t num_para;
    uint8_t end;
    uint8_t check;
    uint8_t redundancy[12];
}MotorTxCmd;

typedef struct
{
    float kp;
    float kd;
    float q_des;
    float dq_des;
    float i_ff;   // feedforward current
}ForcePositionControl;

typedef struct
{
    uint8_t label[3];
    uint8_t cmd;
    int32_t data;
    int32_t ref;
    int32_t spd_motor;
    int32_t cur_motor;
    int32_t cnt_encoder; // encoder count(=reduction_ratio *100 *angle of rotor[deg])
    int16_t volt_source; 
    int16_t cur_source;
    int32_t value_out1;
    int32_t value_out2;
    int16_t value_out3;
    int8_t  value_out4;
    uint8_t value_out5;
}MotorRxMsg;  // Raw Message from Motor Driver Board

typedef struct
{
    float angle; // angle of output axis[rad](=angle of rotor/reduction_ratio)
    float speed; // speed of output axis[rad/s](=speed of rotor/reduction_ratio)
    float current; // current [A]

}MotorFeedback;

uint8_t* motor_get_id_ptr(void);
MotorFeedback* motor_feedback_init(void);
MotorTxCmd* motor_cmds_init(void);
void motor_msg_init(MotorRxMsg* motor_msg);
MotorRxMsg* motor_msgs_init(void);
void motor_fill_cmd(MotorTxCmd *motor_cmd, uint8_t id, uint8_t cmd, int8_t loop, 
                    int32_t des_value, float kp, float ki, float kd, float s,
                    int32_t max_current, int8_t num_para);
void motor_update_feedback(MotorRxMsg* motor_msg, MotorFeedback* motor_fb);
void motor_enable(void);
void motor_disable(void);
void motor_control_speed(MotorTxCmd *motor_cmd, uint8_t id, int32_t des_value, float kp, 
                         float ki, float kd, float s, int32_t max_current, int8_t num_para);
void motor_control_position(MotorTxCmd *motor_cmd, uint8_t id, int32_t des_value, float kp, 
                            float ki, float kd, float s, int32_t max_current, int8_t num_para);
void motor_control_current(MotorTxCmd *motor_cmd, uint8_t id, int32_t des_value, int32_t max_current);

void motor_pd_update_real_value(PDStruct* motor_fd_pd, MotorFeedback* motor_fb);

#endif // __MOTOR_H__
