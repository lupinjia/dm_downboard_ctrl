#ifndef __MOTOR_H__
#define __MOTOR_H__
/********** Includes **********/
#include "stm32h7xx_hal.h"
#include "can_bsp.h"
#include "param.h"

#pragma pack(push, 1) // cancel byte alignment
/*********** Typedefs **********/

// Command Message sent to Motor Driver Board(48 Bytes)
typedef struct
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

// Integration of MotorTxCmd
typedef struct
{
    MotorTxCmd left[MOTOR_NUM_PER_LEG];
    MotorTxCmd right[MOTOR_NUM_PER_LEG];
}MotorTxCmds;

// Raw Message from Motor Driver Board(48 Bytes)
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
    uint8_t redundancy[8];
}MotorRxMsg;

// Integration of MotorRxMsg
typedef struct
{
    MotorRxMsg left[MOTOR_NUM_PER_LEG];
    MotorRxMsg right[MOTOR_NUM_PER_LEG];
}MotorRxMsgs;

// Selected raw motor feedback data, for lower board debug
typedef struct
{
    int32_t cnt_encoder;
    int32_t speed;
    int32_t current;
}MotorFeedbackRaw;

// Integration of MotorFeedback
// 0-hip, 1-thigh, 2-calf
typedef struct
{
    MotorFeedbackRaw left[MOTOR_NUM_PER_LEG];
    MotorFeedbackRaw right[MOTOR_NUM_PER_LEG];
}MotorFeedbacksRaw;

// motor feedback data processed into logical values with int32_t type
// transimitted to upper board
// 8 bytes
typedef struct
{
    int32_t cnt_encoder;
    int32_t speed;
}MotorFeedbackLogical;

// Integration of MotorFeedbackLogical
// 48 bytes
typedef struct
{
    MotorFeedbackLogical left[MOTOR_NUM_PER_LEG];
    MotorFeedbackLogical right[MOTOR_NUM_PER_LEG];
}MotorFeedbacksLogical;

// motor feedback data processed into logical values with float type, for lower board demo
typedef struct
{
    double angle;   // [rad], angle of motor shaft
    double speed;   // [rad/s], speed of motor shaft
    double current; // [A], current of motor
}MotorFeedbackLogicalFloat;

typedef struct
{
    MotorFeedbackLogicalFloat left[MOTOR_NUM_PER_LEG];
    MotorFeedbackLogicalFloat right[MOTOR_NUM_PER_LEG];
}MotorFeedbacksLogicalFloat;

/********** Function Prototypes **********/

const uint8_t* motor_get_id_ptr(void);
const double*  motor_get_logical_initial_pos(void);
const int8_t* motor_get_direction_mask();
const MotorFeedbacksLogical* motor_get_feedbacks_logical(void);
void motor_enable(void);
void motor_disable(void);
void motor_init(void);
void motor_control_speed(uint8_t id, int32_t des_value_left, int32_t des_value_right, double kp, 
                         double ki, double kd, double s, int8_t num_para);
void motor_control_position(uint8_t id, int32_t des_value_left, int32_t des_value_right, double kp, 
                            double ki, double kd, double s, int8_t num_para);
void motor_control_current(uint8_t id, int32_t des_value_left, int32_t des_value_right);
void motor_pd_update_real_value(LEG_TYPE leg_type, uint16_t can_reid);

#pragma pack(pop) // restore byte alignment

#endif // __MOTOR_H__
