#include "force_position.h"

static PDStruct motor_fp_pd[MOTOR_NUM]; // Motor Force Position PD controller

PDStruct* fp_init()
{
    for(int i = 0;i < MOTOR_NUM; i++)
    {
        motor_fp_pd[i].kp = 0.0;
        motor_fp_pd[i].kd = 0.0;
        motor_fp_pd[i].value_target = 0.0;
        motor_fp_pd[i].value_real = 0.0;
        motor_fp_pd[i].value_diff_target = 0.0;
        motor_fp_pd[i].value_diff_real = 0.0;
        motor_fp_pd[i].p_out = 0.0;
        motor_fp_pd[i].d_out = 0.0;
        motor_fp_pd[i].output = 0.0;
        motor_fp_pd[i].value_ff = 0.0;
        motor_fp_pd[i].out_max = 0.0;
    }
    return motor_fp_pd;
}

void fp_control_pos(PDStruct* pd_ptr, double kp, double kd, double pos_target, 
                      double i_ff, double out_max)
{
    pd_ptr->kp = kp;
    pd_ptr->kd = kd;
    pd_ptr->value_target = pos_target;
    pd_ptr->value_diff_target = 0.0;
    pd_ptr->value_ff = i_ff;
    pd_ptr->out_max = out_max;
}

void fp_damp(PDStruct* pd_ptr, double kd, double out_max)
{
    pd_ptr->kp = 0.0;
    pd_ptr->kd = kd;
    pd_ptr->value_target = 0.0;
    pd_ptr->value_diff_target = 0.0;
    pd_ptr->value_ff = 0.0;
    pd_ptr->out_max = out_max;
}