/*****************************************************************//**
 * @file    pd.c
 * @brief   PID algorithm
 * 
 * @author  lupinjia
 * @date    20220930
 * @last_modified 20240904
 *********************************************************************/
#include "pd.h"

static PDStruct motor_fp_pd[MOTOR_NUM]; // Motor Force Position PD controller

PDStruct* pds_init()
{
    for(int i = 0;i < MOTOR_NUM; i++)
    {
        motor_fp_pd[i].kp = 0.0;
        motor_fp_pd[i].kd = 0.0;
        motor_fp_pd[i].pos_target = 0.0;
        motor_fp_pd[i].pos_real = 0.0;
        motor_fp_pd[i].vel_real = 0.0;
        motor_fp_pd[i].p_out = 0.0;
        motor_fp_pd[i].d_out = 0.0;
        motor_fp_pd[i].output = 0.0;
        motor_fp_pd[i].out_max = 0.0;
    }
    return motor_fp_pd;
}

void pd_set_param(PDStruct *pd_ptr, double kp, double kd, double pos_target, double out_max)
{
    pd_ptr->kp = kp;
    pd_ptr->kd = kd;
    pd_ptr->pos_target = pos_target;
    pd_ptr->out_max = out_max;
}

void pd_update_real_value(PDStruct *pd_ptr, double pos_real, double vel_real)
{
    pd_ptr->pos_real = pos_real;
    pd_ptr->vel_real = vel_real;
}

double pd_calc_output(PDStruct* pd_ptr)
{
    pd_ptr->p_out = pd_ptr->kp * (pd_ptr->pos_target - pd_ptr->pos_real);
    pd_ptr->d_out = pd_ptr->kd * (0 - pd_ptr->vel_real);
    pd_ptr->output = pd_ptr->p_out + pd_ptr->d_out;
    if (pd_ptr->out_max != 0) // restrict the output
    {
        if (pd_ptr->output > pd_ptr->out_max)
            pd_ptr->output = pd_ptr->out_max;
        if (pd_ptr->output < -pd_ptr->out_max)
            pd_ptr->output = -pd_ptr->out_max;
    }

    return pd_ptr->output;
}
