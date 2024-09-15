/*****************************************************************//**
 * @file    pd.c
 * @brief   PID algorithm
 * 
 * @author  lupinjia
 * @date    20220930
 * @last_modified 20240904
 *********************************************************************/
#include "pd.h"

void pd_set_param(PDStruct* pd_ptr, double kp, double kd, double value_target,
                  double value_diff_target, double value_ff, double out_max)
{
    pd_ptr->kp = kp;
    pd_ptr->kd = kd;
    pd_ptr->value_target = value_target;
    pd_ptr->value_diff_target = value_diff_target;
    pd_ptr->value_ff = value_ff;
    pd_ptr->out_max = out_max;
}

void pd_update_real_value(PDStruct *pd_ptr, double value_real, double value_diff_real)
{
    pd_ptr->value_real = value_real;
    pd_ptr->value_diff_real = value_diff_real;
}

double pd_calc_output(PDStruct* pd_ptr)
{
    pd_ptr->p_out = pd_ptr->kp * (pd_ptr->value_target - pd_ptr->value_real);
    pd_ptr->d_out = pd_ptr->kd * (pd_ptr->value_diff_target - pd_ptr->value_diff_real);
    pd_ptr->output = pd_ptr->p_out + pd_ptr->d_out + pd_ptr->value_ff;
    if (pd_ptr->out_max != 0) // restrict the output
    {
        if (pd_ptr->output > pd_ptr->out_max)
            pd_ptr->output = pd_ptr->out_max;
        if (pd_ptr->output < -pd_ptr->out_max)
            pd_ptr->output = -pd_ptr->out_max;
    }

    return pd_ptr->output;
}
