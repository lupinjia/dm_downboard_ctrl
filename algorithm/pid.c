/*****************************************************************//**
 * @file    pid.c
 * @brief   PID algorithm
 * 
 * @author  lupinjia
 * @date    20220930
 * @last_modified 20240904
 *********************************************************************/
#include "pid.h"

static PID motor_fp_pd[MOTOR_NUM]; // Motor Force Position PD controller

void pid_init(PID* pid_ptr, double kp, double ki, double kd, double output_max, double i_max)
{
    pid_ptr->kp = kp;
    pid_ptr->ki = ki;
    pid_ptr->kd = kd;
    pid_ptr->err_now = 0;
    pid_ptr->err_last = 0;
    pid_ptr->err_all = 0;
    pid_ptr->p_out = 0;
    pid_ptr->i_out = 0;
    pid_ptr->d_out = 0;
    pid_ptr->output = 0;
    pid_ptr->i_out_max = i_max; 
    pid_ptr->out_max = output_max;
}

void pid_update_err(PID* pid_ptr, double target, double real)
{
    double err = target - real;
    pid_ptr->err_last = pid_ptr->err_now;
    pid_ptr->err_now = err;
    pid_ptr->err_all += err;
}

double pid_calc_output(PID* pid_ptr)
{
    pid_ptr->p_out = pid_ptr->kp * pid_ptr->err_now;
    pid_ptr->i_out = pid_ptr->ki * pid_ptr->err_all;
    if (pid_ptr->i_out_max != 0)
    {
        if (pid_ptr->i_out > pid_ptr->i_out_max)  //restrict the integral output
            pid_ptr->i_out = pid_ptr->i_out_max;
        if (pid_ptr->i_out < -pid_ptr->i_out_max)
            pid_ptr->i_out = -pid_ptr->i_out_max;
    }
    pid_ptr->d_out = pid_ptr->kd * (pid_ptr->err_now - pid_ptr->err_last) / 0.064; //0.064��time_step,�൱���ǿ�������.pid���θ������֮���ʱ���
    pid_ptr->output = pid_ptr->p_out + pid_ptr->i_out + pid_ptr->d_out;
    if (pid_ptr->out_max != 0) // restrict the output
    {
        if (pid_ptr->output > pid_ptr->out_max)
            pid_ptr->output = pid_ptr->out_max;
        if (pid_ptr->output < -pid_ptr->out_max)
            pid_ptr->output = -pid_ptr->out_max;
    }

    return pid_ptr->output;
}
