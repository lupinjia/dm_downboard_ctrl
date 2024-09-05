/*****************************************************************//**
 * @file    pid.h
 * @brief   PID algorithm
 * @details ��pid�ṹ��������alg_pid.c��,����alg_pid.h��extern
 * 
 * @author  RobotFreak
 * @date    20220930
 * @license @etherealize
 *********************************************************************/
#ifndef __PID_H__
#define __PID_H__

#include "motor.h"

typedef struct
{   
    double kp;          // Proportional gain
    double ki;          // Integral gain
    double kd;          // Derivative gain
    double err_all;
    double err_now;
    double err_last;
    double p_out;
    double i_out;
    double d_out;
    double output;
    double i_out_max;   // integral maximum
    double out_max;     // total output maximum
}PID;


/**
 * @brief  pid initialization
 * 
 * @param  pid_ptr    PID Struct Pointer
 * @param  kp         Proportional gain
 * @param  ki         Integral gain
 * @param  kd         Derivative gain
 * @param  output_max Total output maximum
 * @param  i_max      Integral maximum
 * @return None
 */
void pid_init(PID* pid_ptr, double kp, double ki, double kd, double output_max, double i_max);
/**
 * @brief   update current error
 * 
 * @param   pid_ptr   PID Struct Pointer
 * @param   target    target value
 * @param   real      current value
 * @return  None
 */
void pid_update_err(PID* pid_ptr, double target, double real);
/**
 * @brief   calculate output
 * 
 * @param  pid_ptr  PID Struct Pointer
 * @return double   PID output
 */
double pid_calc_output(PID* pid_ptr);


#endif