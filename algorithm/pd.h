/*****************************************************************//**
 * @file    pd.h
 * @brief   PID algorithm
 * @details ��pid�ṹ��������alg_pid.c��,����alg_pid.h��extern
 * 
 * @author  RobotFreak
 * @date    20220930
 * @license @etherealize
 *********************************************************************/
#ifndef __PD_H__
#define __PD_H__

#include "param.h"

typedef struct
{   
    double kp;          // Proportional gain
    double kd;          // Derivative gain
    double pos_target;
    double pos_real;
    double vel_real;
    double p_out;
    double d_out;
    double output;
    double out_max;     // total output maximum
}PDStruct;

/**
 * @brief initialize pid structs
 * 
 * @return PID* pointer to PID struct array
 */
PDStruct* pds_init();
/**
 * @brief set parameters for pd struct
 * 
 * @param pd_ptr 
 * @param kp 
 * @param kd 
 * @param pos_target 
 * @param out_max 
 */
void pd_set_param(PDStruct* pd_ptr, double kp, double kd, double pos_target, double out_max);
/**
 * @brief   update real value
 * 
 * @param   pd_ptr    PD Struct Pointer
 * @param   target    target value
 * @param   real      current value
 * @return  None
 */
void pd_update_real_value(PDStruct* pd_ptr, double pos_real, double vel_real);
/**
 * @brief   calculate output
 * 
 * @param  pd_ptr  PD Struct Pointer
 * @return double   PD output
 */
double pd_calc_output(PDStruct* pd_ptr);


#endif