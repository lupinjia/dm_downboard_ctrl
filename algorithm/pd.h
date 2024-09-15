/*****************************************************************//**
 * @file    pd.h
 * @brief   PD algorithm
 * @details 
 * 
 * @author  lupinjia
 * @date    20220930
 * @last_modified 20240914
 *********************************************************************/
#ifndef __PD_H__
#define __PD_H__



typedef struct
{   
    double kp;              // Proportional gain
    double kd;              // Derivative gain
    double value_target;
    double value_real;
    double value_diff_target;
    double value_diff_real;
    double p_out;
    double d_out;
    double output;
    double value_ff;        // feedforward value
    double out_max;         // total output maximum
}PDStruct;

/**
 * @brief set parameters for pd struct
 * 
 * @param pd_ptr 
 * @param kp 
 * @param kd 
 * @param value_target        target value
 * @param value_diff_target   target differential value
 * @param value_ff            feedforward value
 * @param out_max             total output maximum
 */
void pd_set_param(PDStruct* pd_ptr, double kp, double kd, double value_target,
                  double value_diff_target, double value_ff, double out_max);
/**
 * @brief   update real value
 * 
 * @param   pd_ptr    PD Struct Pointer
 * @param   value_real           real value
 * @param   value_diff_real      real differential value
 * @return  None
 */
void pd_update_real_value(PDStruct* pd_ptr, double value_real, double value_diff_real);
/**
 * @brief   calculate output
 * 
 * @param  pd_ptr  PD Struct Pointer
 * @return double   PD output
 */
double pd_calc_output(PDStruct* pd_ptr);


#endif