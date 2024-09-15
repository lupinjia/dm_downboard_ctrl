#ifndef __FORCE_POSITION_H__
#define __FORCE_POSITION_H__

#include "pd.h"
#include "param.h"

/**
 * @brief initialize force-position control
 * 
 * @return PID* pointer to PID struct array
 */
PDStruct* fp_init();
/**
 * @brief set force-position control to position mode
 * 
 * @param pd_ptr 
 * @param kp 
 * @param kd 
 * @param pos_target 
 * @param i_ff 
 * @param out_max 
 * @return double 
 */
void fp_control_pos(PDStruct* pd_ptr, double kp, double kd, double pos_target, 
                      double i_ff, double out_max);
/**
 * @brief set force-position control to damp mode
 * 
 * @param pd_ptr 
 * @param kd 
 * @param out_max 
 * @return double 
 */
void fp_damp(PDStruct* pd_ptr, double kd, double out_max);
#endif //__FORCE_POSITION_H__