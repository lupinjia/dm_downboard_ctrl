#ifndef __FORCE_POSITION_H__
#define __FORCE_POSITION_H__

#include "pd.h"
#include "param.h"
#include "stm32h7xx_hal.h"

typedef struct
{
    PDStruct left[MOTOR_NUM_PER_LEG];
    PDStruct right[MOTOR_NUM_PER_LEG];
}LegPDs;


/**
 * @brief initialize force-position control
 * 
 * @return PID* pointer to PID struct array
 */
void fp_init();
/**
 * @brief Get the current ptr object
 * 
 * @return float* 
 */
int32_t* fp_get_current_ptr();
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
void fp_set_target(double* pos_target);

/**
 * @brief update real value of force-position control
 * 
 * @param real_value 
 * @param can_reid 
 */
void fp_update_real_value(double pos_real, double vel_real, LEG_TYPE leg_type, uint16_t id);
/**
 * @brief set force-position control to damp mode
 * 
 * @param pd_ptr 
 * @param kd 
 * @param out_max 
 * @return double 
 */
void fp_damp(PDStruct* pd_ptr, double kd, double out_max);

/**
 * @brief calculate output of force-position control
 * 
 * @param id 
 */
void fp_calc_output(uint16_t id);
#endif //__FORCE_POSITION_H__