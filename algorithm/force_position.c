#include <string.h>
#include "force_position.h"

// kp and kd for each motor
const static double  kp[3] = {10.0, 10.0, 10.0};
const static double  kd[3] = {0.5,  0.5,  0.5};

LegPDs leg_pds;
static double torque[MOTOR_NUM_PER_LEG*2] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
static int32_t current[MOTOR_NUM_PER_LEG*2] = {0, 0, 0, 0, 0, 0};

void fp_init()
{
    memset(&leg_pds, 0, sizeof(LegPDs));
}

int32_t* fp_get_current_ptr()
{
    return current;
}

void fp_set_target(double* pos_target)
{
    for(int i=0; i<MOTOR_NUM_PER_LEG; i++)
    {
        pd_set_param(&leg_pds.left[i], kp[i], kd[i], pos_target[i], 0.0, 0.0, MAX_CURRENT/1000.0);
        pd_set_param(&leg_pds.right[i], kp[i], kd[i], pos_target[i+MOTOR_NUM_PER_LEG], 0.0, 0.0, MAX_CURRENT/1000.0);
    }
}

void fp_update_real_value(double pos_real, double vel_real, LEG_TYPE leg_type, uint16_t id)
{
    if(leg_type == LEFT_LEG)
    {
        pd_update_real_value(&leg_pds.left[id-1], pos_real, vel_real);
    }
    else if(leg_type == RIGHT_LEG)
    {
        pd_update_real_value(&leg_pds.right[id-1], pos_real, vel_real);
    }
}

void fp_calc_output(uint16_t id)
{
    torque[id-1] = pd_calc_output(&leg_pds.left[id-1]); // left leg
    torque[id-1+MOTOR_NUM_PER_LEG] = pd_calc_output(&leg_pds.right[id-1]); // right leg
    current[id-1] = (int32_t)(torque[id-1] / TORQUE_CONSTANT * 1000.0); // left leg
    current[id-1+MOTOR_NUM_PER_LEG] = (int32_t)(torque[id-1+MOTOR_NUM_PER_LEG] / TORQUE_CONSTANT * 1000.0); // right leg
}

void fp_damp(PDStruct* pd_ptr, double kd, double out_max)
{
    pd_set_param(pd_ptr, 0.0, kd, 0.0, 0.0, 0.0, out_max);
}