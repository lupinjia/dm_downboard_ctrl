#include <string.h>
#include "force_position.h"

// kp and kd for each motor
const static double  kp[3] = {30.0, 30.0, 30.0};
const static double  kd[3] = {3.0,  3.0,  3.0};
const static double  damp_kd = 1.0;

LegPDs leg_pds;
double torque[MOTOR_NUM_PER_LEG*2] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
int32_t current[MOTOR_NUM_PER_LEG*2] = {0, 0, 0, 0, 0, 0};

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
        pd_set_param(&leg_pds.left[i], kp[i], kd[i], pos_target[i], 0.0, 0.0, MAX_TORQUE_NM);
        pd_set_param(&leg_pds.right[i], kp[i], kd[i], pos_target[i+MOTOR_NUM_PER_LEG], 0.0, 0.0, MAX_TORQUE_NM);
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

void fp_calc_output(uint8_t flag)
{
    torque[flag-1] = pd_calc_output(&leg_pds.left[flag-1]); // left leg
    torque[flag-1+MOTOR_NUM_PER_LEG] = pd_calc_output(&leg_pds.right[flag-1]); // right leg
    current[flag-1] = (int32_t)(torque[flag-1] / TORQUE_CONSTANT * 1000.0); // left leg
    current[flag-1+MOTOR_NUM_PER_LEG] = (int32_t)(torque[flag-1+MOTOR_NUM_PER_LEG] / TORQUE_CONSTANT * 1000.0); // right leg
}

void fp_damp()
{
    for(int i=0; i<MOTOR_NUM_PER_LEG; i++)
    {
        pd_set_param(&leg_pds.left[i], 0.0, damp_kd, 0.0, 0.0, 0.0, MAX_TORQUE_NM);
        pd_set_param(&leg_pds.right[i], 0.0, damp_kd, 0.0, 0.0, 0.0, MAX_TORQUE_NM);
    }
}