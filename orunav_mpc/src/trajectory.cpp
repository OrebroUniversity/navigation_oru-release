#include "trajectory.h"

//======================================================
// Constraints
//======================================================

/**
 * @brief Constructor, all constraints are set to default values.
 *
 * @param[in] param parameters
 */
Constraints::Constraints(const swParameters &param)
{
    v_min = -param.max_tangential_velocity;
    v_max =  param.max_tangential_velocity;

    w_min = -param.max_steer_angular_velocity;
    w_max =  param.max_steer_angular_velocity;

    phi_min = -param.max_steer_angle;
    phi_max =  param.max_steer_angle;

    theta_min = -param.max_orientation_angle;
    theta_max =  param.max_orientation_angle;

    tanacc_min = -param.max_tangential_acceleration;
    tanacc_max =  param.max_tangential_acceleration;

    cenacc_max =  param.max_centripetal_acceleration;
    
    pos_constraints_num = 0;
}



//======================================================
// trajectoryStep
//======================================================

/**
 * @brief Constructor.
 */
trajectoryStep::trajectoryStep()
{
    last = false;
    mode = SW_STEP_MODE_DEFAULT;
}
