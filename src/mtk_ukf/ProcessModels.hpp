#ifndef _PROCESS_MODELS_HPP_
#define _PROCESS_MODELS_HPP_

#include "PoseWithVelocity.hpp"

namespace pose_estimation
{

/** Process model for the 12D robot state.
 * Applies the current velocity to update the robot pose.
 */
template <typename PoseWithVelocityType>
PoseWithVelocityType processModel (const PoseWithVelocityType &state, double delta_time)
{    
    PoseWithVelocityType new_state(state);
    new_state.applyVelocity(delta_time);
    return new_state;
}

/** Process model with acceleration for the 12D robot state.
 * Applies the current velocity and acceleration to update the robot pose.
 */
template <typename PoseWithVelocityType>
PoseWithVelocityType processModelWithAcceleration (const PoseWithVelocityType &state, const Eigen::Vector3d& acc, double delta_time)
{
    PoseWithVelocityType new_state(state);
    new_state.applyVelocity(acc, delta_time);
    return new_state;
}
    
}

#endif