#ifndef _MEASUREMENT_MODELS_HPP_
#define _MEASUREMENT_MODELS_HPP_

#include "PoseWithVelocity.hpp"

namespace pose_estimation
{
    
/** Measurement model for the 12D robot state (pose and velocity)
 */
template <typename PoseWithVelocityType>
PoseWithVelocityType
measurementModel (const PoseWithVelocityType &state)
{
    return state;
}
    
}

#endif