#ifndef _MEASUREMENT_MODELS_HPP_
#define _MEASUREMENT_MODELS_HPP_

#include <Eigen/Core>

namespace pose_estimation
{

/** Measurement model for the 12D robot state (pose and velocity)
 *  The mask defines which subset is returned.
 */
template <typename PoseWithVelocityType>
Eigen::Matrix<typename PoseWithVelocityType::scalar, -1, 1>
measurementModel (const PoseWithVelocityType &state, const Eigen::Matrix<unsigned, PoseWithVelocityType::DOF, 1>& mask)
{
    return state.getSubStateVector(mask);
}


/** Measurement model for the 12D robot state (pose and velocity)
 *  The mask defines which subset is returned.
 */
template <typename PoseWithVelocityType>
Eigen::Matrix<typename PoseWithVelocityType::scalar, -1, 1>
measurementModelWithBias (const PoseWithVelocityType &state, const Eigen::Matrix<unsigned, PoseWithVelocityType::PoseWithVelocityDOF, 1>& mask)
{
    return state.getSubStateVector(mask) - state.bias.getBiasForSubState(mask);
}
    
}

#endif