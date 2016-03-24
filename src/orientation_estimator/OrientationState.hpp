#ifndef _ORIENTATION_STATE_HPP_
#define _ORIENTATION_STATE_HPP_

/** MTK library **/
#include <mtk/src/SubManifold.hpp>
#include <mtk/types/SOn.hpp>
#include <mtk/types/vect.hpp>
#include <mtk/build_manifold.hpp>
#include <mtk/startIdx.hpp>
#include <boost/concept_check.hpp>
#include <pose_estimation/ManifoldHelper.hpp>
#include <Eigen/Core>

namespace pose_estimation
{

typedef MTK::SO3<double> RotationType;
typedef RotationType::vect_type VelocityType;
typedef RotationType::vect_type BiasType;

MTK_BUILD_MANIFOLD(OrientationState,
   ((RotationType, orientation))
   ((VelocityType, velocity)) // navigation/target frame velocity
   ((BiasType, bias_gyro))
   ((BiasType, bias_acc))
)

template<>
inline void getStateVector(const OrientationState &state, Eigen::Matrix<OrientationState::scalar, OrientationState::DOF, 1>& state_vector)
{
    state_vector.block(MTK::getStartIdx(&OrientationState::orientation),0,MTK::getDof(&OrientationState::orientation),1) = MTK::SO3<OrientationState::scalar>::log(state.orientation);
    state_vector.block(MTK::getStartIdx(&OrientationState::velocity),0,MTK::getDof(&OrientationState::velocity),1) = state.velocity;
    state_vector.block(MTK::getStartIdx(&OrientationState::bias_gyro),0,MTK::getDof(&OrientationState::bias_gyro),1) = state.bias_gyro;
    state_vector.block(MTK::getStartIdx(&OrientationState::bias_acc),0,MTK::getDof(&OrientationState::bias_acc),1) = state.bias_acc;
}

}

#endif