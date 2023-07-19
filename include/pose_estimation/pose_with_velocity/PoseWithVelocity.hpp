#ifndef _POSE_WITH_VELOCITY_HPP_
#define _POSE_WITH_VELOCITY_HPP_

/** MTK library **/
#include <mtk/types/SOn.hpp>
#include <mtk/types/vect.hpp>
#include <mtk/startIdx.hpp>
#include <ukfom/mtkwrap.hpp>
#include <mtk/build_manifold.hpp>

namespace pose_estimation
{

typedef ukfom::mtkwrap< MTK::SO3<double> > RotationType;
typedef ukfom::mtkwrap<RotationType::vect_type> TranslationType;
typedef ukfom::mtkwrap<RotationType::vect_type> VelocityType;

MTK_BUILD_MANIFOLD(PoseWithVelocity,
   ((TranslationType, position))
   ((RotationType, orientation))
   ((VelocityType, velocity))
   ((VelocityType, angular_velocity))
)

typedef Eigen::Matrix<PoseWithVelocity::scalar, PoseWithVelocity::DOF, PoseWithVelocity::DOF> PoseWithVelocityCovariance;

}

#endif