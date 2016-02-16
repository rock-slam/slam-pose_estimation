#ifndef _POSE_WITH_VELOCITY_HPP_
#define _POSE_WITH_VELOCITY_HPP_

/** MTK library **/
#include <mtk/src/SubManifold.hpp>
#include <mtk/types/SOn.hpp>
#include <mtk/types/vect.hpp>
#include <mtk/build_manifold.hpp>
#include <mtk/startIdx.hpp>
#include <boost/concept_check.hpp>

namespace pose_estimation
{

typedef MTK::SO3<double> RotationType;
typedef RotationType::vect_type TranslationType;
typedef RotationType::vect_type VelocityType;

MTK_BUILD_MANIFOLD(PoseWithVelocity,
   ((TranslationType, position))
   ((RotationType, orientation))
   ((VelocityType, velocity))
   ((VelocityType, angular_velocity))
)

}

#endif