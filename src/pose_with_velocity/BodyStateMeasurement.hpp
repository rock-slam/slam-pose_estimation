#ifndef _POSE_ESTIMATION_BODY_STATE_MEASUREMENT_HPP
#define _POSE_ESTIMATION_BODY_STATE_MEASUREMENT_HPP

#include <base/Time.hpp>
#include <base/samples/RigidBodyState.hpp>
#include <Eigen/Core>
#include "PoseWithVelocity.hpp"

namespace pose_estimation
{

struct BodyStateMeasurement
{
    static void fromRigidBodyState(const base::samples::RigidBodyState &body_state, PoseWithVelocity &filter_state, PoseWithVelocityCovariance &filter_state_cov)
    {
        filter_state.position = TranslationType(body_state.position);
        filter_state.orientation = RotationType(MTK::SO3<double>(body_state.orientation));
        filter_state.velocity = VelocityType(body_state.velocity);
        filter_state.angular_velocity = VelocityType(body_state.angular_velocity);

        filter_state_cov.setZero();
        filter_state_cov.block(0, 0, 3, 3) = body_state.cov_position;
        filter_state_cov.block(3, 3, 3, 3) = body_state.cov_orientation;
        filter_state_cov.block(6, 6, 3, 3) = body_state.cov_velocity;
        filter_state_cov.block(9, 9, 3, 3) = body_state.cov_angular_velocity;
    }

    static void toRigidBodyState(const PoseWithVelocity &filter_state, const PoseWithVelocityCovariance &filter_state_cov, base::samples::RigidBodyState &body_state)
    {
        body_state.position = filter_state.position;
        body_state.orientation = filter_state.orientation;
        body_state.velocity = body_state.orientation * filter_state.velocity;
        body_state.angular_velocity = filter_state.angular_velocity;

        body_state.cov_position = filter_state_cov.block(0, 0, 3, 3);
        body_state.cov_orientation = filter_state_cov.block(3, 3, 3, 3);
        body_state.cov_velocity = filter_state_cov.block(6, 6, 3, 3);
        body_state.cov_angular_velocity = filter_state_cov.block(9, 9, 3, 3);
    }
};

}

#endif