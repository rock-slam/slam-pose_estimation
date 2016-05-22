#ifndef _POSE_ESTIMATION_BODY_STATE_MEASUREMENT_HPP
#define _POSE_ESTIMATION_BODY_STATE_MEASUREMENT_HPP

#include <base/Time.hpp>
#include <base/samples/RigidBodyState.hpp>
#include <Eigen/Core>
#include <pose_estimation/Measurement.hpp>
#include <pose_estimation/EulerConversion.hpp>

namespace pose_estimation
{

enum BodyStateMembers
{
    BodyStateMemberX = 0,
    BodyStateMemberY,
    BodyStateMemberZ,
    BodyStateMemberRoll,
    BodyStateMemberPitch,
    BodyStateMemberYaw,
    BodyStateMemberVx,
    BodyStateMemberVy,
    BodyStateMemberVz,
    BodyStateMemberVroll,
    BodyStateMemberVpitch,
    BodyStateMemberVyaw
};

const int BODY_STATE_SIZE = 12;

typedef Eigen::Matrix<double, BODY_STATE_SIZE, BODY_STATE_SIZE> BodyStateCovariance;
typedef Eigen::Matrix<unsigned, BODY_STATE_SIZE, 1> MemberMask;

struct BodyStateMeasurement
{
    static void fromBodyStateToMeasurement(const base::samples::RigidBodyState& body_state, const MemberMask& member_mask, Measurement& measurement)
    {
        measurement.time = body_state.time;
        StateAndCovariance::Mu mu;
        StateAndCovariance::Cov cov;
        fromRigidBodyState(body_state, mu, cov);

        unsigned members = member_mask.count();
        measurement.mu.resize(members);
        measurement.cov.resize(members, members);
        measurement.cov.setZero();
        measurement.state_mapping.resize(members);

        // fill mu and state mapping
        unsigned sub_index = 0;
        for(unsigned r = 0; r < BODY_STATE_SIZE; r++)
        {
            if(member_mask(r) > 0)
            {
                measurement.mu(sub_index) = mu(r);
                measurement.state_mapping(sub_index) = r;
                sub_index++;
            }
        }

        // fill covariance
        for(unsigned c = 0; c < members; c++)
        {
            for(unsigned r = 0; r < members; r++)
            {
                measurement.cov(r,c) = cov(measurement.state_mapping(r), measurement.state_mapping(c));
            }
        }

        measurement.measurement_name = "RigidBodyState";
        measurement.measurement_name += ";";
        measurement.measurement_name += body_state.sourceFrame;
        measurement.measurement_name += "In";
        measurement.measurement_name += body_state.targetFrame;
        measurement.integration = pose_estimation::StateMapping;
    }

    static void fromRigidBodyState(const base::samples::RigidBodyState &body_state, StateAndCovariance::Mu &mu, StateAndCovariance::Cov &cov)
    {
        mu.resize(BODY_STATE_SIZE);
        cov.resize(BODY_STATE_SIZE, BODY_STATE_SIZE);

        mu.setZero();
        mu.block(0, 0, 3, 1) = body_state.position;
        Eigen::Vector3d euler;
        EulerConversion::quadToEuler(body_state.orientation, euler);
        mu.block(3, 0, 3, 1) = euler;
        mu.block(6, 0, 3, 1) = body_state.velocity;
        mu.block(9, 0, 3, 1) = body_state.angular_velocity;

        cov.setZero();
        cov.block(0, 0, 3, 3) = body_state.cov_position;
        cov.block(3, 3, 3, 3) = body_state.cov_orientation;
        cov.block(6, 6, 3, 3) = body_state.cov_velocity;
        cov.block(9, 9, 3, 3) = body_state.cov_angular_velocity;
    }

    static void toRigidBodyState(const StateAndCovariance::Mu &mu, const StateAndCovariance::Cov &cov, base::samples::RigidBodyState &body_state)
    {
        assert(mu.rows() >= BODY_STATE_SIZE);
        assert(cov.rows() >= BODY_STATE_SIZE && cov.cols() >= BODY_STATE_SIZE);

        body_state.position = mu.block(0,0,3,1);
        Eigen::Vector3d euler = mu.block(3,0,3,1);
        EulerConversion::eulerToQuad(euler, body_state.orientation);
        body_state.velocity = body_state.orientation * mu.block(6,0,3,1);
        body_state.angular_velocity = mu.block(9, 0, 3, 1);

        body_state.cov_position = cov.block(0, 0, 3, 3);
        body_state.cov_orientation = cov.block(3, 3, 3, 3);
        body_state.cov_velocity = cov.block(6, 6, 3, 3);
        body_state.cov_angular_velocity = cov.block(9, 9, 3, 3);
    }

    static bool hasPositionMeasurement(const MemberMask& member_mask)
    {
        if(member_mask[BodyStateMemberX] || member_mask[BodyStateMemberY] || member_mask[BodyStateMemberZ])
            return true;
        return false;
    }

    static bool hasOrientationMeasurement(const MemberMask& member_mask)
    {
        if(member_mask[BodyStateMemberRoll] || member_mask[BodyStateMemberPitch] || member_mask[BodyStateMemberYaw])
            return true;
        return false;
    }

    static bool hasVelocityMeasurement(const MemberMask& member_mask)
    {
        if(member_mask[BodyStateMemberVx] || member_mask[BodyStateMemberVy] || member_mask[BodyStateMemberVz])
            return true;
        return false;
    }

    static bool hasAngularVelocityMeasurement(const MemberMask& member_mask)
    {
        if(member_mask[BodyStateMemberVroll] || member_mask[BodyStateMemberVpitch] || member_mask[BodyStateMemberVyaw])
            return true;
        return false;
    }
};

}

#endif