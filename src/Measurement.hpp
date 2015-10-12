#ifndef _POSE_ESTIMATION_MEASUREMENT_HPP
#define _POSE_ESTIMATION_MEASUREMENT_HPP

#include <base/samples/RigidBodyState.hpp>
#include <base/samples/RigidBodyAcceleration.hpp>
#include <Eigen/Core>

namespace pose_estimation
{

struct StateAndCovariance
{
    typedef double scalar;
    typedef Eigen::Matrix<scalar, Eigen::Dynamic, 1> Mu;
    typedef Eigen::Matrix<scalar, Eigen::Dynamic, Eigen::Dynamic> Cov;

    Mu mu;
    Cov cov;
};

struct Measurement : public StateAndCovariance
{
    typedef Eigen::Matrix<unsigned, Eigen::Dynamic, 1> StateMapping;
    typedef Eigen::Matrix<unsigned, Eigen::Dynamic, 1> StateMask;

    base::Time time;

    // This maps the measurement state to the actual filter state
    //StateMapping map;
    StateMask mask;

    std::string measurement_name;
};


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
    BodyStateMemberVyaw,
    BodyStateMemberAx,
    BodyStateMemberAy,
    BodyStateMemberAz,
};

const int BODY_STATE_SIZE = 12;
const int MEASUREMENT_SIZE = BODY_STATE_SIZE + 3;

typedef Eigen::Matrix<double, BODY_STATE_SIZE, BODY_STATE_SIZE> Covariance;

struct BodyStateMeasurement
{
    typedef Eigen::Matrix<unsigned, MEASUREMENT_SIZE, 1> MemberMask;
    
    base::Time time;
    base::samples::RigidBodyState body_state;
    base::samples::RigidBodyAcceleration acceleration;
    MemberMask member_mask;

    BodyStateMeasurement() : member_mask(MemberMask::Zero()) {}
    
    bool operator()(const BodyStateMeasurement &a, const BodyStateMeasurement &b) const
    {
	return a.time > b.time;
    }
    
    bool hasPositionMeasurement() const
    {
        if(member_mask[BodyStateMemberX] || member_mask[BodyStateMemberY] || member_mask[BodyStateMemberZ])
            return true;
        return false;
    }
    
    bool hasOrientationMeasurement() const
    {
        if(member_mask[BodyStateMemberRoll] || member_mask[BodyStateMemberPitch] || member_mask[BodyStateMemberYaw])
            return true;
        return false;
    }
    
    bool hasVelocityMeasurement() const
    {
        if(member_mask[BodyStateMemberVx] || member_mask[BodyStateMemberVy] || member_mask[BodyStateMemberVz])
            return true;
        return false;
    }
    
    bool hasAngularVelocityMeasurement() const
    {
        if(member_mask[BodyStateMemberVroll] || member_mask[BodyStateMemberVpitch] || member_mask[BodyStateMemberVyaw])
            return true;
        return false;
    }

    bool hasAccelerationMeasurement() const
    {
        if(member_mask[BodyStateMemberAx] || member_mask[BodyStateMemberAy] || member_mask[BodyStateMemberAz])
            return true;
        return false;
    }
};

}


#endif