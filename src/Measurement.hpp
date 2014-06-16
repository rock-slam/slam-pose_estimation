#ifndef _POSE_ESTIMATION_MEASUREMENT_HPP
#define _POSE_ESTIMATION_MEASUREMENT_HPP

#include <base/samples/RigidBodyState.hpp>
#include <Eigen/Core>

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
  
  typedef Eigen::Matrix<double, BODY_STATE_SIZE, BODY_STATE_SIZE> Covariance;

struct Measurement
{
    typedef Eigen::Matrix<unsigned, BODY_STATE_SIZE, 1> MemberMask;
    
    base::samples::RigidBodyState body_state;
    MemberMask member_mask;
    
    bool operator()(const Measurement &a, const Measurement &b)
    {
	return a.body_state.time > b.body_state.time;
    }
};

}


#endif