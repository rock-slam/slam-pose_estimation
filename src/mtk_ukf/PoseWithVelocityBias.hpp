#ifndef _POSE_WITH_VELOCITY_BIAS_HPP_
#define _POSE_WITH_VELOCITY_BIAS_HPP_

/** MTK library **/
#include <mtk/src/SubManifold.hpp>
#include <mtk/types/SOn.hpp>
#include <mtk/types/vect.hpp>
#include <mtk/startIdx.hpp>
#include <Eigen/Core>

namespace pose_estimation
{

template<class PoseWithVelocityType, class BiasType>
struct PoseWithVelocityBias
{
    enum {PoseWithVelocityDOF = PoseWithVelocityType::DOF, BiasDOF = BiasType::DOF};
    enum {PoseWithVelocityIdx = 0, BiasIdx = PoseWithVelocityDOF};
	
public:
    typedef PoseWithVelocityType pose_with_velocity_type;
    typedef BiasType bias_type;
    typedef typename pose_with_velocity_type::scalar scalar;
    typedef PoseWithVelocityBias self;
    enum {DOF = PoseWithVelocityDOF + BiasDOF};
    typedef Eigen::Matrix<unsigned, PoseWithVelocityDOF, 1> mask_type;

    // State types
    MTK::SubManifold<PoseWithVelocityType, PoseWithVelocityIdx> pose_with_velocity;
    MTK::SubManifold<BiasType, BiasIdx> bias;
    
    
    // Construct from pose and velocities
    PoseWithVelocityBias(const pose_with_velocity_type &pose_with_velocity = pose_with_velocity_type(), const bias_type& bias = bias_type())
		    : pose_with_velocity(pose_with_velocity), bias(bias) {}
    
    // Copy constructor
    template<class PoseWithVelocityType2, class BiasType2>
    PoseWithVelocityBias(const PoseWithVelocityBias<PoseWithVelocityType2, BiasType2> &state)
		    : pose_with_velocity(state.pose_with_velocity), bias(state.bias) {}
    
    void boxplus(MTK::vectview<const scalar, DOF> state_vec, scalar _scale = 1)
    {
        pose_with_velocity.boxplus(MTK::subvector(state_vec, &PoseWithVelocityBias::pose_with_velocity), _scale);
        bias.boxplus(MTK::subvector(state_vec, &PoseWithVelocityBias::bias), _scale);
    }
    void boxminus(MTK::vectview<scalar, DOF> state_ret, const PoseWithVelocityBias &other) const
    {
        pose_with_velocity.boxminus(MTK::subvector(state_ret, &PoseWithVelocityBias::pose_with_velocity), other.pose_with_velocity);
        bias.boxminus(MTK::subvector(state_ret, &PoseWithVelocityBias::bias), other.bias);
    }
    
    friend std::ostream& operator<<(std::ostream &os, const PoseWithVelocityBias<PoseWithVelocityType, BiasType> &state)
    {
	return os << state.pose_with_velocity << " " << state.bias;
    }
    friend std::istream& operator>>(std::istream &is, PoseWithVelocityBias<PoseWithVelocityType, BiasType> &state)
    {
	return is >> state.pose_with_velocity >> state.bias;
    }

    void applyVelocity(double delta_time)
    {
        pose_with_velocity.applyVelocity(delta_time);
    }

    void applyVelocity(const Eigen::Vector3d& acc, double delta_time)
    {
        pose_with_velocity.applyVelocity(acc, delta_time);
    }
    
    Eigen::Matrix<scalar, PoseWithVelocityDOF, 1> getStateVector() const
    {
        return pose_with_velocity.getStateVector();
    }

    Eigen::Matrix<scalar, -1, 1> getSubStateVector(const mask_type& mask) const
    {
        return pose_with_velocity.getSubStateVector(mask);
    }
};

}

#endif