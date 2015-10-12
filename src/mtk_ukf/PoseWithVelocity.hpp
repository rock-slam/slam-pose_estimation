#ifndef _POSE_WITH_VELOCITY_HPP_
#define _POSE_WITH_VELOCITY_HPP_

/** MTK library **/
#include <mtk/src/SubManifold.hpp>
#include <mtk/types/SOn.hpp>
#include <mtk/types/vect.hpp>
#include <mtk/startIdx.hpp>
#include <boost/concept_check.hpp>

#include <base/Pose.hpp>

namespace pose_estimation
{

template<class RotationType, class TranslationType = typename RotationType::vect_type, class VelocityType = typename RotationType::vect_type>
struct PoseWithVelocity
{
    enum {TranslationDOF = TranslationType::DOF, RotationDOF = RotationType::DOF, 
	  VelocityDOF = VelocityType::DOF, AngularVelocityDOF = VelocityType::DOF};
    enum {TranslationIdx = 0, RotationIdx = TranslationDOF, VelocityIdx = TranslationDOF + RotationDOF, 
	  AngularVelocityIdx = TranslationDOF + RotationDOF + VelocityDOF};
	
public:
    typedef TranslationType translation_type;
    typedef RotationType rotation_type;
    typedef VelocityType velocity_type;
    typedef typename rotation_type::scalar scalar;
    typedef PoseWithVelocity self;
    enum {DOF = TranslationDOF + RotationDOF + VelocityDOF + AngularVelocityDOF};
    typedef Eigen::Matrix<scalar, int(DOF), int(DOF)> cov;

    // State types
    MTK::SubManifold<TranslationType, TranslationIdx> position;
    MTK::SubManifold<RotationType, RotationIdx> orientation;
    MTK::SubManifold<VelocityType, VelocityIdx> velocity;
    MTK::SubManifold<VelocityType, AngularVelocityIdx> angular_velocity;
    
    
    // Construct from pose and velocities
    PoseWithVelocity(const translation_type &position = translation_type(), const rotation_type& orientation = rotation_type(), const velocity_type &velocity = velocity_type(), const velocity_type &angular_velocity = velocity_type()) 
		    : position(position), orientation(orientation), velocity(velocity), angular_velocity(angular_velocity) {}
    
    // Copy constructor
    template<class RotationType2, class TranslationType2, class VelocityType2>
    PoseWithVelocity(const PoseWithVelocity<RotationType2, TranslationType2, VelocityType2> &state) 
		    : position(state.position), orientation(state.orientation), velocity(state.velocity), angular_velocity(state.angular_velocity) {}
    
    void boxplus(MTK::vectview<const scalar, DOF> state_vec, scalar _scale = 1)
    {
	position.boxplus(MTK::subvector(state_vec, &PoseWithVelocity::position), _scale);
	orientation.boxplus(MTK::subvector(state_vec, &PoseWithVelocity::orientation), _scale);
	velocity.boxplus(MTK::subvector(state_vec, &PoseWithVelocity::velocity), _scale);
	angular_velocity.boxplus(MTK::subvector(state_vec, &PoseWithVelocity::angular_velocity), _scale);
    }
    void boxminus(MTK::vectview<scalar, DOF> state_ret, const PoseWithVelocity &other) const
    {
	position.boxminus(MTK::subvector(state_ret, &PoseWithVelocity::position), other.position);
	orientation.boxminus(MTK::subvector(state_ret, &PoseWithVelocity::orientation), other.orientation);
	velocity.boxminus(MTK::subvector(state_ret, &PoseWithVelocity::velocity), other.velocity);
	angular_velocity.boxminus(MTK::subvector(state_ret, &PoseWithVelocity::angular_velocity), other.angular_velocity);
    }
    
    friend std::ostream& operator<<(std::ostream &os, const PoseWithVelocity<RotationType, TranslationType, VelocityType> &state)
    {
	return os << state.position << " " << state.orientation << " " << state.velocity << " " << state.angular_velocity;
    }
    friend std::istream& operator>>(std::istream &is, PoseWithVelocity<RotationType, TranslationType, VelocityType> &state)
    {
	return is >> state.position >> state.orientation >> state.velocity >> state.angular_velocity;
    }
    
    void applyVelocity(double delta_time)
    {
	position.boxplus(orientation * velocity, delta_time);
	orientation.boxplus(angular_velocity, delta_time);
    }

    void applyVelocity(const Eigen::Vector3d& acc, double delta_time)
    {
        velocity.boxplus(acc, delta_time);
        position.boxplus(orientation * velocity, delta_time);
        orientation.boxplus(angular_velocity, delta_time);
    }
    
    Eigen::Matrix<scalar, DOF, 1> getStateVector() const
    {
	Eigen::Matrix<scalar, DOF, 1> state_vector;
	state_vector.block(TranslationIdx,0,TranslationDOF,1) = position;
	state_vector.block(RotationIdx,0,RotationDOF,1) = MTK::SO3<scalar>::log(orientation);
	state_vector.block(VelocityIdx,0,VelocityDOF,1) = velocity;
	state_vector.block(AngularVelocityIdx,0,AngularVelocityDOF,1) = angular_velocity;
	return state_vector;
    }

    Eigen::Matrix<scalar, -1, 1> getSubStateVector(const Eigen::Matrix<unsigned, DOF, 1>& mask) const
    {
        Eigen::Matrix<scalar, DOF, 1> state_vector = getStateVector();
        Eigen::Matrix<scalar, -1, 1> sub_state_vector;
        sub_state_vector.resize(mask.count());
        unsigned sub_index = 0;
        for(unsigned i = 0; i < DOF; i++)
        {
            if(mask(i) > 0)
            {
                sub_state_vector(sub_index) = state_vector(i);
                sub_index++;
            }
        }
        return sub_state_vector;
    }
};

}

#endif