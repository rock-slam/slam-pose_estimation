#include "PoseUKF.hpp"
#include <pose_estimation/mtk_ukf/ProcessModels.hpp>
#include <pose_estimation/mtk_ukf/MeasurementModels.hpp>

namespace pose_estimation
{

PoseUKF::PoseUKF() : dirty(true)
{
    body_state.initUnknown();

    process_noise_cov = Covariance::Zero();
    MTK::setDiagonal(process_noise_cov, &WPoseState::position, 0.01);
    MTK::setDiagonal(process_noise_cov, &WPoseState::orientation, 0.001);
    MTK::setDiagonal(process_noise_cov, &WPoseState::velocity, 0.00001);
    MTK::setDiagonal(process_noise_cov, &WPoseState::angular_velocity, 0.00001);
}

void PoseUKF::setInitialState(const base::samples::RigidBodyState& body_state)
{
    dirty = true;
    
    if(ukf.use_count() > 0)
    {
	ukf.reset();
    }
    
    WPoseState state;
    Covariance cov;
    rigidBodyStateToUKFState(body_state, state, cov);
    
    ukf.reset(new MTK_UKF(state, cov));
}

void PoseUKF::setProcessNoiseCovariance(const Covariance& noise_cov)
{
    process_noise_cov = noise_cov;
}

void PoseUKF::predictionStep(const double delta)
{
    if(ukf.use_count() == 0)
    {
	setInitialState(body_state);
    }
    dirty = true;
    
    ukf->predict(boost::bind(processModel<WPoseState>, _1 , delta), MTK_UKF::cov(delta * process_noise_cov));
}

void PoseUKF::correctionStep(const Measurement& measurement)
{
    if(ukf.use_count() == 0)
    {
	setInitialState(body_state);
    }
    dirty = true;
    
    WPoseState state;
    Covariance cov;
    rigidBodyStateToUKFState(measurement.body_state, state, cov);
    
    // augment the current state by new measurements
    WPoseState augmented_state(ukf->mu());
    augmented_state.applyState(state, measurement.member_mask);
    
    // the unknown parts of the covariance matrix to the highest possible error (infinity in non-discrete case)
    for(unsigned i = 0; i < WPoseState::DOF; i++)
	for(unsigned j = 0; j < WPoseState::DOF; j++)
	{
	    if((measurement.member_mask(i,0) * measurement.member_mask(j,0)) == 0)
	    {
		cov(i,j) = 0.0;
		if(i==j)
		    cov(i,j) = std::numeric_limits<double>::max();
	    }
	}
    
    // apply new measurement
    ukf->update(augmented_state, boost::bind(measurementModel<WPoseState>, _1), 
		boost::bind(ukfom::id<MTK_UKF::cov>, cov), ukfom::accept_any_mahalanobis_distance<MTK_UKF::scalar_type>);
}

const base::samples::RigidBodyState& PoseUKF::getCurrentState()
{
    if(dirty)
	UKFStateToRigidBodyState(ukf->mu(), ukf->sigma(), body_state);
    
    return body_state;
}

void PoseUKF::rigidBodyStateToUKFState(const base::samples::RigidBodyState& body_state, PoseUKF::WPoseState& state, ukfom::ukf< PoseUKF::WPoseState >::cov& covariance)
{
    state.position = body_state.position;
    state.orientation = MTK::SO3<double>(body_state.orientation);
    state.velocity = body_state.velocity;
    state.angular_velocity = body_state.angular_velocity;

    covariance.setZero();
    covariance.block(0, 0, 3, 3) = body_state.cov_position;
    covariance.block(3, 3, 3, 3) = body_state.cov_orientation;
    covariance.block(6, 6, 3, 3) = body_state.cov_velocity;
    covariance.block(9, 9, 3, 3) = body_state.cov_angular_velocity;
}

void PoseUKF::UKFStateToRigidBodyState(const PoseUKF::WPoseState& state, const ukfom::ukf< PoseUKF::WPoseState >::cov& covariance, base::samples::RigidBodyState& body_state)
{
    body_state.position = state.position;
    body_state.orientation = state.orientation;
    body_state.velocity = state.velocity;
    body_state.angular_velocity = state.angular_velocity;
    
    body_state.cov_position = covariance.block(0, 0, 3, 3);
    body_state.cov_orientation = covariance.block(3, 3, 3, 3);
    body_state.cov_velocity = covariance.block(6, 6, 3, 3);
    body_state.cov_angular_velocity = covariance.block(9, 9, 3, 3);
}

}