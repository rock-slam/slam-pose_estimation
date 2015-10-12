#include "PoseUKF.hpp"
#include <pose_estimation/mtk_ukf/ProcessModels.hpp>
#include <pose_estimation/mtk_ukf/MeasurementModels.hpp>
#include <base/Float.hpp>
#include <base/Logging.hpp>

namespace pose_estimation
{

PoseUKF::PoseUKF() : dirty(true)
{
    body_state.initUnknown();

    last_acceleration_sample.time.microseconds = 0;

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

    if(last_acceleration_sample.time.isNull())
        ukf->predict(boost::bind(processModel<WPoseState>, _1 , delta), MTK_UKF::cov(delta * process_noise_cov));
    else
    {
        Covariance process_noise = process_noise_cov;
        process_noise.block(6,6,3,3) = 2.0 * last_acceleration_sample.cov_acceleration;
        ukf->predict(boost::bind(processModelWithAcceleration<WPoseState>, _1, last_acceleration_sample.acceleration, delta), MTK_UKF::cov(delta * process_noise));
    }
}

void PoseUKF::correctionStep(const Measurement& measurement)
{
    if(ukf.use_count() == 0)
    {
	setInitialState(body_state);
    }

    // handle acc measurements
    if(measurement.hasAccelerationMeasurement())
    {
        const Measurement::MemberMask &mask = measurement.member_mask;
        if(mask[BodyStateMemberAx] > 0 && mask[BodyStateMemberAy] > 0 && mask[BodyStateMemberAz] > 0)
        {
            if(base::isnotnan(measurement.acceleration.cov_acceleration) && base::isnotnan(measurement.acceleration.acceleration))
            {
                last_acceleration_sample = measurement.acceleration;
                last_acceleration_sample.time = measurement.time;
            }
            else
                LOG_ERROR("Acceleration covariance or acceleration sample contains NaN values!");
        }
        else
        {
            LOG_ERROR("Cannot handle partial selected acceleration samples");
        }
    }

    // handle body state measurements
    if(measurement.hasPositionMeasurement() || measurement.hasOrientationMeasurement() || measurement.hasVelocityMeasurement() || measurement.hasAngularVelocityMeasurement())
    {
        dirty = true;

        WPoseState state;
        Covariance cov;
        rigidBodyStateToUKFState(measurement.body_state, state, cov);

        // check state for NaN values
        Eigen::Matrix<unsigned, BODY_STATE_SIZE, 1> mask = measurement.member_mask.block(0,0,BODY_STATE_SIZE,1);
        Eigen::Matrix<WPoseState::scalar_type, WPoseState::DOF, 1> state_vector = state.getStateVector();
        for(unsigned i = 0; i < WPoseState::DOF; i++)
        {
            if(mask(i,0) != 0 && base::isNaN<WPoseState::scalar_type>(state_vector(i,0)))
            {
                // handle NaN values in state
                LOG_ERROR("State contains NaN values! This Measurement will be excluded from correction step.");
                mask(i,0) = 0;
            }
        }

        // check covariance matrix for NaN values
        for(unsigned i = 0; i < WPoseState::DOF; i++)
            for(unsigned j = 0; j < WPoseState::DOF; j++)
            {
                if(mask(i,0) != 0 && mask(j,0) != 0 && base::isNaN<Covariance::Scalar>(cov(i,j)))
                {
                    // handle NaN variances
                    LOG_ERROR("Covariance contains NaN values! This Measurement will be skipped.");
                    return;
                }
                else if(i==j && cov(i,j) == 0.0)
                {
                    // handle zero variances
                    LOG_WARN("Covariance diagonal contains zero values. Override them with %d", 1e-9);
                    cov(i,j) = 1e-9;
                }
            }

        Eigen::Matrix<WPoseState::scalar, -1, 1> sub_state = state.getSubStateVector(mask);
        unsigned measurement_size = sub_state.rows();

        std::vector<unsigned> m_index2mask_index;
        for(unsigned i = 0; i < WPoseState::DOF; i++)
        {
            if(mask(i) > 0)
            {
                m_index2mask_index.push_back(i);
            }
        }
        Eigen::Matrix<WPoseState::scalar, -1, -1> sub_cov(measurement_size, measurement_size);
        sub_cov.setZero();
        for(unsigned i = 0; i < measurement_size; i++)
        {
            for(unsigned j = 0; j < measurement_size; j++)
            {
                sub_cov(i,j) = cov(m_index2mask_index[i], m_index2mask_index[j]);
            }
        }

        // apply new measurement
        ukf->update(sub_state, boost::bind(measurementModel<WPoseState>, _1, mask),
                    boost::bind(ukfom::id< Eigen::MatrixXd >, sub_cov),
                    ukfom::accept_any_mahalanobis_distance<MTK_UKF::scalar_type>);
    }
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
    body_state.velocity = body_state.orientation * state.velocity;
    body_state.angular_velocity = state.angular_velocity;
    
    body_state.cov_position = covariance.block(0, 0, 3, 3);
    body_state.cov_orientation = covariance.block(3, 3, 3, 3);
    body_state.cov_velocity = covariance.block(6, 6, 3, 3);
    body_state.cov_angular_velocity = covariance.block(9, 9, 3, 3);
}

}
