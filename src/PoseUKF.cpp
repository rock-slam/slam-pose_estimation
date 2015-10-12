#include "PoseUKF.hpp"
#include <pose_estimation/mtk_ukf/ProcessModels.hpp>
#include <pose_estimation/mtk_ukf/MeasurementModels.hpp>
#include <base/Float.hpp>
#include <base/Logging.hpp>

namespace pose_estimation
{

PoseUKF::PoseUKF() : dirty(true)
{
    last_acceleration_sample.time.microseconds = 0;

    process_noise_cov = Covariance::Zero();
    MTK::setDiagonal(process_noise_cov, &WPoseState::position, 0.01);
    MTK::setDiagonal(process_noise_cov, &WPoseState::orientation, 0.001);
    MTK::setDiagonal(process_noise_cov, &WPoseState::velocity, 0.00001);
    MTK::setDiagonal(process_noise_cov, &WPoseState::angular_velocity, 0.00001);
}

void PoseUKF::setInitialState(const FilterState& initial_state)
{
    dirty = true;
    
    if(ukf.use_count() > 0)
    {
	ukf.reset();
    }

    WPoseState state;
    MTK_UKF::cov cov;
    assert(initial_state.mu.rows() == WPoseState::DOF);
    assert(cov.rows() == initial_state.cov.rows() && cov.cols() == initial_state.cov.cols());
    muToUKFState(initial_state.mu, state);
    cov = initial_state.cov;
    
    ukf.reset(new MTK_UKF(state, cov));
}

void PoseUKF::setProcessNoiseCovariance(const FilterState::Cov& noise_cov)
{
    assert(noise_cov.rows() == process_noise_cov.rows() && noise_cov.cols() == process_noise_cov.cols());
    process_noise_cov = noise_cov;
}

void PoseUKF::predictionStep(const double delta)
{
    if(ukf.use_count() == 0)
    {
        base::samples::RigidBodyState body_state;
        body_state.initUnknown();
 	AbstractRBSFilter::setInitialState(body_state);
    }
    dirty = true;

    if(last_acceleration_sample.time.isNull())
        ukf->predict(boost::bind(processModel<WPoseState>, _1 , delta), MTK_UKF::cov(delta * process_noise_cov));
    else
    {
        MTK_UKF::cov process_noise = process_noise_cov;
        process_noise.block(6,6,3,3) = 2.0 * last_acceleration_sample.cov_acceleration;
        ukf->predict(boost::bind(processModelWithAcceleration<WPoseState>, _1, last_acceleration_sample.acceleration, delta), MTK_UKF::cov(delta * process_noise));
    }
}

void PoseUKF::correctionStep(const Measurement& measurement)
{
    if(ukf.use_count() == 0)
    {
        base::samples::RigidBodyState body_state;
        body_state.initUnknown();
 	AbstractRBSFilter::setInitialState(body_state);
        return;
    }

    assert(measurement.mu.rows() == MEASUREMENT_SIZE);
    assert(measurement.cov.rows() == MEASUREMENT_SIZE && measurement.cov.cols() == MEASUREMENT_SIZE);
    assert(measurement.mask.rows() == MEASUREMENT_SIZE);

    // handle acc measurements
    const Measurement::StateMask &mask = measurement.mask;

    if(mask[BodyStateMemberAx] > 0 && mask[BodyStateMemberAy] > 0 && mask[BodyStateMemberAz] > 0)
    {
        Eigen::Vector3d acc = measurement.mu.block(12,0,3,1);
        Eigen::Matrix3d acc_cov = measurement.cov.block(12,12,3,3);
        if(base::isnotnan(acc_cov) && base::isnotnan(acc))
        {
            last_acceleration_sample.acceleration = acc;
            last_acceleration_sample.cov_acceleration = acc_cov;
            last_acceleration_sample.time = measurement.time;
        }
        else
            LOG_ERROR("Acceleration covariance or acceleration sample contains NaN values!");
    }

    // handle body state measurements
    unsigned body_state_members = 0;
    for(unsigned i = 0; i < WPoseState::DOF; i++)
        body_state_members += mask[i];
    if(body_state_members > 0)
    {
        dirty = true;

        WPoseState state;
        muToUKFState(measurement.mu, state);
        MTK_UKF::cov cov = measurement.cov.block(0,0,WPoseState::DOF,WPoseState::DOF);

        // check state for NaN values
        Eigen::Matrix<unsigned, WPoseState::DOF, 1> sub_mask = mask.block(0,0,WPoseState::DOF,1);
        Eigen::Matrix<WPoseState::scalar_type, WPoseState::DOF, 1> state_vector = state.getStateVector();
        for(unsigned i = 0; i < WPoseState::DOF; i++)
        {
            if(sub_mask(i,0) != 0 && base::isNaN<WPoseState::scalar_type>(state_vector(i,0)))
            {
                // handle NaN values in state
                LOG_ERROR("State contains NaN values! This Measurement will be excluded from correction step.");
                sub_mask(i,0) = 0;
            }
        }

        // check covariance matrix for NaN values
        for(unsigned i = 0; i < WPoseState::DOF; i++)
            for(unsigned j = 0; j < WPoseState::DOF; j++)
            {
                if(sub_mask(i,0) != 0 && sub_mask(j,0) != 0 && base::isNaN<Covariance::Scalar>(cov(i,j)))
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

        Eigen::Matrix<WPoseState::scalar, -1, 1> sub_state = state.getSubStateVector(sub_mask);
        unsigned measurement_size = sub_state.rows();

        std::vector<unsigned> m_index2mask_index;
        for(unsigned i = 0; i < WPoseState::DOF; i++)
        {
            if(sub_mask(i) > 0)
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
        ukf->update(sub_state, boost::bind(measurementModel<WPoseState>, _1, sub_mask),
                    boost::bind(ukfom::id< Eigen::MatrixXd >, sub_cov),
                    ukfom::accept_any_mahalanobis_distance<MTK_UKF::scalar_type>);
    }
}

const AbstractFilter::FilterState& PoseUKF::getCurrentState()
{
    if(dirty)
    {
        UKFStateToMu(ukf->mu(), state.mu);
        state.cov = ukf->sigma();

        dirty = false;
    }
    
    return state;
}

void PoseUKF::muToUKFState(const StateAndCovariance::Mu& mu, PoseUKF::WPoseState& state)
{
    assert(mu.rows() >= PoseUKF::WPoseState::DOF);

    state.position = mu.block(0, 0, 3, 1);
    base::Orientation orientation;
    Eigen::Vector3d euler = mu.block(3, 0, 3, 1);
    eulerToQuad(euler, orientation);
    state.orientation = MTK::SO3<double>(orientation);
    state.velocity = mu.block(6, 0, 3, 1);
    Eigen::Vector3d angle_axis;
    Eigen::Vector3d euler_velocity = mu.block(9, 0, 3, 1);
    eulerAngleVelocityToAngleAxis(euler_velocity, angle_axis);
    state.angular_velocity = angle_axis;

}

void PoseUKF::UKFStateToMu(const PoseUKF::WPoseState& state, StateAndCovariance::Mu& mu)
{
    mu.resize(PoseUKF::WPoseState::DOF);
    mu.setZero();

    mu.block(0, 0, 3, 1) = state.position;
    Eigen::Vector3d euler;
    base::Orientation orientation = state.orientation;
    quadToEuler(orientation, euler);
    mu.block(3, 0, 3, 1) = euler;
    mu.block(6, 0, 3, 1) = state.velocity;
    Eigen::Vector3d euler_velocity;
    Eigen::Vector3d angular_velocity = state.angular_velocity;
    angleAxisToEulerAngleVelocity(angular_velocity, euler_velocity);
    mu.block(9, 0, 3, 1) = euler_velocity;
}

}
