#include "PoseUKF.hpp"
#include <base/Float.hpp>

namespace pose_estimation
{

template <typename PoseWithVelocityType>
TranslationType
measurementPosition(const PoseWithVelocityType &state)
{
    return state.position;
}

template <typename PoseWithVelocityType>
Eigen::Matrix<TranslationType::scalar, 2, 1>
measurementXYPosition(const PoseWithVelocityType &state)
{
    return state.position.block(0,0,2,1);
}

template <typename PoseWithVelocityType>
Eigen::Matrix<TranslationType::scalar, 1, 1>
measurementZPosition(const PoseWithVelocityType &state)
{
    return state.position.block(2,0,1,1);
}

template <typename PoseWithVelocityType>
RotationType
measurementOrientation(const PoseWithVelocityType &state)
{
    return state.orientation;
}

template <typename PoseWithVelocityType>
VelocityType
measurementVelocity(const PoseWithVelocityType &state)
{
    return state.velocity;
}

template <typename PoseWithVelocityType>
Eigen::Matrix<VelocityType::scalar, 2, 1>
measurementXYVelocity(const PoseWithVelocityType &state)
{
    return state.velocity.block(0,0,2,1);
}

template <typename PoseWithVelocityType>
Eigen::Matrix<VelocityType::scalar, 1, 1>
measurementZVelocity(const PoseWithVelocityType &state)
{
    return state.velocity.block(2,0,1,1);
}

template <typename PoseWithVelocityType>
Eigen::Matrix<VelocityType::scalar, 2, 1>
measurementXVelYawVel(const PoseWithVelocityType &state)
{
    Eigen::Matrix<VelocityType::scalar, 2, 1> substate(state.velocity.x(), state.angular_velocity.z());
    return substate;
}

template <typename PoseWithVelocityType>
VelocityType
measurementAngularVelocity(const PoseWithVelocityType &state)
{
    return state.angular_velocity;
}


/** Process model for the 12D robot state.
 * Applies the current velocity to update the robot pose.
 */
template <typename PoseWithVelocityType>
PoseWithVelocityType
processModel (const PoseWithVelocityType &state, double delta_time)
{
    PoseWithVelocityType new_state(state);
    new_state.position.boxplus(new_state.orientation * new_state.velocity, delta_time);
    new_state.orientation.boxplus(new_state.orientation * new_state.angular_velocity, delta_time);
    return new_state;
}

/** Process model with acceleration for the 12D robot state.
 * Applies the current velocity and acceleration to update the robot pose.
 */
template <typename PoseWithVelocityType>
PoseWithVelocityType
processModelWithAcceleration (const PoseWithVelocityType &state, const Eigen::Vector3d& acc, double delta_time)
{
    PoseWithVelocityType new_state(state);
    new_state.velocity.boxplus(acc, delta_time);
    new_state.position.boxplus(new_state.orientation * new_state.velocity, delta_time);
    new_state.orientation.boxplus(new_state.orientation * new_state.angular_velocity, delta_time);
    return new_state;
}

PoseUKF::PoseUKF(const State& initial_state, const Covariance& state_cov) : UnscentedKalmanFilter<pose_estimation::PoseWithVelocity>()
{
    initializeFilter(initial_state, state_cov);

    process_noise_cov = MTK_UKF::cov::Zero();
    MTK::setDiagonal(process_noise_cov, &WState::position, 0.01);
    MTK::setDiagonal(process_noise_cov, &WState::orientation, 0.001);
    MTK::setDiagonal(process_noise_cov, &WState::velocity, 0.00001);
    MTK::setDiagonal(process_noise_cov, &WState::angular_velocity, 0.00001);

    acceleration.mu = base::NaN<double>() * AccelerationMeasurement::Mu::Ones();
}

void PoseUKF::integrateMeasurement(const PositionMeasurement& measurement)
{
    ukf->update(measurement.mu, boost::bind(measurementPosition<WState>, _1),
                boost::bind(ukfom::id< PositionMeasurement::Cov >, measurement.cov),
                ukfom::accept_any_mahalanobis_distance<WState::scalar>);
}

void PoseUKF::integrateMeasurement(const XYMeasurement& measurement)
{
    ukf->update(measurement.mu, boost::bind(measurementXYPosition<WState>, _1),
                boost::bind(ukfom::id< XYMeasurement::Cov >, measurement.cov),
                ukfom::accept_any_mahalanobis_distance<WState::scalar>);
}

void PoseUKF::integrateMeasurement(const ZMeasurement& measurement)
{
    ukf->update(measurement.mu, boost::bind(measurementZPosition<WState>, _1),
                boost::bind(ukfom::id< ZMeasurement::Cov >, measurement.cov),
                ukfom::accept_any_mahalanobis_distance<WState::scalar>);
}

void PoseUKF::integrateMeasurement(const OrientationMeasurement& measurement)
{
    ukf->update(RotationType(MTK::SO3<double>::exp(measurement.mu)), boost::bind(measurementOrientation<WState>, _1),
                boost::bind(ukfom::id< OrientationMeasurement::Cov >, measurement.cov),
                ukfom::accept_any_mahalanobis_distance<WState::scalar>);
}

void PoseUKF::integrateMeasurement(const VelocityMeasurement& measurement)
{
    ukf->update(measurement.mu, boost::bind(measurementVelocity<WState>, _1),
                boost::bind(ukfom::id< VelocityMeasurement::Cov >, measurement.cov),
                ukfom::accept_any_mahalanobis_distance<WState::scalar>);
}

void PoseUKF::integrateMeasurement(const XYVelocityMeasurement& measurement)
{
    ukf->update(measurement.mu, boost::bind(measurementXYVelocity<WState>, _1),
                boost::bind(ukfom::id< XYVelocityMeasurement::Cov >, measurement.cov),
                ukfom::accept_any_mahalanobis_distance<WState::scalar>);
}

void PoseUKF::integrateMeasurement(const ZVelocityMeasurement& measurement)
{
    ukf->update(measurement.mu, boost::bind(measurementZVelocity<WState>, _1),
                boost::bind(ukfom::id< ZVelocityMeasurement::Cov >, measurement.cov),
                ukfom::accept_any_mahalanobis_distance<WState::scalar>);
}

void PoseUKF::integrateMeasurement(const XVelYawVelMeasurement& measurement)
{
    ukf->update(measurement.mu, boost::bind(measurementXVelYawVel<WState>, _1),
                boost::bind(ukfom::id< XVelYawVelMeasurement::Cov >, measurement.cov),
                ukfom::accept_any_mahalanobis_distance<WState::scalar>);
}

void PoseUKF::integrateMeasurement(const AngularVelocityMeasurement& measurement)
{
    ukf->update(measurement.mu, boost::bind(measurementAngularVelocity<WState>, _1),
                boost::bind(ukfom::id< AngularVelocityMeasurement::Cov >, measurement.cov),
                ukfom::accept_any_mahalanobis_distance<WState::scalar>);
}

void PoseUKF::integrateMeasurement(const AccelerationMeasurement& measurement)
{
    acceleration = measurement;
}

void PoseUKF::predictionStepImpl(const double delta)
{
    Eigen::Matrix3d rot = ukf->mu().orientation.matrix();
    MTK_UKF::cov process_noise = process_noise_cov;
    MTK::subblock(process_noise, &State::position) = rot * MTK::subblock(process_noise_cov, &State::position) * rot.transpose();
    MTK::subblock(process_noise, &State::orientation) = rot * MTK::subblock(process_noise_cov, &State::orientation) * rot.transpose();
    process_noise = delta * process_noise;

    if(acceleration.mu.allFinite())
    {
        MTK_UKF::cov process_noise = process_noise_cov;
        process_noise.block(6,6,3,3) = 2.0 * acceleration.cov;
        ukf->predict(boost::bind(processModelWithAcceleration<WState>, _1, acceleration.mu, delta), process_noise);
    }
    else
        ukf->predict(boost::bind(processModel<WState>, _1 , delta), process_noise);
}

}