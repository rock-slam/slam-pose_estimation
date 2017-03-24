#include "OrientationUKF.hpp"
#include <base/Float.hpp>
#include <base-logging/Logging.hpp>
#include "../GravityModel.hpp"


namespace pose_estimation
{

/** Process model for the robot orientation
 */
template <typename OrientationState>
OrientationState
processModel (const OrientationState &state, const Eigen::Vector3d& acc, const Eigen::Vector3d& omega,
              double gyro_bias_tau, double acc_bias_tau, double delta_time)
{
    OrientationState new_state(state);
    Eigen::Vector3d angular_velocity = state.orientation * (omega - state.bias_gyro) - Eigen::Vector3d(EARTHW * cos(state.latitude(0)), 0., EARTHW * sin(state.latitude(0)));
    new_state.orientation.boxplus(angular_velocity, delta_time);
    
    Eigen::Vector3d acceleration = state.orientation * (acc - state.bias_acc) - Eigen::Vector3d(0., 0., state.gravity(0));
    new_state.velocity.boxplus(acceleration, delta_time);
    
    Eigen::Vector3d gyro_bias_delta = (-1.0/gyro_bias_tau) * state.bias_gyro;
    new_state.bias_gyro.boxplus(gyro_bias_delta, delta_time);

    Eigen::Vector3d acc_bias_delta = (-1.0/acc_bias_tau) * state.bias_acc;
    new_state.bias_acc.boxplus(acc_bias_delta, delta_time);

    Eigen::Matrix<double, 1, 1> latitude_delta;
    latitude_delta << atan2(state.velocity(0) * delta_time, EQUATORIAL_RADIUS);
    new_state.latitude.boxplus(latitude_delta);
    
    return new_state;
}

template <typename OrientationState>
VelocityType
velocityMeasurementModel ( const OrientationState &state )
{
    return VelocityType(state.orientation.inverse() * state.velocity);
}

OrientationUKF::OrientationUKF(const State& initial_state, const Covariance& state_cov,
                               double gyro_bias_tau, double acc_bias_tau) :
                                    gyro_bias_tau(gyro_bias_tau), acc_bias_tau(acc_bias_tau)
{
    initializeFilter(initial_state, state_cov);

    // initialize rotation rate and acceleration in IMU with zero
    rotation_rate.mu = RotationRate::Mu::Zero();
    acceleration.mu = initial_state.orientation.inverse() * Acceleration::Mu(0., 0., initial_state.gravity(0));
}

void OrientationUKF::integrateMeasurement(const RotationRate& measurement)
{
    checkMeasurment(measurement.mu, measurement.cov);
    rotation_rate = measurement;
}

void OrientationUKF::integrateMeasurement(const Acceleration& measurement)
{
    checkMeasurment(measurement.mu, measurement.cov);
    acceleration = measurement;
}

void OrientationUKF::integrateMeasurement(const VelocityMeasurement& measurement)
{
    checkMeasurment(measurement.mu, measurement.cov);
    // handle velocity measurement
    ukf->update(measurement.mu, boost::bind(velocityMeasurementModel<State>, _1),
                    boost::bind(ukfom::id< VelocityMeasurement::Cov >, measurement.cov),
                    ukfom::accept_any_mahalanobis_distance<WState::scalar>);
}

OrientationUKF::RotationRate::Mu OrientationUKF::getRotationRate()
{
    return rotation_rate.mu - ukf->mu().bias_gyro - ukf->mu().orientation.inverse() * Eigen::Vector3d(EARTHW * cos(ukf->mu().latitude(0)), 0., EARTHW * sin(ukf->mu().latitude(0)));
}

void OrientationUKF::predictionStepImpl(double delta)
{
    Eigen::Matrix3d rot = ukf->mu().orientation.matrix();
    Covariance process_noise = process_noise_cov;
    // uncertainty matrix calculations
    MTK::subblock(process_noise, &State::orientation) = rot * MTK::subblock(process_noise_cov, &State::orientation) * rot.transpose();
    MTK::subblock(process_noise, &State::velocity) = rot * MTK::subblock(process_noise_cov, &State::velocity) * rot.transpose();
    process_noise = pow(delta, 2.) * process_noise;

    ukf->predict(boost::bind(processModel<WState>, _1, acceleration.mu, rotation_rate.mu, gyro_bias_tau, acc_bias_tau, delta), process_noise);
}

}