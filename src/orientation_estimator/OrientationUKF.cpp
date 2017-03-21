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
              double gyro_bias_tau, double acc_bias_tau, const Eigen::Vector3d& earth_rotation,
              const Eigen::Vector3d& gravity, double delta_time)
{
    OrientationState new_state(state);
    Eigen::Vector3d angular_velocity = new_state.orientation * (omega - new_state.bias_gyro) - earth_rotation;
    new_state.orientation.boxplus(angular_velocity, delta_time);
    
    Eigen::Vector3d acceleration = new_state.orientation * (acc - new_state.bias_acc) - gravity;
    new_state.velocity.boxplus(acceleration, delta_time);
    
    Eigen::Vector3d gyro_bias_delta = (-1.0/gyro_bias_tau) * new_state.bias_gyro;
    new_state.bias_gyro.boxplus(gyro_bias_delta, delta_time);

    Eigen::Vector3d acc_bias_delta = (-1.0/acc_bias_tau) * new_state.bias_acc;
    new_state.bias_acc.boxplus(acc_bias_delta, delta_time);
    
    return new_state;
}

template <typename OrientationState>
VelocityType
velocityMeasurementModel ( const OrientationState &state )
{
    return VelocityType(state.orientation.inverse() * state.velocity);
}

OrientationUKF::OrientationUKF(const State& initial_state, const Covariance& state_cov,
                               double gyro_bias_tau, double acc_bias_tau, const LocationConfiguration& location) :
                                    gyro_bias_tau(gyro_bias_tau), acc_bias_tau(acc_bias_tau)
{
    initializeFilter(initial_state, state_cov);

    earth_rotation = Eigen::Vector3d(EARTHW * cos(location.latitude), 0., EARTHW * sin(location.latitude));
    gravity = Eigen::Vector3d(0., 0., GravityModel(location.latitude, location.altitude));

    rotation_rate.mu = RotationRate::Mu::Zero();
    acceleration.mu = gravity;
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
    return rotation_rate.mu - ukf->mu().bias_gyro - ukf->mu().orientation.inverse() * earth_rotation;
}

void OrientationUKF::predictionStepImpl(double delta)
{
    Eigen::Matrix3d rot = ukf->mu().orientation.matrix();
    Covariance process_noise = Covariance::Zero();
    // uncertainty matrix calculations
    process_noise.block(0,0,3,3) = rot * process_noise_cov.block(0,0,3,3) * rot.transpose();
    process_noise.block(3,3,3,3) = rot * process_noise_cov.block(3,3,3,3) * rot.transpose();
    process_noise.block(6,6,3,3) = process_noise_cov.block(6,6,3,3);
    process_noise.block(9,9,3,3) = process_noise_cov.block(9,9,3,3);
    process_noise = pow(delta, 2.) * process_noise;

    ukf->predict(boost::bind(processModel<WState>, _1, acceleration.mu, rotation_rate.mu, gyro_bias_tau, acc_bias_tau, earth_rotation, gravity, delta), process_noise);
}

}