#include "OrientationUKF.hpp"
#include <base/Float.hpp>
#include <base/Logging.hpp>

/** WGS-84 ellipsoid constants (Nominal Gravity Model and Earth angular velocity) **/
static const double EQUATORIAL_RADIUS = 6378137.0; /** Equatorial radius in meters **/
static const double ECC = 0.0818191908426; /** First eccentricity **/
static const double GRAVITY = 9.79766542; /** Mean value of gravity value in m/s^2 according to WGS-84 **/
static const double GRAVITY_SI = 9.80665; /** Mean value of gravity value in m/s^2 according to SI standard **/
static const double GWGS0 = 9.7803267714; /** Gravity value at the equator in m/s^2 **/
static const double GWGS1 = 0.00193185138639; /** Gravity formula constant **/
static const double EARTHW = ((2.0*M_PI)/86164.0); //7.292115e-05; /** Earth angular velocity in rad/s **/

namespace pose_estimation
{

/** Process model for the robot orientation
 */
template <typename OrientationState>
OrientationState
processModel (const OrientationState &state, const Eigen::Vector3d& acc, const Eigen::Vector3d& omega, double delta_time,
              const OrientationUKFConfig &config, const Eigen::Vector3d& earth_rotation, const Eigen::Vector3d& gravity)
{
    OrientationState new_state(state);
    Eigen::Vector3d angular_velocity = omega - new_state.bias_gyro - new_state.orientation.inverse()*earth_rotation;
    new_state.orientation.boxplus(angular_velocity, delta_time);
    
    Eigen::Vector3d velocity = new_state.orientation*(acc - new_state.bias_acc) - gravity;
    new_state.velocity.boxplus(velocity, delta_time);
    
    Eigen::Vector3d gyro_bias_delta = (-1.0/config.gyro_bias_tau) * new_state.bias_gyro;
    new_state.bias_gyro.boxplus(gyro_bias_delta, delta_time);
    
    Eigen::Vector3d acc_bias_delta = (-1.0/config.acc_bias_tau) * new_state.bias_acc;
    new_state.bias_acc.boxplus(acc_bias_delta, delta_time);
    
    return new_state;
}

template <typename OrientationState>
VelocityType
velocityMeasurementModel ( const OrientationState &state )
{
    return VelocityType(state.orientation.inverse() * state.velocity);
}

OrientationUKF::OrientationUKF(const State& initial_state, const Covariance& state_cov, const OrientationUKFConfig& config) : config(config)
{
    initializeFilter(initial_state, state_cov);
    updateFilterParamter();
}

void OrientationUKF::setFilterConfiguration(const OrientationUKFConfig& config)
{
    this->config = config;
    updateFilterParamter();
}

void OrientationUKF::updateFilterParamter()
{
    double gyro_bias_var = (2.0 *pow(config.gyro_bias_std,2)) / config.gyro_bias_tau;
    double acc_bias_var = (2.0 *pow(config.acc_bias_std,2)) / config.acc_bias_tau;

    process_noise_cov = MTK_UKF::cov::Zero();
    MTK::setDiagonal(process_noise_cov, &WState::orientation, config.gyro_var);
    MTK::setDiagonal(process_noise_cov, &WState::velocity, config.acc_var);
    MTK::setDiagonal(process_noise_cov, &WState::bias_gyro, gyro_bias_var);
    MTK::setDiagonal(process_noise_cov, &WState::bias_acc, acc_bias_var);

    earth_rotation[0] = cos(config.latitude)*EARTHW;
    earth_rotation[1] = 0.0;
    earth_rotation[2] = sin(config.latitude)*EARTHW;

    /** Gravity affects by the altitude (aprox the value r = EQUATORIAL_RADIUS **/
    //g = g*pow(EQUATORIAL_RADIUS/(EQUATORIAL_RADIUS+altitude), 2); //assume zero altitude for now

    gravity[0] = 0.0;
    gravity[1] = 0.0;
    gravity[2] = GWGS0*((1+GWGS1*pow(sin(config.latitude),2))/sqrt(1-pow(ECC,2)*pow(sin(config.latitude),2)));
}

void OrientationUKF::integrateMeasurement(const RotationRate& measurement)
{
    rotation_rate = measurement;
}

void OrientationUKF::integrateMeasurement(const Acceleration& measurement)
{
    acceleration = measurement;
}

void OrientationUKF::integrateMeasurement(const VelocityMeasurement& measurement)
{
    // handle velocity measurement
    ukf->update(measurement.mu, boost::bind(velocityMeasurementModel<State>, _1),
                    boost::bind(ukfom::id< VelocityMeasurement::Cov >, measurement.cov),
                    ukfom::accept_any_mahalanobis_distance<WState::scalar>);
}

void OrientationUKF::predictionStepImpl(double delta)
{
    if(!acceleration.mu.allFinite())
    {
        LOG_ERROR_S << "No acceleration measurement available! Skipping prediction step.";
        return;
    }

    if(!rotation_rate.mu.allFinite())
    {
        LOG_ERROR_S << "No angular velocity measurement available! Skipping prediction step.";
        return;
    }

    Covariance process_noise = process_noise_cov * delta; // might be pow(delta,2), depending on sensor spec.

    //process_noise.block(6,6,3,3) = 2.0 * it1->second.cov;
    // need to do uncertainty matrix calculations
    ukf->predict(boost::bind(processModel<WState>, _1, acceleration.mu, rotation_rate.mu, delta, config, earth_rotation, gravity), process_noise);
}

}