#include "OrientationUKF.hpp"
#include <base/Float.hpp>
#include <base/Logging.hpp>
#include <pose_estimation/EulerConversion.hpp>
#include <assert.h>

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

const std::string OrientationUKF::acceleration_measurement = "acceleration";
const std::string OrientationUKF::rotation_rate_measurement = "rotation_rate";
const std::string OrientationUKF::velocity_measurement = "velocity";

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
Eigen::Matrix<typename OrientationState::scalar, -1, 1>
velocityMeasurementModel ( const OrientationState &state )
{
    return state.orientation.inverse() * state.velocity;
}

OrientationUKF::OrientationUKF(const AbstractFilter::FilterState& initial_state, const OrientationUKFConfig& config) : config(config)
{
    setInitialState(initial_state);
    updateFilterParamter();
}

void OrientationUKF::predictionStep(const double delta)
{
    std::map<std::string, pose_estimation::Measurement>::const_iterator acceleration = latest_measurements.find(acceleration_measurement);
    std::map<std::string, pose_estimation::Measurement>::const_iterator rotation_rate = latest_measurements.find(rotation_rate_measurement);

    if(acceleration == latest_measurements.end())
    {
        LOG_ERROR_S << "No acceleration measurement available! Skipping prediction step.";
        return;
    }

    if(rotation_rate == latest_measurements.end())
    {
        LOG_ERROR_S << "No angular velocity measurement available! Skipping prediction step.";
        return;
    }

    MTK_UKF::cov process_noise = process_noise_cov * delta; // might be pow(delta,2), depending on sensor spec.

    //process_noise.block(6,6,3,3) = 2.0 * it1->second.cov;
    // need to do uncertainty matrix calculations
    ukf->predict(boost::bind(processModel<WState>, _1, acceleration->second.mu, rotation_rate->second.mu, delta, config, earth_rotation, gravity), MTK_UKF::cov(process_noise));

}

void OrientationUKF::setFilterConfiguration(const OrientationUKFConfig& config)
{
    this->config = config;
    updateFilterParamter();
}

void OrientationUKF::correctionStepUser(const pose_estimation::Measurement& measurement)
{
    if(measurement.measurement_name == acceleration_measurement)
        latest_measurements[measurement.measurement_name] = measurement;
    else if(measurement.measurement_name == rotation_rate_measurement)
        latest_measurements[measurement.measurement_name] = measurement;
    else if(measurement.measurement_name == velocity_measurement)
    {
        // handle velocity measurement
        ukf->update(measurement.mu, boost::bind(velocityMeasurementModel<State>, _1),
                        boost::bind(ukfom::id< Eigen::MatrixXd >, measurement.cov),
                        allowed_distance);
    }
    else
        LOG_ERROR_S << "Measurement " << measurement.measurement_name << " is not supported by the Orientation filter.";
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

void OrientationUKF::muToUKFState(const FilterState::Mu& mu, WState& state) const
{
    assert(mu.rows() >= WState::DOF);

    base::Orientation orientation;
    Eigen::Vector3d euler = mu.block(0, 0, 3, 1);
    EulerConversion::eulerToQuad(euler, orientation);
    state.orientation = MTK::SO3<double>(orientation);
    state.velocity = mu.block(3, 0, 3, 1);
    state.bias_gyro = mu.block(6, 0, 3, 1);
    state.bias_acc = mu.block(9, 0, 3, 1);
}

void OrientationUKF::UKFStateToMu(const WState& state, FilterState::Mu& mu) const
{
    mu.resize(WState::DOF);
    mu.setZero();

    Eigen::Vector3d euler;
    EulerConversion::quadToEuler(state.orientation, euler);
    mu.block(0, 0, 3, 1) = euler;
    mu.block(3, 0, 3, 1) = state.velocity;
    mu.block(6, 0, 3, 1) = state.bias_gyro;
    mu.block(9, 0, 3, 1) = state.bias_acc;        
}

}
