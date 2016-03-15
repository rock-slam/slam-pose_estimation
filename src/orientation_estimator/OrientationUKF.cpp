#include "OrientationUKF.hpp"
#include <base/Float.hpp>
#include <base/Logging.hpp>
#include <pose_estimation/EulerConversion.hpp>
#include <assert.h>

// required config
// gyro_bias_tau
// acc_bias_tau
// gyro_bias_std
// acc_bias_std

/** WGS-84 ellipsoid constants (Nominal Gravity Model and Earth angular velocity) **/
#ifndef Re
#define Re	6378137 /**< Equatorial radius in meters **/
#endif
#ifndef Rp
#define Rp	6378137 /**< Polar radius in meters **/
#endif
#ifndef ECC
#define ECC  0.0818191908426 /**< First eccentricity **/
#endif
#ifndef GRAVITY
#define GRAVITY 9.79766542 /**< Mean value of gravity value in m/s^2 **/
#endif
#ifndef GWGS0
#define GWGS0 9.7803267714 /**< Gravity value at the equator in m/s^2 **/
#endif
#ifndef GWGS1
#define GWGS1 0.00193185138639 /**< Gravity formula constant **/
#endif
#ifndef EARTHW
#define EARTHW  7.292115e-05 /**< Earth angular velocity in rad/s **/
#endif

namespace orientation
{

/** Process model with acceleration for the 12D robot state.
 * Applies the current velocity and acceleration to update the robot pose.
 */
template <typename OrientationState>
OrientationState
processModel (const OrientationState &state, const Eigen::Vector3d& acc, const Eigen::Vector3d& omega, double delta_time, const OrientationUKF::ParamsType &params)
{
    OrientationState new_state(state);
    Eigen::Vector3d angular_velocity = omega - new_state.bias_gyro - new_state.orientation.inverse()*params.Earth_rotation;
    new_state.orientation.boxplus(angular_velocity, delta_time);
    
    Eigen::Vector3d velocity = new_state.orientation*(acc - new_state.bias_acc) - params.Gravity;
    new_state.velocity.boxplus(velocity, delta_time);
    
    Eigen::Vector3d gyro_bias_delta = -1.0/params.gyro_bias_tau * new_state.bias_gyro;
    new_state.bias_gyro.boxplus(gyro_bias_delta, delta_time);
    
    Eigen::Vector3d acc_bias_delta = -1.0/params.acc_bias_tau * new_state.bias_acc;
    new_state.bias_acc.boxplus(acc_bias_delta, delta_time);
    
    return new_state;
}

OrientationUKF::OrientationUKF(const AbstractFilter::FilterState& initial_state)
{
    setInitialState(initial_state);

    params.gyro_bias_var = (2.0 *pow(params.gyro_bias_std,2) / params.gyro_bias_tau) * 0.01;
    params.acc_bias_var = (2.0 *pow(params.acc_bias_std,2) / params.acc_bias_tau) * 0.01;
    
    process_noise_cov = MTK_UKF::cov::Zero();
    MTK::setDiagonal(process_noise_cov, &WState::orientation, params.gyro_var);
    MTK::setDiagonal(process_noise_cov, &WState::velocity, params.acc_var);
    MTK::setDiagonal(process_noise_cov, &WState::bias_gyro, params.gyro_bias_var);
    MTK::setDiagonal(process_noise_cov, &WState::bias_acc, params.acc_bias_var);
    
    params.Earth_rotation[0] = cos(params.Latitude)*EARTHW;
    params.Earth_rotation[1] = 0.0;
    params.Earth_rotation[2] = -sin(params.Latitude)*EARTHW;
    
    /** Gravity affects by the altitude (aprox the value r = Re **/
    //g = g*pow(Re/(Re+altitude), 2); //assume zero altitude for now
    
    params.Gravity[0] = 0.0;
    params.Gravity[1] = 0.0;
    params.Gravity[2] = GWGS0*((1+GWGS1*pow(sin(params.Latitude),2))/sqrt(1-pow(ECC,2)*pow(sin(params.Latitude),2)));;
    
}

void OrientationUKF::predictionStep(const double delta)
{
    std::map<std::string, pose_estimation::Measurement>::const_iterator it1 = latest_measurements.find("acceleration");
    std::map<std::string, pose_estimation::Measurement>::const_iterator it2 = latest_measurements.find("rotation_rate");

    MTK_UKF::cov process_noise = process_noise_cov;
    
    params.gyro_bias_var = (2.0 * pow(params.gyro_bias_std,2) / params.gyro_bias_tau)* delta;
    params.acc_bias_var = (2.0 * pow(params.acc_bias_std,2) / params.acc_bias_tau)* delta;
    
    MTK::setDiagonal(process_noise_cov, &WState::bias_gyro, params.gyro_bias_var);
    MTK::setDiagonal(process_noise_cov, &WState::bias_acc, params.acc_bias_var);
    
    MTK::setDiagonal(process_noise_cov, &WState::orientation, params.gyro_var * delta); // might be pow(delta,2), depending on sensor spec. 
    MTK::setDiagonal(process_noise_cov, &WState::velocity, params.acc_var * delta);
    
    //process_noise.block(6,6,3,3) = 2.0 * it1->second.cov; 
    // need to do uncertainty matrix calculations
    ukf->predict(boost::bind(processModel<WState>, _1, it1->second.mu, it2->second.mu, delta, params), MTK_UKF::cov(process_noise));

}

void OrientationUKF::correctionStepUser(const pose_estimation::Measurement& measurement)
{
    if(measurement.measurement_name == "acceleration")
        latest_measurements[measurement.measurement_name] = measurement;
    else
        LOG_ERROR_S << "Measurement " << measurement.measurement_name << " is not supported by the Orientation filter.";
}

void OrientationUKF::muToUKFState(const FilterState::Mu& mu, WState& state) const
{
    assert(mu.rows() >= WState::DOF);

    //state.position = mu.block(0, 0, 3, 1);
    base::Orientation orientation;
    Eigen::Vector3d euler = mu.block(0, 0, 3, 1);
    EulerConversion::eulerToQuad(euler, orientation);
    state.orientation = MTK::SO3<double>(orientation);
    state.velocity = mu.block(3, 0, 3, 1);
    state.bias_gyro = mu.block(6, 0, 3, 1);
    state.bias_acc = mu.block(9, 0, 3, 1);
    //Eigen::Vector3d angle_axis;
    //Eigen::Vector3d euler_velocity = mu.block(9, 0, 3, 1);
    //EulerConversion::eulerAngleVelocityToAngleAxis(euler_velocity, angle_axis);
    //state.angular_velocity = angle_axis;

}

void OrientationUKF::UKFStateToMu(const WState& state, FilterState::Mu& mu) const
{
    mu.resize(WState::DOF);
    mu.setZero();

    //mu.block(0, 0, 3, 1) = state.position;
    Eigen::Vector3d euler;
    base::Orientation orientation = state.orientation;
    EulerConversion::quadToEuler(orientation, euler);
    mu.block(0, 0, 3, 1) = euler;
    mu.block(3, 0, 3, 1) = state.velocity;
    mu.block(6, 0, 3, 1) = state.bias_gyro;
    mu.block(9, 0, 3, 1) = state.bias_acc;        
    //Eigen::Vector3d euler_velocity;
    //Eigen::Vector3d angular_velocity = state.angular_velocity;
    //EulerConversion::angleAxisToEulerAngleVelocity(angular_velocity, euler_velocity);
    //mu.block(9, 0, 3, 1) = euler_velocity;
}

    void OrientationUKF::setLatitude(const double latitude)
    {
      params.Latitude = latitude;
    }
    
    void OrientationUKF::setIMUParams(const double gyro_var, const double acc_var, const double gyro_bias_std, 
				   const double acc_bias_std, const double gyro_bias_tau, const double acc_bias_tau)
    {
      params.gyro_var = gyro_var;
      params.acc_var = acc_var;
      params.gyro_bias_std = gyro_bias_std;
      params.acc_bias_std = acc_bias_std;
      params.gyro_bias_tau = gyro_bias_tau;
      params.acc_bias_tau = acc_bias_tau;
    }

}
