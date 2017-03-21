#ifndef _POSE_ESTIMATION_ORIENTATION_UKF_HPP
#define _POSE_ESTIMATION_ORIENTATION_UKF_HPP

#include "OrientationState.hpp"
#include "OrientationUKFConfig.hpp"
#include <pose_estimation/Measurement.hpp>
#include <pose_estimation/UnscentedKalmanFilter.hpp>
#include <Eigen/Core>

namespace pose_estimation
{

class OrientationUKF : public UnscentedKalmanFilter<OrientationState>
{
public:
    MEASUREMENT(RotationRate, 3)
    MEASUREMENT(Acceleration, 3)
    MEASUREMENT(VelocityMeasurement, 3)

public:
    OrientationUKF(const State& initial_state, const Covariance& state_cov,
                   double gyro_bias_tau, double acc_bias_tau, const LocationConfiguration& location);
    virtual ~OrientationUKF() {}

    void integrateMeasurement(const RotationRate& measurement);
    void integrateMeasurement(const Acceleration& measurement);
    void integrateMeasurement(const VelocityMeasurement& measurement);

    /* Returns unbiased rotation rate in IMU frame */
    RotationRate::Mu getRotationRate();
    
protected:
    void predictionStepImpl(double delta);

protected:
    RotationRate rotation_rate;
    Acceleration acceleration;
    Eigen::Vector3d earth_rotation;
    Eigen::Vector3d gravity;
    double gyro_bias_tau;
    double acc_bias_tau;
};

}

#endif