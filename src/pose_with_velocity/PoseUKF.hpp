#ifndef _POSE_ESTIMATION_POSE_UKF_HPP
#define _POSE_ESTIMATION_POSE_UKF_HPP

#include "PoseWithVelocity.hpp"
#include "PoseUKFConfig.hpp"
#include <pose_estimation/Measurement.hpp>
#include <pose_estimation/UnscentedKalmanFilter.hpp>

namespace pose_estimation
{

class PoseUKF : public UnscentedKalmanFilter<PoseWithVelocity>
{
public:
    MEASUREMENT(PositionMeasurement, 3)
    MEASUREMENT(XYMeasurement, 2)
    MEASUREMENT(ZMeasurement, 1)
    MEASUREMENT(OrientationMeasurement, 3)
    MEASUREMENT(VelocityMeasurement, 3)
    MEASUREMENT(XYVelocityMeasurement, 2)
    MEASUREMENT(ZVelocityMeasurement, 1)
    MEASUREMENT(XVelYawVelMeasurement, 2)
    MEASUREMENT(AngularVelocityMeasurement, 3)
    MEASUREMENT(AccelerationMeasurement, 3)

public:
    PoseUKF(const State& initial_state, const Covariance& state_cov, const VelocityBiasConfig& bias_config);
    virtual ~PoseUKF() {}

    void integrateMeasurement(const PositionMeasurement& measurement);
    void integrateMeasurement(const XYMeasurement& measurement);
    void integrateMeasurement(const ZMeasurement& measurement);
    void integrateMeasurement(const OrientationMeasurement& measurement);
    void integrateMeasurement(const VelocityMeasurement& measurement, bool velocity_bias = true);
    void integrateMeasurement(const XYVelocityMeasurement& measurement);
    void integrateMeasurement(const ZVelocityMeasurement& measurement);
    void integrateMeasurement(const XVelYawVelMeasurement& measurement);
    void integrateMeasurement(const AngularVelocityMeasurement& measurement);
    void integrateMeasurement(const AccelerationMeasurement& measurement);

protected:
    virtual void predictionStepImpl(const double delta);

protected:
    AccelerationMeasurement acceleration;
    VelocityBiasConfig velocity_bias_config;
};

}

#endif
