#ifndef _POSE_ESTIMATION_POSE_UKF_HPP
#define _POSE_ESTIMATION_POSE_UKF_HPP

#include "PoseWithVelocity.hpp"
#include <pose_estimation/Measurement.hpp>
#include <pose_estimation/UnscentedKalmanFilter.hpp>

namespace pose_estimation
{

/**
 * This filter integrates various linear and angular position and velocity measurements
 * into a 12D pose and velocity state. It is simple to use and doesn't need much configurations.
 * But since its model is limited to the propagation of accelerations and velocities into poses
 * its accuracy will be limited as well, however for a lot of cases sufficient.
 */
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
    PoseUKF(const State& initial_state, const Covariance& state_cov);
    virtual ~PoseUKF() {}

    /**
     * Integrates a 3D position measurement. Body in navigation frame in m.
     */
    void integrateMeasurement(const PositionMeasurement& measurement);

    /**
     * Integrates a 2D position measurement. XY of body in navigation frame in m.
     */
    void integrateMeasurement(const XYMeasurement& measurement);

    /**
     * Integrates a 1D position measurement. Z of body in navigation frame in m.
     */
    void integrateMeasurement(const ZMeasurement& measurement);

    /**
     * Integrates a 3D orientation measurement. As axis-angle representation of body in navigation frame.
     */
    void integrateMeasurement(const OrientationMeasurement& measurement);

    /**
     * Integrates a 3D linear velocity measurement. Velocity of body in m/s.
     */
    void integrateMeasurement(const VelocityMeasurement& measurement);

    /**
     * Integrates a 2D linear velocity measurement. XY velocity of body in m/s.
     */
    void integrateMeasurement(const XYVelocityMeasurement& measurement);

    /**
     * Integrates a 1D linear velocity measurement. Z velocity of body in m/s.
     */
    void integrateMeasurement(const ZVelocityMeasurement& measurement);

    /**
     * Integrates a 2D linear and angular velocity measurement.
     * X velocity of body in m/s and yaw velocity of body in rad/s.
     */
    void integrateMeasurement(const XVelYawVelMeasurement& measurement);

    /**
     * Integrates the 3D rotation rates. As angular velocity vector of the body in rad/s.
     */
    void integrateMeasurement(const AngularVelocityMeasurement& measurement);

    /**
     * Sets the current acceleration of the body in m/s^2.
     * Note that this is not integrated but propagated in the prediction step to the velocity.
     * As soon as this is used it should be updated continuously.
     */
    void integrateMeasurement(const AccelerationMeasurement& measurement);

protected:
    virtual void predictionStepImpl(const double delta);

protected:
    AccelerationMeasurement acceleration;
};

}

#endif