#ifndef _POSE_ESTIMATION_ORIENTATION_UKF_HPP
#define _POSE_ESTIMATION_ORIENTATION_UKF_HPP

#include "OrientationState.hpp"
#include "OrientationUKFConfig.hpp"
#include <pose_estimation/Measurement.hpp>
#include <pose_estimation/UnscentedKalmanFilter.hpp>
#include <Eigen/Geometry>

namespace pose_estimation
{

    /**
     * This filter estimates the orientation of an IMU in the NWU navigation frame.
     * It integrates rotation rates, accelerations and linear velocities.
     * The linear velocities help to constrain the acceleration and to estimate the biases.
     * Given gyroscopes capable of sensing the rotation of the earth (e.g. a fibre optic gyro)
     * this filter is able to estimate it's true heading.
     */
    class OrientationUKF : public UnscentedKalmanFilter<OrientationState>
    {
    public:
        MEASUREMENT(RotationRate, 3)
        MEASUREMENT(Acceleration, 3)
        MEASUREMENT(VelocityMeasurement, 3)

    public:
        OrientationUKF(const State &initial_state, const Covariance &state_cov,
                       double gyro_bias_tau, double acc_bias_tau, const LocationConfiguration &location);
        virtual ~OrientationUKF() {}

        /**
         * Sets the current rotation rate of the IMU in rad/s.
         */
        void integrateMeasurement(const RotationRate &measurement);

        /**
         * Sets the current acceleration of the IMU in m/s^2.
         */
        void integrateMeasurement(const Acceleration &measurement);

        /**
         * Integrate the linear velocitiy of the IMU in m/s.
         */
        void integrateMeasurement(const VelocityMeasurement &measurement);

        /* Returns unbiased rotation rate in IMU frame */
        RotationRate::Mu getRotationRate();

    protected:
        void predictionStepImpl(double delta);

    protected:
        RotationRate rotation_rate;
        Acceleration acceleration;
        Eigen::Vector3d earth_rotation;
        double gyro_bias_tau;
        double acc_bias_tau;
    };

}

#endif