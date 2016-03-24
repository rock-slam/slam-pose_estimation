#ifndef _POSE_ESTIMATION_ORIENTATION_UKF_CONFIG_HPP
#define _POSE_ESTIMATION_ORIENTATION_UKF_CONFIG_HPP

namespace pose_estimation
{

struct OrientationUKFConfig
{
    double gyro_var; // (rad/s)/sqrt(Hz)
    double acc_var; // (m/s)/sqrt(Hz)
    double gyro_bias_std; // rad/s
    double acc_bias_std;  // m/s^2
    double gyro_bias_tau; // in seconds
    double acc_bias_tau;
    double latitude; // in radians
};

}

#endif