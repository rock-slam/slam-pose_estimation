#ifndef _POSE_ESTIMATION_ORIENTATION_UKF_CONFIG_HPP
#define _POSE_ESTIMATION_ORIENTATION_UKF_CONFIG_HPP

#include <base/Eigen.hpp>

namespace pose_estimation
{

struct InertialNoiseParameters
{
    /*  Random walk ((m/s^s)/sqrt(Hz) for accelerometers or (rad/s)/sqrt(Hz) for gyros) */
    base::Vector3d randomwalk;

    /*  Bias offset in static regimen (initial bias value) */
    base::Vector3d bias_offset;

    /* Bias instability (m/s^2 for accelerometers or rad/s for gyros) */
    base::Vector3d bias_instability;

    /* Tau value to limit the bias gain in seconds */
    double bias_tau;
};

struct LocationConfiguration
{
    /* Latitude in radians */
    double latitude;

    /* Longitude in radians */
    double longitude;

    /* Altitude in meters */
    double altitude;
};

struct OrientationUKFConfig
{
    /* Inerial noise parameters for acceleration */
    InertialNoiseParameters acceleration;

    /** Inerial noise parameters for acceleration */
    InertialNoiseParameters rotation_rate;

    /** Velocity process noise, diagonal of the covariance matrix */
    base::Vector3d velocity_diag;

    /* Latitude and Longitude of operational area */
    LocationConfiguration location;
};

}

#endif