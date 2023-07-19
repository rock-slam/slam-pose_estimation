#ifndef _POSE_ESTIMATION_ORIENTATION_UKF_CONFIG_HPP
#define _POSE_ESTIMATION_ORIENTATION_UKF_CONFIG_HPP

#include <Eigen/Geometry>

namespace pose_estimation
{

    struct InertialNoiseParameters
    {
        /*  Random walk ((m/s^s)/sqrt(Hz) for accelerometers or (rad/s)/sqrt(Hz) for gyros) */
        Eigen::Vector3d randomwalk;

        /*  Bias offset in static regimen (initial bias value) */
        Eigen::Vector3d bias_offset;

        /* Bias instability (m/s^2 for accelerometers or rad/s for gyros) */
        Eigen::Vector3d bias_instability;

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

        /* Latitude and Longitude of operational area */
        LocationConfiguration location;

        /* Max velocity in m/s */
        Eigen::Vector3d max_velocity;
    };

}

#endif