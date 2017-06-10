#ifndef _POSE_ESTIMATION_POSE_UKF_CONFIG_HPP
#define _POSE_ESTIMATION_POSE_UKF_CONFIG_HPP

namespace pose_estimation
{

struct VelocityBiasConfig
{
    // time scale for velocity bias change
    double tau;
    // rate change of velocity bias based on spatial change
    double scale;
};

}

#endif