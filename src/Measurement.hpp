#ifndef _POSE_ESTIMATION_MEASUREMENT_HPP
#define _POSE_ESTIMATION_MEASUREMENT_HPP

#include <Eigen/Core>
#include <base/Time.hpp>
#include <string>

namespace pose_estimation
{

struct StateAndCovariance
{
    typedef double scalar;
    typedef Eigen::Matrix<scalar, Eigen::Dynamic, 1> Mu;
    typedef Eigen::Matrix<scalar, Eigen::Dynamic, Eigen::Dynamic> Cov;

    Mu mu;
    Cov cov;
};

enum MeasurementIntegration
{
    Filter,
    User
};

struct Measurement : public StateAndCovariance
{
    typedef Eigen::Matrix<unsigned, Eigen::Dynamic, 1> StateMapping;

    base::Time time;

    // This maps the measurement state to the actual filter state
    StateMapping state_mapping;

    std::string measurement_name;

    MeasurementIntegration integration;

    Measurement() : integration(Filter) {}
};

}

#endif