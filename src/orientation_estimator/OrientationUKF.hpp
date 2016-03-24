#ifndef _POSE_ESTIMATION_ORIENTATION_UKF_HPP
#define _POSE_ESTIMATION_ORIENTATION_UKF_HPP

#include "OrientationState.hpp"
#include "OrientationUKFConfig.hpp"
#include <pose_estimation/Measurement.hpp>
#include <pose_estimation/UKF.hpp>
#include <map>
#include <Eigen/Core>

namespace pose_estimation
{

class OrientationUKF : public UKF<OrientationState>
{
public:
    static const std::string acceleration_measurement;
    static const std::string rotation_rate_measurement;
    static const std::string velocity_measurement;

public:

    OrientationUKF(const FilterState& initial_state, const OrientationUKFConfig& config);
    virtual ~OrientationUKF() {}
    
    virtual void predictionStep(const double delta);

    void setFilterConfiguration(const OrientationUKFConfig& config);
    
    
protected:
    virtual void correctionStepUser(const pose_estimation::Measurement& measurement);

    /* This needs to called after the filter configuration was updated */
    void updateFilterParamter();

    virtual void muToUKFState(const FilterState::Mu &mu, WState& state) const;
    virtual void UKFStateToMu(const WState& state, FilterState::Mu &mu) const;

protected:
    std::map<std::string, pose_estimation::Measurement> latest_measurements;
    
    OrientationUKFConfig config;

    Eigen::Vector3d earth_rotation;
    Eigen::Vector3d gravity;
};

}

#endif