#ifndef _POSE_ESTIMATION_POSE_UKF_HPP
#define _POSE_ESTIMATION_POSE_UKF_HPP

#include <pose_estimation/Measurement.hpp>
#include <pose_estimation/UKF.hpp>
#include "PoseWithVelocity.hpp"
#include <map>

namespace pose_estimation
{

class PoseUKF : public UKF<PoseWithVelocity>
{
public:
    PoseUKF(const FilterState& initial_state);
    virtual ~PoseUKF() {}
    
    virtual void predictionStep(const double delta);
    
protected:
    virtual void correctionStepImpl(const Measurement& measurement);
    virtual void userIntegrationImpl(const Measurement& measurement);

    virtual void muToUKFState(const FilterState::Mu &mu, WState& state) const;
    virtual void UKFStateToMu(const WState& state, FilterState::Mu &mu) const;

protected:
    std::map<std::string, Measurement> latest_measurements;
};

}

#endif