#ifndef _POSE_ESTIMATION_POSE_EKF_HPP
#define _POSE_ESTIMATION_POSE_EKF_HPP

#include <pose_estimation/Measurement.hpp>
#include <pose_estimation/AbstractFilter.hpp>
#include <pose_estimation/ros_ekf/ekf.hpp>

namespace pose_estimation
{
    
class PoseEKF : public RobotLocalization::Ekf, public AbstractRBSFilter
{
public:
    PoseEKF();
    virtual ~PoseEKF() {};
    
    virtual void setInitialState(const FilterState& initial_state);
    virtual void setProcessNoiseCovariance(const FilterState::Cov& noise_cov);
    virtual void predictionStep(const double delta);
    virtual void correctionStep(const Measurement& measurement);
    virtual const FilterState& getCurrentState();
    
protected:
    bool dirty;
    FilterState state;
};

}

#endif