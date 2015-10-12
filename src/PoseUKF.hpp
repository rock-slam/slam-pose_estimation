#ifndef _POSE_ESTIMATION_POSE_UKF_HPP
#define _POSE_ESTIMATION_POSE_UKF_HPP

#include <pose_estimation/Measurement.hpp>
#include <pose_estimation/AbstractFilter.hpp>
#include <pose_estimation/mtk_ukf/PoseWithVelocity.hpp>
#include <iostream>
#include <ukfom/ukf.hpp>
#include <ukfom/mtkwrap.hpp>
#include <boost/shared_ptr.hpp>

namespace pose_estimation
{
    
class PoseUKF : public AbstractRBSFilter
{
public:
    PoseUKF();
    virtual ~PoseUKF() {}
    
    virtual void setInitialState(const FilterState& initial_state);
    virtual void setProcessNoiseCovariance(const FilterState::Cov& noise_cov);
    virtual void predictionStep(const double delta);
    virtual void correctionStep(const Measurement& measurement);
    virtual const FilterState& getCurrentState();
    
protected:
    typedef MTK::SO3<double> MTKRotationType;
    typedef PoseWithVelocity<MTKRotationType> PoseState;
    typedef ukfom::mtkwrap<PoseState> WPoseState;
    typedef ukfom::ukf<WPoseState> MTK_UKF;

    static void muToUKFState(const FilterState::Mu &mu, WPoseState& state);
    static void UKFStateToMu(const WPoseState& state, FilterState::Mu &mu);

protected:
    boost::shared_ptr<MTK_UKF> ukf;
    bool dirty;
    FilterState state;
    base::samples::RigidBodyAcceleration last_acceleration_sample;
    MTK_UKF::cov process_noise_cov;
};



}

#endif
