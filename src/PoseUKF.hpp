#ifndef _POSE_ESTIMATION_POSE_UKF_HPP
#define _POSE_ESTIMATION_POSE_UKF_HPP

#include <pose_estimation/Measurement.hpp>
#include <pose_estimation/AbstractFilter.hpp>
#include <pose_estimation/mtk_ukf/PoseWithVelocity.hpp>
#include <pose_estimation/mtk_ukf/Bias.hpp>
#include <pose_estimation/mtk_ukf/PoseWithVelocityBias.hpp>
#include <iostream>
#include <ukfom/ukf.hpp>
#include <ukfom/mtkwrap.hpp>
#include <boost/shared_ptr.hpp>

namespace pose_estimation
{
    
class PoseUKF : public AbstractFilter
{
public:
    PoseUKF();
    
    virtual void setInitialState(const base::samples::RigidBodyState &body_state);
    virtual void setProcessNoiseCovariance(const Covariance& noise_cov);
    virtual void predictionStep(const double delta);
    virtual void correctionStep(const Measurement &measurement);
    virtual const base::samples::RigidBodyState& getCurrentState();
    virtual base::VectorXd getFullState();
    virtual base::MatrixXd getFullCovariance();
    
protected:
    typedef MTK::SO3<double> MTKRotationType;
    typedef PoseWithVelocity<MTKRotationType> PoseState;
    typedef Bias<1> BiasState;
    typedef PoseWithVelocityBias<PoseState, BiasState> PoseStateBias;
    typedef ukfom::mtkwrap<PoseStateBias> WPoseState;
    typedef ukfom::ukf<WPoseState> MTK_UKF;
    
    void rigidBodyStateToUKFState(const base::samples::RigidBodyState &body_state, PoseState& state, PoseState::cov& covariance);
    void UKFStateToRigidBodyState(const PoseState& state, const PoseState::cov& covariance, base::samples::RigidBodyState &body_state);

    template<class Base, class T, int idx, int cov_size>
    void setDiagonal(Eigen::Matrix<typename Base::scalar, cov_size, cov_size> &cov,
                    MTK::SubManifold<T, idx> Base::*, const typename Base::scalar &val)
    {
        cov.diagonal().template segment<T::DOF>(idx).setConstant(val);
    }
    
protected:    
    boost::shared_ptr<MTK_UKF> ukf;
    bool dirty;
    base::samples::RigidBodyState body_state;
    base::samples::RigidBodyAcceleration last_acceleration_sample;
    MTK_UKF::cov process_noise_cov;
};

}

#endif
