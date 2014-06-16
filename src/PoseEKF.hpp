#ifndef _POSE_ESTIMATION_POSE_EKF_HPP
#define _POSE_ESTIMATION_POSE_EKF_HPP

#include <pose_estimation/Measurement.hpp>
#include <pose_estimation/AbstractFilter.hpp>
#include <pose_estimation/ros_ekf/ekf.hpp>

namespace pose_estimation
{
    
class PoseEKF : public RobotLocalization::Ekf, public AbstractFilter
{
public:
    PoseEKF();
    
    virtual void setInitialState(const base::samples::RigidBodyState &body_state);
    virtual void setProcessNoiseCovariance(const Covariance& noise_cov);
    virtual void predictionStep(const double delta);
    virtual void correctionStep(const Measurement &measurement);
    virtual const base::samples::RigidBodyState& getCurrentState();
    
protected:
    void rigidBodyStateToMarix(const base::samples::RigidBodyState &body_state, Eigen::VectorXd& state, Eigen::MatrixXd& covariance);
    void matrixToRigidBodyState(const Eigen::VectorXd& state, const Eigen::MatrixXd& covariance, base::samples::RigidBodyState &body_state);
    
protected:
    bool dirty;
    base::samples::RigidBodyState body_state;
};

}

#endif