#include "PoseEKF.hpp"

namespace pose_estimation
{

PoseEKF::PoseEKF() : Ekf(), dirty(true)
{
}

void PoseEKF::setInitialState(const base::samples::RigidBodyState& body_state)
{
    dirty = true;

    rigidBodyStateToMarix(body_state, Ekf::state_, Ekf::estimateErrorCovariance_);
}

void PoseEKF::setProcessNoiseCovariance(const Covariance& noise_cov)
{
    Ekf::processNoiseCovariance_ = noise_cov;
}

void PoseEKF::predictionStep(const double delta)
{
    dirty = true;
    
    Ekf::predict(delta);
}

void PoseEKF::correctionStep(const Measurement& measurement)
{
    dirty = true;
    
    RobotLocalization::Measurement m;
    m.time_ = measurement.body_state.time.toSeconds();
    rigidBodyStateToMarix(measurement.body_state, m.measurement_, m.covariance_);
    m.updateVector_.resize(BODY_STATE_SIZE);
    Eigen::Map< Eigen::Matrix<int, BODY_STATE_SIZE, 1> >(m.updateVector_.data()) = measurement.member_mask.cast<int>();
    Ekf::correct(m);
}

const base::samples::RigidBodyState& PoseEKF::getCurrentState()
{
    if(dirty)
	matrixToRigidBodyState(Ekf::getState(), Ekf::getEstimateErrorCovariance(), body_state);
    
    return body_state;
}

void PoseEKF::matrixToRigidBodyState(const Eigen::VectorXd& state, const Eigen::MatrixXd& covariance, base::samples::RigidBodyState &body_state)
{
    body_state.position = state.block(0,0,3,1);
    body_state.orientation = base::Orientation(Eigen::AngleAxisd(state(5), Eigen::Vector3d::UnitZ()) * 
						Eigen::AngleAxisd(state(4), Eigen::Vector3d::UnitY()) * 
						Eigen::AngleAxisd(state(3), Eigen::Vector3d::UnitX()));
    body_state.velocity = state.block(6,0,3,1);
    Eigen::AngleAxisd angle_axis = Eigen::AngleAxisd(Eigen::AngleAxisd(state(11), Eigen::Vector3d::UnitZ()) * 
						    Eigen::AngleAxisd(state(10), Eigen::Vector3d::UnitY()) * 
						    Eigen::AngleAxisd(state(9), Eigen::Vector3d::UnitX()));
    body_state.angular_velocity = angle_axis.angle() * angle_axis.axis();
    
    body_state.cov_position = covariance.block(0, 0, 3, 3);
    body_state.cov_orientation = covariance.block(3, 3, 3, 3);
    body_state.cov_velocity = covariance.block(6, 6, 3, 3);
    body_state.cov_angular_velocity = covariance.block(9, 9, 3, 3);
}

void PoseEKF::rigidBodyStateToMarix(const base::samples::RigidBodyState& body_state, Eigen::VectorXd& state, Eigen::MatrixXd& covariance)
{
    state.resize(BODY_STATE_SIZE, 1);
    state.setZero();
    state.block(0,0,3,1) = body_state.position;
    base::Vector3d euler_angles = base::getEuler(body_state.orientation);
    state.block(3,0,3,1) = Eigen::Vector3d(euler_angles.z(), euler_angles.y(), euler_angles.x());
    state.block(6,0,3,1) = body_state.velocity;
    base::Vector3d euler_angle_velocity(0.0,0.0,0.0);
    if(!body_state.angular_velocity.isZero())
	euler_angle_velocity = base::getEuler(base::Orientation(Eigen::AngleAxisd(body_state.angular_velocity.norm(), body_state.angular_velocity.normalized())));
    state.block(9,0,3,1) = Eigen::Vector3d(euler_angle_velocity.z(), euler_angle_velocity.y(), euler_angle_velocity.x());
   
    covariance.resize(BODY_STATE_SIZE, BODY_STATE_SIZE);
    covariance.setZero();
    covariance.block(0, 0, 3, 3) = body_state.cov_position;
    covariance.block(3, 3, 3, 3) = body_state.cov_orientation;
    covariance.block(6, 6, 3, 3) = body_state.cov_velocity;
    covariance.block(9, 9, 3, 3) = body_state.cov_angular_velocity;
}

}