#include "PoseEKF.hpp"

namespace pose_estimation
{

PoseEKF::PoseEKF() : Ekf(), dirty(true)
{
}

void PoseEKF::setInitialState(const FilterState& initial_state)
{
    dirty = true;

    Ekf::state_ = initial_state.mu;
    Ekf::estimateErrorCovariance_ = initial_state.cov;
}

void PoseEKF::setProcessNoiseCovariance(const FilterState::Cov& noise_cov)
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
    m.time_ = measurement.time.toSeconds();
    m.measurement_ = measurement.mu;
    m.covariance_ = measurement.cov;
    m.updateVector_.resize(BODY_STATE_SIZE);
    assert(measurement.mask.rows() >= BODY_STATE_SIZE);
    Eigen::Map< Eigen::Matrix<int, BODY_STATE_SIZE, 1> >(m.updateVector_.data()) = measurement.mask.cast<int>().block(0,0,BODY_STATE_SIZE,1);
    Ekf::correct(m);
}

const AbstractFilter::FilterState& PoseEKF::getCurrentState()
{
    if(dirty)
    {
        state.mu = Ekf::getState();
        state.cov = Ekf::getEstimateErrorCovariance();
        dirty = false;
    }
    
    return state;
}

}