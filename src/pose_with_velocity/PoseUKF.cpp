#include "PoseUKF.hpp"
#include <base/Float.hpp>
#include <base/Logging.hpp>
#include <pose_estimation/EulerConversion.hpp>
#include <assert.h>

namespace pose_estimation
{

/** Measurement model for the 12D robot state (pose and velocity)
 *  The mapping defines which subset is returned.
 */
template <typename PoseWithVelocityType>
Eigen::Matrix<typename PoseWithVelocityType::scalar, -1, 1>
measurementModel (const PoseWithVelocityType &state, const Eigen::Matrix<unsigned, -1, 1>& mapping)
{
    //return state.getSubStateVector(mapping);

    Eigen::Matrix<typename PoseWithVelocityType::scalar, PoseWithVelocityType::DOF, 1> state_vector;
    state_vector.block(MTK::getStartIdx(&PoseWithVelocity::position),0,MTK::getDof(&PoseWithVelocity::position),1) = state.position;
    state_vector.block(MTK::getStartIdx(&PoseWithVelocity::orientation),0,MTK::getDof(&PoseWithVelocity::orientation),1) = MTK::SO3<typename PoseWithVelocityType::scalar>::log(state.orientation);
    state_vector.block(MTK::getStartIdx(&PoseWithVelocity::velocity),0,MTK::getDof(&PoseWithVelocity::velocity),1) = state.velocity;
    state_vector.block(MTK::getStartIdx(&PoseWithVelocity::angular_velocity),0,MTK::getDof(&PoseWithVelocity::angular_velocity),1) = state.angular_velocity;

    Eigen::Matrix<typename PoseWithVelocityType::scalar, -1, 1> sub_state_vector;
    assert(mapping.rows() <= PoseWithVelocityType::DOF);
    sub_state_vector.resize(mapping.rows());
    for(unsigned i = 0; i < mapping.rows(); i++)
    {
        sub_state_vector(i) = state_vector(mapping(i));
    }
    return sub_state_vector;

}

/** Process model for the 12D robot state.
 * Applies the current velocity to update the robot pose.
 */
template <typename PoseWithVelocityType>
PoseWithVelocityType
processModel (const PoseWithVelocityType &state, double delta_time)
{
    PoseWithVelocityType new_state(state);
    new_state.position.boxplus(new_state.orientation * new_state.velocity, delta_time);
    new_state.orientation.boxplus(new_state.angular_velocity, delta_time);
    return new_state;
}

/** Process model with acceleration for the 12D robot state.
 * Applies the current velocity and acceleration to update the robot pose.
 */
template <typename PoseWithVelocityType>
PoseWithVelocityType
processModelWithAcceleration (const PoseWithVelocityType &state, const Eigen::Vector3d& acc, double delta_time)
{
    PoseWithVelocityType new_state(state);
    new_state.velocity.boxplus(acc, delta_time);
    new_state.position.boxplus(new_state.orientation * new_state.velocity, delta_time);
    new_state.orientation.boxplus(new_state.angular_velocity, delta_time);
    return new_state;
}

PoseUKF::PoseUKF(const FilterState& initial_state) : UKF<pose_estimation::PoseWithVelocity>()
{
    setInitialState(initial_state);

    process_noise_cov = MTK_UKF::cov::Zero();
    MTK::setDiagonal(process_noise_cov, &WState::position, 0.01);
    MTK::setDiagonal(process_noise_cov, &WState::orientation, 0.001);
    MTK::setDiagonal(process_noise_cov, &WState::velocity, 0.00001);
    MTK::setDiagonal(process_noise_cov, &WState::angular_velocity, 0.00001);
}

void PoseUKF::predictionStep(const double delta)
{
    std::map<std::string, Measurement>::const_iterator it = latest_measurements.find("acceleration");
    if(it == latest_measurements.end())
        ukf->predict(boost::bind(processModel<WState>, _1 , delta), MTK_UKF::cov(delta * process_noise_cov));
    else
    {
        MTK_UKF::cov process_noise = process_noise_cov;
        process_noise.block(6,6,3,3) = 2.0 * it->second.cov;
        ukf->predict(boost::bind(processModelWithAcceleration<WState>, _1, it->second.mu, delta), MTK_UKF::cov(delta * process_noise));
    }
}

void PoseUKF::correctionStepImpl(const Measurement& measurement)
{
    // apply new measurement
    ukf->update(measurement.mu, boost::bind(measurementModel<WState>, _1, measurement.state_mapping),
                boost::bind(ukfom::id< Eigen::MatrixXd >, measurement.cov),
                ukfom::accept_any_mahalanobis_distance<MTK_UKF::scalar_type>);
}

void PoseUKF::userIntegrationImpl(const Measurement& measurement)
{
    latest_measurements[measurement.measurement_name] = measurement;
}

void PoseUKF::muToUKFState(const FilterState::Mu& mu, WState& state) const
{
    assert(mu.rows() >= WState::DOF);

    state.position = mu.block(0, 0, 3, 1);
    base::Orientation orientation;
    Eigen::Vector3d euler = mu.block(3, 0, 3, 1);
    EulerConversion::eulerToQuad(euler, orientation);
    state.orientation = MTK::SO3<double>(orientation);
    state.velocity = mu.block(6, 0, 3, 1);
    Eigen::Vector3d angle_axis;
    Eigen::Vector3d euler_velocity = mu.block(9, 0, 3, 1);
    EulerConversion::eulerAngleVelocityToAngleAxis(euler_velocity, angle_axis);
    state.angular_velocity = angle_axis;

}

void PoseUKF::UKFStateToMu(const WState& state, FilterState::Mu& mu) const
{
    mu.resize(WState::DOF);
    mu.setZero();

    mu.block(0, 0, 3, 1) = state.position;
    Eigen::Vector3d euler;
    base::Orientation orientation = state.orientation;
    EulerConversion::quadToEuler(orientation, euler);
    mu.block(3, 0, 3, 1) = euler;
    mu.block(6, 0, 3, 1) = state.velocity;
    Eigen::Vector3d euler_velocity;
    Eigen::Vector3d angular_velocity = state.angular_velocity;
    EulerConversion::angleAxisToEulerAngleVelocity(angular_velocity, euler_velocity);
    mu.block(9, 0, 3, 1) = euler_velocity;
}

}
