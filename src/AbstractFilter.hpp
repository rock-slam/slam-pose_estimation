#ifndef _POSE_ESTIMATION_ABSTRACT_FILTER_HPP
#define _POSE_ESTIMATION_ABSTRACT_FILTER_HPP

#include <pose_estimation/Measurement.hpp>
#include <base/samples/RigidBodyState.hpp>
#include <Eigen/Core>
#include <assert.h>

namespace pose_estimation
{
    class AbstractFilter
    {
    public:
        typedef StateAndCovariance FilterState;

        virtual ~AbstractFilter() { }
	virtual void setInitialState(const FilterState& initial_state) = 0;
	virtual void setProcessNoiseCovariance(const FilterState::Cov& noise_cov) = 0;
	virtual void predictionStep(const double delta) = 0;
	virtual void correctionStep(const Measurement& measurement) = 0;
	virtual const FilterState& getCurrentState() = 0;
    };


    class AbstractRBSFilter : public AbstractFilter
    {
    public:
        virtual ~AbstractRBSFilter() { }
        virtual void setInitialState(const FilterState& initial_state) = 0;
        virtual void setProcessNoiseCovariance(const FilterState::Cov& noise_cov) = 0;
        virtual void correctionStep(const Measurement& measurement) = 0;
        virtual const FilterState& getCurrentState() = 0;

        void setInitialState(const base::samples::RigidBodyState& body_state)
        {
            FilterState state;
            fromRigidBodyState(body_state, state.mu, state.cov);
            setInitialState(state);
        }

        void setProcessNoiseCovariance(const Covariance& noise_cov)
        {
            FilterState::Cov cov(noise_cov);
            setProcessNoiseCovariance(cov);
        }

        void correctionStep(const BodyStateMeasurement& measurement)
        {
            Measurement m;
            fromBodyStateMeasurement(measurement, m);
            correctionStep(m);
        }

        base::samples::RigidBodyState getCurrentRBSState()
        {
            const FilterState& state = getCurrentState();
            base::samples::RigidBodyState body_state;
            toRigidBodyState(state.mu, state.cov, body_state);
            return body_state;
        }

    protected:

        static void fromBodyStateMeasurement(const BodyStateMeasurement& body_state_measurement, Measurement& measurement)
        {
            measurement.time = body_state_measurement.time;
            StateAndCovariance::Mu mu;
            StateAndCovariance::Cov cov;
            fromRigidBodyState(body_state_measurement.body_state, mu, cov);
            measurement.mu.resize(MEASUREMENT_SIZE);
            measurement.mu.setZero();
            measurement.mu.block(0, 0, BODY_STATE_SIZE, 1) = mu;
            measurement.mu.block(BODY_STATE_SIZE, 0, 3, 1) = body_state_measurement.acceleration.acceleration;
            measurement.cov.resize(MEASUREMENT_SIZE, MEASUREMENT_SIZE);
            measurement.cov.setZero();
            measurement.cov.block(0, 0, BODY_STATE_SIZE, BODY_STATE_SIZE) = cov;
            measurement.cov.block(BODY_STATE_SIZE, BODY_STATE_SIZE, 3, 3) = body_state_measurement.acceleration.cov_acceleration;

            // TODO change this to an index mapping
            measurement.mask.resize(body_state_measurement.member_mask.rows());
            measurement.mask = body_state_measurement.member_mask;

            measurement.measurement_name = body_state_measurement.body_state.sourceFrame;
            measurement.measurement_name += "2";
            measurement.measurement_name += body_state_measurement.body_state.targetFrame;
        }

        static void fromRigidBodyState(const base::samples::RigidBodyState &body_state, StateAndCovariance::Mu &mu, StateAndCovariance::Cov &cov)
        {
            mu.resize(BODY_STATE_SIZE);
            cov.resize(BODY_STATE_SIZE, BODY_STATE_SIZE);

            mu.setZero();
            mu.block(0, 0, 3, 1) = body_state.position;
            Eigen::Vector3d euler;
            quadToEuler(body_state.orientation, euler);
            mu.block(3, 0, 3, 1) = euler;
            mu.block(6, 0, 3, 1) = body_state.velocity;
            Eigen::Vector3d euler_velocity;
            Eigen::Vector3d angular_velocity = body_state.angular_velocity;
            angleAxisToEulerAngleVelocity(angular_velocity, euler_velocity);
            mu.block(9, 0, 3, 1) = euler_velocity;

            cov.setZero();
            cov.block(0, 0, 3, 3) = body_state.cov_position;
            cov.block(3, 3, 3, 3) = body_state.cov_orientation;
            cov.block(6, 6, 3, 3) = body_state.cov_velocity;
            cov.block(9, 9, 3, 3) = body_state.cov_angular_velocity;
        }

        static void toRigidBodyState(const StateAndCovariance::Mu &mu, const StateAndCovariance::Cov &cov, base::samples::RigidBodyState &body_state)
        {
            assert(mu.rows() >= BODY_STATE_SIZE);
            assert(cov.rows() >= BODY_STATE_SIZE && cov.cols() >= BODY_STATE_SIZE);

            body_state.position = mu.block(0,0,3,1);
            Eigen::Vector3d euler = mu.block(3,0,3,1);
            eulerToQuad(euler, body_state.orientation);
            body_state.velocity = body_state.orientation * mu.block(6,0,3,1);
            Eigen::Vector3d euler_velocity = mu.block(9, 0, 3, 1);
            Eigen::Vector3d angular_velocity;
            eulerAngleVelocityToAngleAxis(euler_velocity, angular_velocity);
            body_state.angular_velocity = angular_velocity;

            body_state.cov_position = cov.block(0, 0, 3, 3);
            body_state.cov_orientation = cov.block(3, 3, 3, 3);
            body_state.cov_velocity = cov.block(6, 6, 3, 3);
            body_state.cov_angular_velocity = cov.block(9, 9, 3, 3);
        }

        static void eulerAngleVelocityToAngleAxis(const Eigen::Vector3d &euler, Eigen::Vector3d &angle_axis)
        {
            Eigen::AngleAxisd angle_axis_d = Eigen::AngleAxisd(Eigen::AngleAxisd(euler.z(), Eigen::Vector3d::UnitZ()) *
                                                            Eigen::AngleAxisd(euler.y(), Eigen::Vector3d::UnitY()) *
                                                            Eigen::AngleAxisd(euler.x(), Eigen::Vector3d::UnitX()));
            angle_axis = angle_axis_d.angle() * angle_axis_d.axis();
        }

        static void angleAxisToEulerAngleVelocity(const Eigen::Vector3d &angle_axis, Eigen::Vector3d &euler)
        {
            base::Vector3d euler_angle_velocity(0.0,0.0,0.0);
            if(!angle_axis.isZero())
                euler_angle_velocity = base::getEuler(base::Orientation(Eigen::AngleAxisd(angle_axis.norm(), angle_axis.normalized())));
            euler = Eigen::Vector3d(euler_angle_velocity.z(), euler_angle_velocity.y(), euler_angle_velocity.x());
        }

        static void quadToEuler(const base::Orientation &orientation, Eigen::Vector3d &euler)
        {
            base::Vector3d euler_angles = base::getEuler(orientation);
            euler = Eigen::Vector3d(euler_angles.z(), euler_angles.y(), euler_angles.x());
        }

        static void eulerToQuad(const Eigen::Vector3d euler, base::Orientation &orientation)
        {
            orientation = Eigen::Quaterniond(Eigen::AngleAxisd(euler.z(), Eigen::Vector3d::UnitZ()) *
                                            Eigen::AngleAxisd(euler.y(), Eigen::Vector3d::UnitY()) *
                                            Eigen::AngleAxisd(euler.x(), Eigen::Vector3d::UnitX()));
        }
    };
}

#endif
