#ifndef _POSE_ESTIMATION_ABSTRACT_FILTER_HPP
#define _POSE_ESTIMATION_ABSTRACT_FILTER_HPP

#include <pose_estimation/Measurement.hpp>
#include <base/samples/RigidBodyState.hpp>

namespace pose_estimation
{
    class AbstractFilter
    {
    public:
        virtual ~AbstractFilter() { }
	virtual void setInitialState(const base::samples::RigidBodyState& body_state) = 0;
	virtual void setProcessNoiseCovariance(const Covariance& noise_cov) = 0;
	virtual void predictionStep(const double delta) = 0;
	virtual void correctionStep(const Measurement& measurement) = 0;
	virtual const base::samples::RigidBodyState& getCurrentState() = 0;
        virtual base::VectorXd getFullState() = 0;
        virtual base::MatrixXd getFullCovariance() = 0;
    };
}

#endif
