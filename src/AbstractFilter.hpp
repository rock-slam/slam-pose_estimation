#ifndef _POSE_ESTIMATION_ABSTRACT_FILTER_HPP
#define _POSE_ESTIMATION_ABSTRACT_FILTER_HPP

#include <pose_estimation/Measurement.hpp>

namespace pose_estimation
{
    class AbstractFilter
    {
    public:
        typedef StateAndCovariance FilterState;

        virtual ~AbstractFilter() { }

	void setInitialState(const FilterState& initial_state);

	void setProcessNoiseCovariance(const FilterState::Cov& noise_cov);

        void correctionStep(const Measurement& measurement);

        virtual void predictionStep(const double delta) = 0;

	virtual const FilterState& getCurrentState() = 0;

        virtual unsigned getStateSize() const = 0;

    protected:
        virtual void setInitialStateImpl(const FilterState& initial_state) = 0;

        virtual void setProcessNoiseCovarianceImpl(const FilterState::Cov& noise_cov) = 0;

        virtual void correctionStepImpl(const Measurement& measurement) = 0;

        bool checkMeasurement(const Measurement& measurement, Measurement& measurement_corrected);
    };
}

#endif
