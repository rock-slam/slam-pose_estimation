#include "AbstractFilter.hpp"

#include <pose_estimation/Exceptions.hpp>
#include <base/Logging.hpp>
#include <base/Float.hpp>

using namespace pose_estimation;

void AbstractFilter::setInitialState(const FilterState& initial_state)
{
    unsigned state_size = getStateSize();
    if(initial_state.mu.rows() != state_size ||
        initial_state.cov.rows() != state_size ||
        initial_state.cov.cols() != state_size)
        throw WrongStateSizeException(state_size);
    setInitialStateImpl(initial_state);
}

void AbstractFilter::setProcessNoiseCovariance(const FilterState::Cov& noise_cov)
{
    unsigned state_size = getStateSize();
    if(noise_cov.rows() != state_size ||
        noise_cov.cols() != state_size)
        throw WrongStateSizeException(state_size);
    setProcessNoiseCovarianceImpl(noise_cov);
}

void AbstractFilter::correctionStep(const Measurement& measurement)
{
    Measurement m_corrected;
    if(checkMeasurement(measurement, m_corrected))
        correctionStepImpl(m_corrected);
}

void AbstractFilter::userIntegration(const Measurement& measurement)
{
    Measurement m_corrected;
    if(checkMeasurement(measurement, m_corrected))
        userIntegrationImpl(m_corrected);
}

bool AbstractFilter::checkMeasurement(const Measurement& measurement, Measurement& measurement_corrected)
{
    // check if measurement is healty
    // check measurement name
    if(measurement.measurement_name.empty())
    {
        LOG_ERROR("Measurement name must be set! This measurement will be skipped.");
        return false;
    }

    // check timestamp
    if(measurement.time.isNull())
    {
        LOG_ERROR_S << "Measurement " << measurement.measurement_name << " doesn't have a valid timestamp! This measurement will be skipped.";
        return false;
    }

    // check state mapping
    Measurement::StateMapping mapping = measurement.state_mapping;
    unsigned state_size = getStateSize();
    if(measurement.state_mapping.rows() > state_size)
    {
        LOG_ERROR_S << "State mapping can not extend the current state size! This measurement " << measurement.measurement_name << " will be skipped.";
        return false;
    }
    for(unsigned r = 0; r < measurement.state_mapping.rows(); r++)
    {
        if(base::isNaN<Measurement::StateMapping::Scalar>(measurement.state_mapping(r)))
        {
            LOG_ERROR_S << "State mapping contains NaN values! This measurement " << measurement.measurement_name << " will be skipped.";
            return false;
        }
        else if(measurement.state_mapping(r) > state_size)
        {
            LOG_ERROR_S  << "State mapping can not extend the current state size! This measurement " << measurement.measurement_name << " will be skipped.";
            return false;
        }
    }

    // check measurement for NaN values
    for(unsigned r = 0; r < measurement.mu.rows(); r++)
    {
        if(base::isNaN<Measurement::Mu::Scalar>( measurement.mu(r) ))
        {
            // handle NaN values in state
            LOG_ERROR_S << "State contains NaN values! This measurement " << measurement.measurement_name << " will be skipped.";
            return false;
        }
    }

    // check covariance matrix for NaN values
    Measurement::Cov cov = measurement.cov;
    for(unsigned c = 0; c < cov.cols(); c++)
    {
        for(unsigned r = 0; r < cov.rows(); r++)
        {
            if(base::isNaN<Measurement::Cov::Scalar>(cov(r,c)))
            {
                // handle NaN variances
                LOG_ERROR_S  << "Covariance contains NaN values! This measurement " << measurement.measurement_name << " will be skipped.";
                return false;
            }
            else if(c==r && cov(r,c) == 0.0)
            {
                // handle zero variances
                LOG_WARN_S << "Covariance diagonal contains zero values. Override them with %d", 1e-9;
                cov(r,c) = 1e-9;
            }
        }
    }

    measurement_corrected = measurement;
    measurement_corrected.cov = cov;
    return true;
}