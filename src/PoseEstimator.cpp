#include "PoseEstimator.hpp"
#include <pose_estimation/pose_with_velocity/PoseUKF.hpp>
#include <base/Logging.hpp>
#include <base/Float.hpp>

namespace pose_estimation
{

PoseEstimator::PoseEstimator(boost::shared_ptr<AbstractFilter> filter) : filter(filter),
                last_measurement_time(base::Time::fromSeconds(0.0)), max_time_delta(base::infinity<double>())
{
    if(filter.get() == NULL)
        throw std::runtime_error("The pose estimator needs a valid filter!");
}

void PoseEstimator::setInitialState(const AbstractFilter::FilterState& state)
{
    filter->setInitialState(state);
}

void PoseEstimator::setProcessNoise(const AbstractFilter::FilterState::Cov& process_noise)
{
    filter->setProcessNoiseCovariance(process_noise);
}

void PoseEstimator::setMaxTimeDelta(double max_time_delta)
{
    this->max_time_delta = max_time_delta;
}

bool PoseEstimator::enqueueMeasurement(const Measurement& measurement)
{
    if(measurement.time < last_measurement_time)
    {
	LOG_WARN("Attempt to enqueue an older measurement. This Measurement will be skiped.");
	return false;
    }

    measurement_queue.push(measurement);
    latest_measurements[measurement.measurement_name] = measurement;
    
    return true;
}

void PoseEstimator::integrateMeasurements(unsigned measurement_count)
{
    unsigned measurements_handled = 0;
    while (!measurement_queue.empty() && measurements_handled <= measurement_count)
    {
	Measurement measurement = measurement_queue.top();
	measurement_queue.pop();

	processMeasurement(measurement);
        measurements_handled++;
    }
}

void PoseEstimator::integrateMeasurements(const base::Time& current_time)
{
    integrateMeasurements();
    
    if(!last_measurement_time.isNull())
    {
	double time_delta = (current_time - last_measurement_time).toSeconds();
	
	if(time_delta < 0.0)
	    throw std::runtime_error("Attempt to go back in time. Time delta is negative!");
	
	// prediction step
	if(time_delta > max_time_delta)
	{ 
	    LOG_WARN("Time delta is to high: %f. Skip this prediction step.");
	}
	else if(time_delta > 0.0)
	    filter->predictionStep(time_delta);
    }
    
    last_measurement_time = current_time;
}

void PoseEstimator::processMeasurement(const Measurement& measurement)
{
    if(!last_measurement_time.isNull())
    {
	double time_delta = (measurement.time - last_measurement_time).toSeconds();
	
	if(time_delta < 0.0)
	    throw std::runtime_error("Attempt to process an older measurement. Time delta is negative!");
	
	// prediction step
	if(time_delta > max_time_delta)
	{ 
	    LOG_WARN("Time delta is to high: %f. Skip this prediction step.");
	}
	else if(time_delta > 0.0)
	    filter->predictionStep(time_delta);
    }
    
    // correction step
    if(measurement.integration == Filter)
        filter->correctionStep(measurement);
    else if(measurement.integration == User)
        filter->userIntegration(measurement);
    else
        throw std::runtime_error("Unknown integration type selected!");
    
    last_measurement_time = measurement.time;
}

bool PoseEstimator::getEstimatedState(AbstractFilter::FilterState &estimated_state)
{
    if(last_measurement_time.isNull())
	return false;
    
    estimated_state = filter->getCurrentState();
    return true;
}

}