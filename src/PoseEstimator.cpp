#include "PoseEstimator.hpp"
#include <pose_estimation/PoseEKF.hpp>
#include <pose_estimation/PoseUKF.hpp>
#include <base/Logging.hpp>
#include <base/Float.hpp>

namespace pose_estimation
{

PoseEstimator::PoseEstimator(FilterType filter_type) : last_measurement_time(base::Time::fromSeconds(0.0)), max_time_delta(base::infinity<double>())
{
    if(filter_type == UKF)
	filter.reset(new PoseUKF());
    else  
	filter.reset(new PoseEKF());
}

void PoseEstimator::setInitialState(const base::samples::RigidBodyState& body_state)
{
    filter->setInitialState(body_state);
}

void PoseEstimator::setProcessNoise(const Covariance& process_noise)
{
    filter->setProcessNoiseCovariance(process_noise);
}

void PoseEstimator::setMaxTimeDelta(double max_time_delta)
{
    this->max_time_delta = max_time_delta;
}

bool PoseEstimator::enqueueMeasurement(const base::samples::RigidBodyState& body_state, const Measurement::MemberMask& member_mask)
{
    if(!checkMemberMask(member_mask))
    {
	throw std::runtime_error("Member mask contains invalid values. Only 0 and 1 is allowed." );
    }
    
    Measurement measurement;
    measurement.time = body_state.time;
    measurement.body_state = body_state;
    measurement.member_mask = member_mask;
    return enqueueMeasurement(measurement);
}

bool PoseEstimator::enqueueMeasurement(const base::Time time,
                                       const base::samples::RigidBodyState& body_state,
                                       const base::samples::RigidBodyAcceleration& acceleration,
                                       const Measurement::MemberMask& member_mask)
{
    if(!checkMemberMask(member_mask))
    {
        throw std::runtime_error("Member mask contains invalid values. Only 0 and 1 is allowed." );
    }

    Measurement measurement;
    measurement.time = time;
    measurement.body_state = body_state;
    measurement.acceleration = acceleration;
    measurement.member_mask = member_mask;
    return enqueueMeasurement(measurement);
}

bool PoseEstimator::enqueueMeasurement(const Measurement& measurement)
{
    if(measurement.time < last_measurement_time)
    {
	LOG_WARN("Attempt to enqueue an older measurement. This Measurement will be skiped.");
	return false;
    }
	
    measurement_queue.push(measurement);
    
    return true;
}

void PoseEstimator::integrateMeasurements()
{
    while (!measurement_queue.empty())
    {
	Measurement measurement = measurement_queue.top();
	measurement_queue.pop();

	processMeasurement(measurement);
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
    filter->correctionStep(measurement);
    
    last_measurement_time = measurement.time;
}

bool PoseEstimator::getEstimatedState(base::samples::RigidBodyState &estimated_state)
{
    if(last_measurement_time.isNull())
	return false;
    
    estimated_state = filter->getCurrentState();
    estimated_state.time = last_measurement_time;
    return true;
}

bool PoseEstimator::checkMemberMask(const Measurement::MemberMask& member_mask)
{
    if(member_mask.rows() != MEASUREMENT_SIZE || member_mask.cols() != 1)
	return false;
    for(unsigned i = 0; i < MEASUREMENT_SIZE; i++)
    {
	if(!(member_mask(i, 0) == 0 || member_mask(i, 0) == 1))
	    return false;
    }
    return true;
}

}