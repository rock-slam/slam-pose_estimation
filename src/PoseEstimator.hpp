#ifndef _POSE_ESTIMATION_POSE_ESTIMATOR_HPP
#define _POSE_ESTIMATION_POSE_ESTIMATOR_HPP

#include <pose_estimation/Measurement.hpp>
#include <pose_estimation/AbstractFilter.hpp>
#include <pose_estimation/PoseEstimatorConfig.hpp>

#include <boost/shared_ptr.hpp>
#include <queue>

namespace pose_estimation
{

class PoseEstimator
{
public:    
    PoseEstimator(FilterType filter_type);
    
    void setInitialState(const base::samples::RigidBodyState &body_state);
    void setProcessNoise(const Covariance& process_noise);
    void setMaxTimeDelta(double max_time_delta);
    
    bool enqueueMeasurement(const base::samples::RigidBodyState& body_state,
			    const Measurement::MemberMask& member_mask);
    bool enqueueMeasurement(const Measurement& measurement);
    
    void integrateMeasurements();
    void integrateMeasurements(const base::Time& current_time);
    
    void processMeasurement(const Measurement &measurement);
    
    bool getEstimatedState(base::samples::RigidBodyState &estimated_state);
    
    
protected:
    bool checkMemberMask(const Measurement::MemberMask& member_mask);
    
    
    std::priority_queue<Measurement, std::vector<Measurement>, Measurement> measurement_queue;
    boost::shared_ptr<AbstractFilter> filter;
    base::Time last_measurement_time;
    double max_time_delta;
};

}


#endif