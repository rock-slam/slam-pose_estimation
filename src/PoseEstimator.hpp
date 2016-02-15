#ifndef _POSE_ESTIMATION_POSE_ESTIMATOR_HPP
#define _POSE_ESTIMATION_POSE_ESTIMATOR_HPP

#include <pose_estimation/Measurement.hpp>
#include <pose_estimation/AbstractFilter.hpp>
#include <pose_estimation/PoseEstimatorConfig.hpp>

#include <boost/shared_ptr.hpp>
#include <limits>
#include <queue>
#include <list>

namespace pose_estimation
{

struct newer_measurement : std::binary_function <BodyStateMeasurement,BodyStateMeasurement,bool> {
  bool operator() (const BodyStateMeasurement& x, const BodyStateMeasurement& y) const {return x.time>=y.time;}
};

class PoseEstimator
{
public:
    PoseEstimator();
    
    void setInitialState(const base::samples::RigidBodyState &body_state);
    void setProcessNoise(const Covariance& process_noise);
    void setMaxTimeDelta(double max_time_delta);
    
    bool enqueueMeasurement(const base::samples::RigidBodyState& body_state,
			    const BodyStateMeasurement::MemberMask& member_mask);
    bool enqueueMeasurement(const base::Time time,
                            const base::samples::RigidBodyState& body_state,
                            const base::samples::RigidBodyAcceleration& acceleration,
                            const BodyStateMeasurement::MemberMask& member_mask);
    bool enqueueMeasurement(const BodyStateMeasurement& measurement);
    
    void integrateMeasurements(unsigned measurement_count = std::numeric_limits<unsigned>::max());
    void integrateMeasurements(const base::Time& current_time);
    
    void processMeasurement(const BodyStateMeasurement &measurement);
    
    bool getEstimatedState(base::samples::RigidBodyState &estimated_state);

    bool measurementsInQueue() {return !measurement_queue.empty();}
    size_t measurementQueueSize() {return measurement_queue.size();}
    
    
protected:
    bool checkMemberMask(const BodyStateMeasurement::MemberMask& member_mask);
    
    
    std::priority_queue<BodyStateMeasurement, std::deque<BodyStateMeasurement>, newer_measurement> measurement_queue;
    boost::shared_ptr<AbstractRBSFilter> filter;
    base::Time last_measurement_time;
    double max_time_delta;
};

}


#endif