#ifndef _POSE_ESTIMATION_POSE_ESTIMATOR_HPP
#define _POSE_ESTIMATION_POSE_ESTIMATOR_HPP

#include <pose_estimation/Measurement.hpp>
#include <pose_estimation/AbstractFilter.hpp>

#include <boost/shared_ptr.hpp>
#include <limits>
#include <queue>
#include <map>

namespace pose_estimation
{

struct newer_measurement : std::binary_function <Measurement,Measurement,bool> {
  bool operator() (const Measurement& x, const Measurement& y) const {return x.time>=y.time;}
};

class PoseEstimator
{
public:
    PoseEstimator(boost::shared_ptr<AbstractFilter> filter);
    
    void setInitialState(const AbstractFilter::FilterState &state);
    void setProcessNoise(const AbstractFilter::FilterState::Cov& process_noise);
    void setMaxTimeDelta(double max_time_delta);

    bool enqueueMeasurement(const Measurement& measurement);
    
    void integrateMeasurements(unsigned measurement_count = std::numeric_limits<unsigned>::max());
    void integrateMeasurements(const base::Time& current_time);
    
    bool getEstimatedState(AbstractFilter::FilterState &estimated_state);

    bool measurementsInQueue() {return !measurement_queue.empty();}
    size_t measurementQueueSize() {return measurement_queue.size();}
    
    base::Time getLastMeasurementTime() {return last_measurement_time;}
    
protected:

    void processMeasurement(const Measurement &measurement);
    
    
    std::priority_queue<Measurement, std::deque<Measurement>, newer_measurement> measurement_queue;
    std::map<std::string, Measurement> latest_measurements;
    boost::shared_ptr<AbstractFilter> filter;
    base::Time last_measurement_time;
    double max_time_delta;
};

}


#endif