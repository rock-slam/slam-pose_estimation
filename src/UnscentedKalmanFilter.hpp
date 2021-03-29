#ifndef _POSE_ESTIMATION_UKF_HPP
#define _POSE_ESTIMATION_UKF_HPP

#include <iostream>
#include <stdexcept>
#include <ukfom/ukf.hpp>
#include <ukfom/mtkwrap.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/noncopyable.hpp>

namespace pose_estimation
{

template<typename Manifold>
class UnscentedKalmanFilter : private boost::noncopyable
{
public:
    enum {
        DOF = Manifold::DOF
    };
    typedef Manifold State;
    typedef ukfom::mtkwrap<Manifold> WState;
    typedef ukfom::ukf<WState> MTK_UKF;
    typedef typename MTK_UKF::cov Covariance;

    UnscentedKalmanFilter()
    {
        process_noise_cov = Covariance::Zero();
        last_measurement_time_usec = 0;
        min_time_delta = 1.0e-9;
        max_time_delta = std::numeric_limits<double>::max();
    }

    virtual ~UnscentedKalmanFilter() {}

    /**
     * (Re-)initializes the UKF filter from a given state.
     */
    void initializeFilter(const State& initial_state, const Covariance& state_cov)
    {
        ukf.reset(new MTK_UKF(initial_state, state_cov));
        last_measurement_time_usec = 0;
    }

    /**
     * Provides the current state and covariance of the filter.
     *
     * @returns false if the filter has not been initialized.
     */
    bool getCurrentState(State& state, Covariance& state_cov) const
    {
        if(ukf.get() != NULL)
        {
            state = ukf->mu();
            state_cov = ukf->sigma();
            return true;
        }
        return false;
    }

    /**
     * Provides the current state of the filter.
     *
     * @returns false if the filter has not been initialized.
     */
    bool getCurrentState(State& state) const
    {
        if(ukf.get() != NULL)
        {
            state = ukf->mu();
            return true;
        }
        return false;
    }
    
    template<class Time>
    void predictionStepFromSampleTime(const Time& sample_time)
    {
        predictionStepFromSampleTime(sample_time.microseconds);
    }

    /**
     * Computes the time delta from a given sample timestamp and
     * calls predictionStep(delta_t)
     *
     * @param sample_time in microseconds
     * @throws runtime_error if delta_t is negative or greater then the allowed maximum.
     */
    void predictionStepFromSampleTime(int64_t sample_time)
    {
        // first call
        if(!last_measurement_time_usec)
        {
            last_measurement_time_usec = sample_time;
            return;
        }

        // compute delta t
        double delta_t = static_cast<double>(sample_time - last_measurement_time_usec) / 1000000.0;

        // set new last measurement time
        if(delta_t > min_time_delta)
            last_measurement_time_usec = sample_time;

        predictionStep(delta_t);
    }

    /**
     * Calls the predictionStepImpl after checking the delta_t value.
     *
     * @throws runtime_error if delta_t is negative or greater then the allowed maximum.
     */
    void predictionStep(double delta_t)
    {
        // check delta time
        if(delta_t < 0.0)
        {
            throw std::runtime_error("Delta time is negative!");
        }
        else if(delta_t <= min_time_delta)
        {
            // delta time is zero or close to zero
            return;
        }
        else if(delta_t > max_time_delta)
        {
            throw std::runtime_error("Delta time is greater then the allowed maximum!");
        }

        predictionStepImpl(delta_t);
    }

    unsigned getStateSize() const {return unsigned(WState::DOF);}
    bool isInitialized() const {return ukf.get() != NULL;}
    const Covariance& getProcessNoiseCovariance() const {return process_noise_cov;}
    void setProcessNoiseCovariance(const Covariance& noise_cov) {process_noise_cov = noise_cov;}
    int64_t getLastMeasurementTime() const {return last_measurement_time_usec;}
    void setLastMeasurementTime(int64_t last_measurement_time)
                               {this->last_measurement_time_usec = last_measurement_time;}
    double getMaxTimeDelta() const {return max_time_delta;}
    void setMaxTimeDelta(double max_time_delta) {this->max_time_delta = max_time_delta;}
    double getMinTimeDelta() const {return min_time_delta;}
    void setMinTimeDelta(double min_time_delta) {this->min_time_delta = min_time_delta;}

protected:
    virtual void predictionStepImpl(double delta_t) = 0;

    template<int DIM, typename scalar_type>
    void checkMeasurment(const Eigen::Matrix<scalar_type, DIM, 1>& mu, const Eigen::Matrix<scalar_type, DIM, DIM>& cov) const
    {
        if(!mu.allFinite() || !cov.allFinite())
            throw std::runtime_error("Measurement or covariance contains non-finite values!");
    }

protected:
    boost::shared_ptr<MTK_UKF> ukf;
    Covariance process_noise_cov;
    int64_t last_measurement_time_usec;
    double max_time_delta;
    double min_time_delta;
};

}

#endif
