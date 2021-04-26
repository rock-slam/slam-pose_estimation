#ifndef _POSE_ESTIMATION_DELAYED_STATES_HPP_
#define _POSE_ESTIMATION_DELAYED_STATES_HPP_

#include <Eigen/Core>
#include <map>

namespace pose_estimation
{

/**
 * This class acts as a helper in order to store filter states at a given time t.
 * This is useful in the case of delayed measurement integration.
 * To integrate a delayed measurement the rows and colloms of the state at time t 
 * can be extracted and exchanged in the current filter state.
 */
template<typename State>
class DelayedStates
{
public:
    typedef Eigen::Matrix<typename State::scalar, int(State::DOF), int(State::DOF)> Cov;

    /**
     * @param window_size in seconds for which states are stored
     */
    DelayedStates(double window_size = 10.0) : window_size_usec(fromSeconds(window_size)) {}
    
    template<class Time>
    void pushState(const Time& ts, const State& mu, const Cov& sigma)
    {
        pushState(ts.microseconds, mu, sigma);
    }

    /**
     * Saves the state mu and its covariance sigma at time ts.
     * Also removes states older then window_size.
     * @param ts in microseconds
     */
    void pushState(int64_t ts, const State& mu, const Cov& sigma)
    {
        // remove states exceeding the window size
        State_it it_low = states.lower_bound(ts - window_size_usec);
        states.erase(states.begin(), it_low);

        // add new element
        DelayedState<State, Cov>& ds = states[ts];
        ds.mu = mu;
        ds.sigma = sigma;
    }
    
    template<class Time>
    bool getClosestState(const Time& ts, State& mu, Cov& sigma, double max_dist = 1.0) const
    {
        return getClosestState(ts.microseconds, mu, sigma, max_dist);
    }

    /**
     * Returns the closest state mu and its covariance sigma at time ts.
     * @param ts in microseconds
     * @param max_dist maximum distance in seconds
     * @returns true if a state could be found
     */
    bool getClosestState(int64_t ts, State& mu, Cov& sigma, double max_dist = 1.0) const
    {
        if(states.empty())
            return false;

        // identify adjacent elements
        State_it it_high = states.lower_bound(ts);
        State_it it_low = it_high;
        it_low--;

        // identify closest element
        State_it closest;
        if(it_high == states.begin())
            closest = it_high;
        else if(it_high == states.end())
            closest = it_low;
        else if((it_high->first - ts) < (ts - it_low->first))
            closest = it_high;
        else
            closest = it_low;

        // check max time delta
        if(std::abs(toSeconds(closest->first - ts)) > max_dist)
            return false;

        mu = closest->second.mu;
        sigma = closest->second.sigma;
        return true;
    }
    
    static int64_t fromSeconds(double value)
    {
        int64_t seconds = value;
        return seconds * UsecPerSec + static_cast<int64_t>(round((value - seconds) * UsecPerSec));
    }

    static double toSeconds(int64_t microseconds)
    {
        return static_cast<double>(microseconds) / UsecPerSec;
    }

protected:
    template<typename _State, typename _Cov>
    struct DelayedState
    {
        _State mu;
        _Cov sigma;
    };
    typedef typename std::map< int64_t, DelayedState<State, Cov> >::const_iterator State_it;

    std::map< int64_t, DelayedState<State, Cov> > states;
    int64_t window_size_usec;
    static const int UsecPerSec = 1000000LL;
};

}

#endif
