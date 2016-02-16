#ifndef _POSE_ESTIMATION_UKF_HPP
#define _POSE_ESTIMATION_UKF_HPP

#include <pose_estimation/AbstractFilter.hpp>
#include <iostream>
#include <ukfom/ukf.hpp>
#include <ukfom/mtkwrap.hpp>
#include <boost/shared_ptr.hpp>
#include <pose_estimation/Exceptions.hpp>

namespace pose_estimation
{

template<typename Manifold>
class UKF : public AbstractFilter
{
public:
    typedef Manifold State;
    typedef ukfom::mtkwrap<Manifold> WState;
    typedef ukfom::ukf<WState> MTK_UKF;
    typedef typename MTK_UKF::cov Covariance;

    virtual ~UKF() {}

    virtual unsigned getStateSize() const
    {
        return unsigned(State::DOF);
    }

    virtual const FilterState& getCurrentState()
    {
        if(ukf.get() != NULL)
        {
            UKFStateToMu(ukf->mu(), current_state.mu);
            current_state.cov = ukf->sigma();
        }
        return current_state;
    }

protected:
    UKF() {}

    virtual void setProcessNoiseCovarianceImpl(const FilterState::Cov& noise_cov)
    {
        process_noise_cov = noise_cov;
    }

    virtual void setInitialStateImpl(const FilterState& initial_state)
    {
        WState state;
        muToUKFState(initial_state.mu, state);
        ukf.reset(new MTK_UKF(state, initial_state.cov));
    }

    virtual void muToUKFState(const FilterState::Mu &mu, WState& state) const = 0;
    virtual void UKFStateToMu(const WState& state, FilterState::Mu &mu) const = 0;

protected:
    typename MTK_UKF::cov process_noise_cov;
    boost::shared_ptr<MTK_UKF> ukf;

private:
    FilterState current_state;
};

}

#endif