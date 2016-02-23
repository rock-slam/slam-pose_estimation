#ifndef _POSE_ESTIMATION_UKF_HPP
#define _POSE_ESTIMATION_UKF_HPP

#include <pose_estimation/AbstractFilter.hpp>
#include <pose_estimation/Exceptions.hpp>
#include <pose_estimation/ManifoldHelper.hpp>
#include <iostream>
#include <ukfom/ukf.hpp>
#include <ukfom/mtkwrap.hpp>
#include <boost/shared_ptr.hpp>
#include <base/Logging.hpp>

namespace pose_estimation
{

/**
*  The mapping defines which subset is returned.
*/
template <typename MTKStateType>
Eigen::Matrix<typename MTKStateType::scalar, -1, 1>
measurementModelStateMapping(const MTKStateType &state, const Eigen::Matrix<unsigned, -1, 1>& mapping)
{
    Eigen::Matrix<typename MTKStateType::scalar, MTKStateType::DOF, 1> state_vector;
    getStateVector(state, state_vector);

    Eigen::Matrix<typename MTKStateType::scalar, -1, 1> sub_state_vector;
    assert(mapping.rows() <= MTKStateType::DOF);
    sub_state_vector.resize(mapping.rows());
    for(unsigned i = 0; i < mapping.rows(); i++)
    {
        sub_state_vector(i) = state_vector(mapping(i));
    }
    return sub_state_vector;
}


template<typename Manifold>
class UKF : public AbstractFilter
{
public:
    typedef Manifold State;
    typedef ukfom::mtkwrap<Manifold> WState;
    typedef ukfom::ukf<WState> MTK_UKF;
    typedef typename MTK_UKF::cov Covariance;
    template <typename Scalar>
    struct MahalanobisDistance
    {
        typedef bool (*Func)(const Scalar& s);
    };

    virtual ~UKF() {}

    virtual unsigned getStateSize() const
    {
        return unsigned(State::DOF);
    }

    virtual bool getCurrentState(FilterState& filter_state)
    {
        if(ukf.get() != NULL)
        {
            UKFStateToMu(ukf->mu(), filter_state.mu);
            filter_state.cov = ukf->sigma();
            return true;
        }
        return false;
    }

protected:
    UKF()
    {
        // accept any mahalanobis distance per default
        allowed_distance = &ukfom::accept_any_mahalanobis_distance<typename MTK_UKF::scalar_type>;
    }

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

    virtual void correctionStepImpl(const Measurement& measurement)
    {
        if(measurement.integration == StateMapping && measurement.state_mapping.rows() != 0)
        {
            ukf->update(measurement.mu, boost::bind(measurementModelStateMapping<State>, _1, measurement.state_mapping),
                        boost::bind(ukfom::id< Eigen::MatrixXd >, measurement.cov),
                        allowed_distance);
            return;
        }
        else if(measurement.integration == StateMapping)
        {
            LOG_ERROR_S << "Selected state mapping based integration, but state mapping is empty! Falling back to user defined correction step.";
        }
        // call user defined correction step
        correctionStepUser(measurement);
    }

    virtual void correctionStepUser(const Measurement& measurement) = 0;

    virtual void muToUKFState(const FilterState::Mu &mu, WState& state) const = 0;
    virtual void UKFStateToMu(const WState& state, FilterState::Mu &mu) const = 0;

protected:
    typename MTK_UKF::cov process_noise_cov;
    boost::shared_ptr<MTK_UKF> ukf;
    typename MahalanobisDistance<typename MTK_UKF::scalar_type>::Func allowed_distance;
};

}

#endif