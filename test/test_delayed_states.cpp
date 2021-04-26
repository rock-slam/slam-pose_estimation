#include <boost/test/unit_test.hpp>
#include <pose_estimation/DelayedStates.hpp>
#include <Eigen/Core>
#include <iostream>
#include <mtk/src/SubManifold.hpp>
#include <mtk/types/SOn.hpp>
#include <mtk/types/vect.hpp>
#include <mtk/build_manifold.hpp>
#include <mtk/startIdx.hpp>
#include <ukfom/mtkwrap.hpp>
#include <ukfom/ukf.hpp>
#include <boost/random.hpp>
#include <boost/math/distributions/normal.hpp>
#include <time.h>
#include <sys/time.h>

using namespace pose_estimation;

BOOST_AUTO_TEST_CASE(test_delayed_states)
{
    // state set with a window size of 10 seconds
    DelayedStates< MTK::vect<2> > states(10.0);
    MTK::vect<2> mu_in, mu_out;
    Eigen::Matrix2d sigma_in, sigma_out;
    mu_in.setRandom();
    mu_out.setRandom();
    sigma_in.setRandom();
    sigma_out.setRandom();

    // time in microseconds
    timeval t;
    gettimeofday(&t, 0);
    int64_t ts = static_cast<int64_t>(t.tv_sec) * 1000000 + t.tv_usec;
    int64_t one_s = DelayedStates< MTK::vect<2> >::fromSeconds(1.0);

    // check empty case
    BOOST_CHECK(states.getClosestState(ts, mu_out, sigma_out) == false);

    // test one element
    states.pushState(ts, mu_in, sigma_in);
    BOOST_CHECK(states.getClosestState(ts, mu_out, sigma_out));
    BOOST_CHECK(mu_in.isApprox(mu_out));
    BOOST_CHECK(sigma_in.isApprox(sigma_out));
    mu_out.setRandom();
    sigma_out.setRandom();

    BOOST_CHECK(states.getClosestState(ts-one_s, mu_out, sigma_out));
    BOOST_CHECK(mu_in.isApprox(mu_out));
    BOOST_CHECK(sigma_in.isApprox(sigma_out));
    mu_out.setRandom();
    sigma_out.setRandom();

    BOOST_CHECK(states.getClosestState(ts+one_s, mu_out, sigma_out));
    BOOST_CHECK(mu_in.isApprox(mu_out));
    BOOST_CHECK(sigma_in.isApprox(sigma_out));
    mu_out.setRandom();
    sigma_out.setRandom();

    // test max distance
    BOOST_CHECK(states.getClosestState(ts+one_s, mu_out, sigma_out, 0.5) == false);

    // add one elemet before
    states.pushState(ts-one_s, Eigen::Vector2d::Random(), Eigen::Matrix2d::Random());
    BOOST_CHECK(states.getClosestState(ts, mu_out, sigma_out));
    BOOST_CHECK(mu_in.isApprox(mu_out));
    BOOST_CHECK(sigma_in.isApprox(sigma_out));
    mu_out.setRandom();
    sigma_out.setRandom();

    // add one element after
    states.pushState(ts+one_s, Eigen::Vector2d::Random(), Eigen::Matrix2d::Random());
    BOOST_CHECK(states.getClosestState(ts, mu_out, sigma_out));
    BOOST_CHECK(mu_in.isApprox(mu_out));
    BOOST_CHECK(sigma_in.isApprox(sigma_out));
    mu_out.setRandom();
    sigma_out.setRandom();

    BOOST_CHECK(states.getClosestState(ts + DelayedStates< MTK::vect<2> >::fromSeconds(0.1), mu_out, sigma_out));
    BOOST_CHECK(mu_in.isApprox(mu_out));
    BOOST_CHECK(sigma_in.isApprox(sigma_out));
    mu_out.setRandom();
    sigma_out.setRandom();

    BOOST_CHECK(states.getClosestState(ts - DelayedStates< MTK::vect<2> >::fromSeconds(0.1), mu_out, sigma_out));
    BOOST_CHECK(mu_in.isApprox(mu_out));
    BOOST_CHECK(sigma_in.isApprox(sigma_out));
    mu_out.setRandom();
    sigma_out.setRandom();

    // test window size
    states.pushState(ts+DelayedStates< MTK::vect<2> >::fromSeconds(20.0), Eigen::Vector2d::Random(), Eigen::Matrix2d::Random());
    BOOST_CHECK(states.getClosestState(ts, mu_out, sigma_out) == false);
    BOOST_CHECK(states.getClosestState(ts+DelayedStates< MTK::vect<2> >::fromSeconds(20.0), mu_out, sigma_out));
}



typedef ukfom::mtkwrap< MTK::vect<1> > StateType;

MTK_BUILD_MANIFOLD(TestState,
   ((StateType, state))
   ((StateType, delayed_state))
)

typedef ukfom::mtkwrap<TestState> WTestState;
typedef ukfom::ukf<WTestState> MTK_UKF;
typedef typename MTK_UKF::cov Covariance;

// process model
template <typename FilterState>
FilterState
processModel(const FilterState &state, const Eigen::Matrix<TestState::scalar, 1, 1>& derivative, double delta_time)
{
    FilterState new_state(state);

    // apply derivative
    new_state.state.boxplus(derivative, delta_time);

    // clone state
    new_state.delayed_state = state.state;

    return new_state;
}

// measurement model
template <typename FilterState>
Eigen::Matrix<TestState::scalar, 1, 1>
measurementModel(const FilterState &state)
{
    return state.state;
}

template <typename FilterState>
Eigen::Matrix<TestState::scalar, 1, 1>
delayedMeasurementModel(const FilterState &state)
{
    return state.delayed_state;
}

struct Filter
{
    Filter()
    {
        state.state << 0.;
        state.delayed_state << 0.;
        cov << 0.01, 0.,
            0., 0.01;
        ukf.reset(new MTK_UKF(state, cov));
        process_noise << 0.001, 0.,
                        0., 0.0;
        measurement_cov << 0.1;
        step = M_PI * 0.01;
        steps = 1000;
        var_nor.reset(new boost::variate_generator<boost::mt19937,
                            boost::normal_distribution<> >(boost::mt19937(),
                                                           boost::normal_distribution<>(0., std::sqrt(process_noise(0)))));
    }

    TestState state;
    Covariance cov;
    Covariance process_noise;
    Eigen::Matrix<TestState::scalar, 1, 1> measurement_cov;
    double step;
    unsigned steps;
    boost::shared_ptr<MTK_UKF> ukf;
    boost::shared_ptr<boost::variate_generator<boost::mt19937,
                            boost::normal_distribution<> > > var_nor;
};

BOOST_FIXTURE_TEST_CASE(test_non_delayed_state_integration, Filter)
{
    for(unsigned i = 0; i < steps; i++)
    {
        double current_t = (double)i * step;

        // non-delayed update
        if(i % 10 == 0)
        {
                Eigen::Matrix<TestState::scalar, 1, 1> measurement;
                measurement << sin(current_t);

                ukf->update(measurement, boost::bind(measurementModel<WTestState>, _1),
                            boost::bind(ukfom::id< Eigen::Matrix<TestState::scalar, 1, 1> >, measurement_cov));
        }

        // check if error to true state is inside 1sigma
        BOOST_CHECK(std::abs(ukf->mu().state(0) - sin(current_t) < ukf->sigma()(0)));

        // prediction step
        Eigen::Matrix<TestState::scalar, 1, 1> derivative;
        derivative << cos(current_t + step * 0.5) + (*var_nor)();
        ukf->predict(boost::bind(processModel<WTestState>, _1, derivative, step),
                    MTK_UKF::cov(process_noise));
    }
}

BOOST_FIXTURE_TEST_CASE(test_delayed_state_integration, Filter)
{
    unsigned max_delay = 10;
    DelayedStates<TestState> states(2.0 * (double)max_delay * step);

    std::srand(std::time(nullptr));

    for(unsigned i = 0; i < steps; i++)
    {
        double current_t = (double)i * step;

        // delayed update
        if(i % 10 == 0)
        {
            // delay between [1,9] steps
            unsigned delay = ((double)std::rand() / ((double)RAND_MAX * 0.125)) + 1.5;
            double delayed_t = current_t - (double)delay * step;

            if(delayed_t > 0.)
            {
                Eigen::Matrix<TestState::scalar, 1, 1> measurement;
                measurement << sin(delayed_t);

                TestState mu;
                Covariance cov;
                BOOST_CHECK(states.getClosestState(DelayedStates< TestState >::fromSeconds(delayed_t), mu, cov, step * 0.5));

                TestState new_state = ukf->mu();
                Covariance new_cov = ukf->sigma();
                new_state.delayed_state = mu.delayed_state;
                new_cov.block(0,1,2,1) = cov.block(0,1,2,1);
                new_cov.block(1,0,1,2) = cov.block(1,0,1,2);
                ukf.reset(new MTK_UKF(new_state, new_cov));

                ukf->update(measurement, boost::bind(delayedMeasurementModel<WTestState>, _1),
                            boost::bind(ukfom::id< Eigen::Matrix<TestState::scalar, 1, 1> >, measurement_cov));
            }
        }
        

        // check if error to true state is inside 1sigma
        BOOST_CHECK(std::abs(ukf->mu().state(0) - sin(current_t) < ukf->sigma()(0)));

        // prediction step
        Eigen::Matrix<TestState::scalar, 1, 1> derivative;
        derivative << cos(current_t + step * 0.5) + (*var_nor)();
        ukf->predict(boost::bind(processModel<WTestState>, _1, derivative, step),
                    MTK_UKF::cov(process_noise));

        // save filter state
        states.pushState(DelayedStates< TestState >::fromSeconds(current_t), ukf->mu(), ukf->sigma());
    }
}
