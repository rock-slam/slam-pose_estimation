#include <boost/test/unit_test.hpp>
#include <pose_estimation/Dummy.hpp>

using namespace pose_estimation;

BOOST_AUTO_TEST_CASE(it_should_not_crash_when_welcome_is_called)
{
    pose_estimation::DummyClass dummy;
    dummy.welcome();
}
