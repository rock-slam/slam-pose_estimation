#include <boost/test/unit_test.hpp>
#include <pose_estimation/GeographicProjection.hpp>
#include <Eigen/Core>
#include <ogr_spatialref.h>

using namespace pose_estimation;

BOOST_AUTO_TEST_CASE(test_coordinate_projection)
{
    // DFKI Bremen
    double latitude = 0.92698121;
    double longitude = 0.154595663;
    GeographicProjection projection(latitude, longitude);

    // test identity
    Eigen::Vector2d pos;
    BOOST_CHECK(projection.worldToNav(latitude, longitude, pos.x(), pos.y()));

    BOOST_CHECK(pos.x() == 0);
    BOOST_CHECK(pos.y() == 0);

    double latitude2, longitude2;
    BOOST_CHECK(projection.navToWorld(pos.x(), pos.y(), latitude2, longitude2));

    BOOST_CHECK(latitude2 == latitude);
    BOOST_CHECK(longitude2 == longitude);

    // test with pos offset
    GeographicProjection projection2(latitude, longitude, -500., 1234.);

    BOOST_CHECK(projection2.worldToNav(latitude, longitude, pos.x(), pos.y()));

    BOOST_CHECK(pos.x() == -500.);
    BOOST_CHECK(pos.y() == 1234.);

    // inverse
    BOOST_CHECK(projection2.navToWorld(pos.x(), pos.y(), latitude2, longitude2));

    BOOST_CHECK(latitude2 == latitude);
    BOOST_CHECK(longitude2 == longitude);

    // add 0.1 degree in gps frame
    Eigen::Vector2d pos2;
    BOOST_CHECK(projection2.worldToNav(latitude + 0.1, longitude + 0.1, pos2.x(), pos2.y()));

    BOOST_CHECK(pos2.x() > pos.x());
    BOOST_CHECK(pos2.y() < pos.y());

    // substract 10000m in nav frame
    BOOST_CHECK(projection2.navToWorld(pos.x() - 10000, pos.y() - 10000, latitude2, longitude2));

    BOOST_CHECK(latitude2 < latitude);
    BOOST_CHECK(longitude2 > longitude);
}