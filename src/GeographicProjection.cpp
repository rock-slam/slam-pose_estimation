#include "GeographicProjection.hpp"
#include <ogr_spatialref.h>

using namespace pose_estimation;

GeographicProjection::GeographicProjection(double latitude, double longitude, double x, double y) : offset(x, y)
{
    OGRSpatialReference world;
    OGRSpatialReference navigation; // transverse mercator plane

    world.SetWellKnownGeogCS( "WGS84" );

    navigation.SetProjCS( "local transverse mercator" );
    navigation.SetWellKnownGeogCS( "WGS84" );
    navigation.SetTM(latitude, longitude, 0.9996, 0., 0.);

    world2nav = OGRCreateCoordinateTransformation(&world, &navigation);
    nav2world = OGRCreateCoordinateTransformation(&navigation, &world);
}

GeographicProjection::~GeographicProjection()
{
    delete world2nav;
    delete nav2world;
}

bool GeographicProjection::worldToNav(double latitude, double longitude, double& x, double& y)
{
    x = latitude;
    y = longitude;
    bool r = world2nav->Transform(1, &y, &x) > 0;
    x = offset.x() + x;
    y = offset.y() - y;
    return r;
}

bool GeographicProjection::navToWorld(double x, double y, double& latitude, double& longitude)
{
    latitude = x - offset.x();
    longitude = offset.y() - y;
    return nav2world->Transform(1, &longitude, &latitude) > 0;
}