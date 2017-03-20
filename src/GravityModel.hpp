#ifndef _POSE_ESTIMATION_GRAVITY_MODEL_HPP
#define _POSE_ESTIMATION_GRAVITY_MODEL_HPP

#include <math.h>

namespace pose_estimation
{
    /** WGS-84 ellipsoid constants (Nominal Gravity Model and Earth angular velocity) **/
    static const double EQUATORIAL_RADIUS = 6378137.0; /** Equatorial radius in meters **/
    static const double ECC = 0.0818191908426; /** First eccentricity **/
    static const double GRAVITY = 9.79766542; /** Mean value of gravity value in m/s^2 according to WGS-84 **/
    static const double GRAVITY_SI = 9.80665; /** Mean value of gravity value in m/s^2 according to SI standard **/
    static const double GWGS0 = 9.7803267714; /** Gravity value at the equator in m/s^2 **/
    static const double GWGS1 = 0.00193185138639; /** Gravity formula constant **/
    static const double EARTHW = ((2.0*M_PI)/86164.0); //7.292115e-05; /** Earth angular velocity in rad/s **/


   /**
    * @brief This computes the theoretical gravity value according to the WGS-84 ellipsoid earth model.
    *
    * @author Javier Hidalgo Carrio.
    *
    * @param[in] latitude double the latitude value in radian
    * @param[in] altitude double with the altitude value in meters
    *
    * @return double. the theoretical value of the local gravity
    *
    */
    static double GravityModel (double latitude, double altitude)
    {
        double g; /**< g magnitude at zero altitude **/

        /** Nominal Gravity model **/
        g = GWGS0*((1+GWGS1*pow(sin(latitude),2))/sqrt(1-pow(ECC,2)*pow(sin(latitude),2)));

        /** Gravity affects by the altitude (aprox the value r = EQUATORIAL_RADIUS) **/
        g = g*pow(EQUATORIAL_RADIUS/(EQUATORIAL_RADIUS+altitude), 2);

        return g;
    }
}

#endif