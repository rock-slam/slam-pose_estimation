#ifndef _POSE_ESTIMATION_MEASUREMENT_HPP
#define _POSE_ESTIMATION_MEASUREMENT_HPP

#include <Eigen/Core>

#define MEASUREMENT(NAME, DIM) \
struct NAME \
{ \
typedef Eigen::Matrix<double, DIM, 1> Mu; \
typedef Eigen::Matrix<double, DIM, DIM> Cov; \
 \
NAME() : mu(Mu::Zero()), cov(Cov::Identity()) {} \
 \
Mu mu; \
Cov cov; \
};


#endif