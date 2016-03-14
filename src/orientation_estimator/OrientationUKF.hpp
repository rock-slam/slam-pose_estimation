#ifndef _POSE_ESTIMATION_ORIENTATION_UKF_HPP
#define _POSE_ESTIMATION_ORIENTATION_UKF_HPP

#include "OrientationState.hpp"
#include <pose_estimation/Measurement.hpp>
#include <pose_estimation/UKF.hpp>
#include <map>
#include <boost/graph/graph_concepts.hpp>
#include <Eigen/Core>

namespace orientation
{

class OrientationUKF : public pose_estimation::UKF<pose_estimation::OrientationState>
{
public:
    OrientationUKF(const FilterState& initial_state);
    virtual ~OrientationUKF() {}
    
    virtual void predictionStep(const double delta);
    void setLatitude(const double latitude);
    void setIMUParams(const double gyro_var, const double acc_var, const double gyro_bias_std, 
		      const double acc_bias_std, const double gyro_bias_tau, const double acc_bias_tau);

    
    
protected:
    virtual void correctionStepUser(const pose_estimation::Measurement& measurement);

    virtual void muToUKFState(const FilterState::Mu &mu, WState& state) const;
    virtual void UKFStateToMu(const WState& state, FilterState::Mu &mu) const;

protected:
    std::map<std::string, pose_estimation::Measurement> latest_measurements;
    
    struct filter_params {
      double gyro_var;
      double acc_var;
      double gyro_bias_std;
      double acc_bias_std;
      double gyro_bias_tau;
      double acc_bias_tau;
      double gyro_bias_var;
      double acc_bias_var;
      double Latitude; // in radians
      Eigen::Vector3d Earth_rotation;
      Eigen::Vector3d Gravity;
    } params;
    
    typedef filter_params ParamsType;    
};

}

#endif