#ifndef _POSE_ESTIMATION_EULER_CONVERSION_HPP
#define _POSE_ESTIMATION_EULER_CONVERSION_HPP

#include <Eigen/Geometry>
#include <base/Eigen.hpp>
#include <base/Pose.hpp>

class EulerConversion
{
public:
    static void eulerAngleVelocityToAngleAxis(const Eigen::Vector3d &euler, Eigen::Vector3d &angle_axis)
    {
        Eigen::AngleAxisd angle_axis_d = Eigen::AngleAxisd(Eigen::AngleAxisd(euler.z(), Eigen::Vector3d::UnitZ()) *
                                                        Eigen::AngleAxisd(euler.y(), Eigen::Vector3d::UnitY()) *
                                                        Eigen::AngleAxisd(euler.x(), Eigen::Vector3d::UnitX()));
        angle_axis = angle_axis_d.angle() * angle_axis_d.axis();
    }

    static void angleAxisToEulerAngleVelocity(const Eigen::Vector3d &angle_axis, Eigen::Vector3d &euler)
    {
        base::Vector3d euler_angle_velocity(0.0,0.0,0.0);
        if(!angle_axis.isZero())
            euler_angle_velocity = base::getEuler(base::Orientation(Eigen::AngleAxisd(angle_axis.norm(), angle_axis.normalized())));
        euler = Eigen::Vector3d(euler_angle_velocity.z(), euler_angle_velocity.y(), euler_angle_velocity.x());
    }

    static void quadToEuler(const base::Orientation &orientation, Eigen::Vector3d &euler)
    {
        base::Vector3d euler_angles = base::getEuler(orientation);
        euler = Eigen::Vector3d(euler_angles.z(), euler_angles.y(), euler_angles.x());
    }

    static void eulerToQuad(const Eigen::Vector3d euler, base::Orientation &orientation)
    {
        orientation = Eigen::Quaterniond(Eigen::AngleAxisd(euler.z(), Eigen::Vector3d::UnitZ()) *
                                        Eigen::AngleAxisd(euler.y(), Eigen::Vector3d::UnitY()) *
                                        Eigen::AngleAxisd(euler.x(), Eigen::Vector3d::UnitX()));
    }
};

#endif