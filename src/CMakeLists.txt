set (POSE_ESTIMATION_HEADERS
    PoseEstimator.hpp
    AbstractFilter.hpp
    Measurement.hpp
    Exceptions.hpp
    EulerConversion.hpp
    UKF.hpp
    ManifoldHelper.hpp
    pose_with_velocity/PoseWithVelocity.hpp
    pose_with_velocity/PoseUKF.hpp
    pose_with_velocity/BodyStateMeasurement.hpp
    )
    
set (POSE_ESTIMATION_SOURCES
    PoseEstimator.cpp
    AbstractFilter.cpp
    pose_with_velocity/PoseUKF.cpp
    )

rock_library(pose_estimation
    SOURCES ${POSE_ESTIMATION_SOURCES}
    HEADERS ${POSE_ESTIMATION_HEADERS}
    DEPS_PKGCONFIG eigen3 base-types base-lib mtk
    DEPS_CMAKE LAPACK)
    
