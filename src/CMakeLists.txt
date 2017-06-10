set (POSE_ESTIMATION_HEADERS
    Measurement.hpp
    UnscentedKalmanFilter.hpp
    StreamAlignmentVerifier.hpp
    pose_with_velocity/PoseWithVelocity.hpp
    pose_with_velocity/PoseUKF.hpp
    pose_with_velocity/PoseUKFConfig.hpp
    pose_with_velocity/BodyStateMeasurement.hpp
    )
    
set (POSE_ESTIMATION_SOURCES
    StreamAlignmentVerifier.cpp
    pose_with_velocity/PoseUKF.cpp
    )

rock_library(pose_estimation
    SOURCES ${POSE_ESTIMATION_SOURCES}
    HEADERS ${POSE_ESTIMATION_HEADERS}
    DEPS_PKGCONFIG eigen3 base-types base-lib mtk aggregator
    DEPS_CMAKE LAPACK)
    
