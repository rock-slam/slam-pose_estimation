set (POSE_ESTIMATION_HEADERS
    ros_ekf/ekf.hpp
    ros_ekf/filter_base.hpp
    ros_ekf/filter_common.hpp
    mtk_ukf/PoseWithVelocity.hpp
    mtk_ukf/ProcessModels.hpp
    mtk_ukf/MeasurementModels.hpp
    PoseEstimator.hpp
    PoseEstimatorConfig.hpp
    AbstractFilter.hpp
    Measurement.hpp
    PoseEKF.hpp
    PoseUKF.hpp
    )
    
set (POSE_ESTIMATION_SOURCES
    ros_ekf/ekf.cpp
    ros_ekf/filter_base.cpp
    PoseEstimator.cpp
    PoseEKF.cpp
    PoseUKF.cpp
    )

rock_library(pose_estimation
    SOURCES ${POSE_ESTIMATION_SOURCES}
    HEADERS ${POSE_ESTIMATION_HEADERS}
    DEPS_PKGCONFIG eigen3 base-types base-lib mtk
    DEPS_CMAKE LAPACK)
    
