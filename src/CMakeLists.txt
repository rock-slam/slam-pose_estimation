set (POSE_ESTIMATION_HEADERS
    mtk_ukf/PoseWithVelocity.hpp
    mtk_ukf/ProcessModels.hpp
    mtk_ukf/MeasurementModels.hpp
    PoseEstimator.hpp
    PoseEstimatorConfig.hpp
    AbstractFilter.hpp
    Measurement.hpp
    PoseUKF.hpp
    )
    
set (POSE_ESTIMATION_SOURCES
    PoseEstimator.cpp
    PoseUKF.cpp
    )

rock_library(pose_estimation
    SOURCES ${POSE_ESTIMATION_SOURCES}
    HEADERS ${POSE_ESTIMATION_HEADERS}
    DEPS_PKGCONFIG eigen3 base-types base-lib mtk
    DEPS_CMAKE LAPACK)
    
