list(APPEND CMAKE_PREFIX_PATH "${CMAKE_BINARY_DIR}")

find_package(Eigen3 REQUIRED)
list(APPEND pose_estimation_EXTRA_LIBRARIES Eigen3::Eigen)
message(STATUS "EIGEN3_LIBRARIES: Eigen3::Eigen")
find_package(GDAL REQUIRED)
list(APPEND pose_estimation_EXTRA_LIBRARIES GDAL::GDAL)
message(STATUS "GDAL_LIBRARIES: GDAL::GDAL")
find_package(mtk REQUIRED)
list(APPEND pose_estimation_EXTRA_LIBRARIES mtk::mtk)
message(STATUS "mtk_LIBRARIES: mtk::mtk")

set(pose_estimation_LIB_DESTINATION lib)
set(pose_estimation_INCLUDE_DESTINATION include)
set(pose_estimation_BIN_DESTINATION bin)

mark_as_advanced(
  pose_estimation_EXTRA_LIBRARIES
  pose_estimation_LIB_DESTINATION
  pose_estimation_INCLUDE_DESTINATION
  pose_estimation_BIN_DESTINATION)

macro(export_pose_estimation_package)
  install(EXPORT ${PROJECT_NAME}Targets
    FILE "${PROJECT_NAME}Targets.cmake"
    DESTINATION "${pose_estimation_LIB_DESTINATION}/cmake/${PROJECT_NAME}"
    NAMESPACE pose_estimation::
  )
  export(PACKAGE ${PROJECT_NAME})

  include(CMakePackageConfigHelpers)

  configure_package_config_file(
    "${PROJECT_SOURCE_DIR}/cmake/Config.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
    INSTALL_DESTINATION "${pose_estimation_LIB_DESTINATION}/cmake/${PROJECT_NAME}"
  )

  install(
    FILES
    "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
    DESTINATION "${pose_estimation_LIB_DESTINATION}/cmake/${PROJECT_NAME}"
  )
endmacro()