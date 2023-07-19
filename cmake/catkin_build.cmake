# ---- Add the subdirectory cmake ----
set(CMAKE_CONFIG_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/cmake")
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CONFIG_PATH}")

find_package(Eigen3 REQUIRED)
find_package(GDAL REQUIRED)
find_package(mtk REQUIRED)

find_package(catkin REQUIRED COMPONENTS roslib)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${pose_estimation_LIBRARY}
  CATKIN_DEPENDS roslib)

set(pose_estimation_EXTRA_INCLUDE_DIRS ${catkin_INCLUDE_DIRS})

set(pose_estimation_EXTRA_LIBRARIES
  ${catkin_LIBRARIES}
  Eigen3::Eigen
  GDAL::GDAL
  mtk::mtk
)

set(pose_estimation_LIB_DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
set(pose_estimation_INCLUDE_DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})
set(pose_estimation_BIN_DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

mark_as_advanced(
  pose_estimation_EXTRA_LIBRARIES
  pose_estimation_EXTRA_INCLUDE_DIRS
  pose_estimation_LIB_DESTINATION
  pose_estimation_INCLUDE_DESTINATION
  pose_estimation_BIN_DESTINATION)

macro(export_pose_estimation_package)
  # do nothing
endmacro()