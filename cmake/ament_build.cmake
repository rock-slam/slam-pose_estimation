# ---- Add the subdirectory cmake ----
set(CMAKE_CONFIG_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/cmake")
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CONFIG_PATH}")

find_package(Eigen3 REQUIRED)
find_package(GDAL REQUIRED)
find_package(mtk REQUIRED)

find_package(ament_index_cpp REQUIRED)

set(pose_estimation_EXTRA_LIBRARIES
  $<BUILD_INTERFACE:ament_index_cpp::ament_index_cpp>
  Eigen3::Eigen
  GDAL::GDAL
  mtk::mtk
)

ament_export_dependencies(ament_index_cpp)

set(pose_estimation_LIB_DESTINATION lib)
set(pose_estimation_INCLUDE_DESTINATION include)
set(pose_estimation_BIN_DESTINATION bin)

mark_as_advanced(
  pose_estimation_EXTRA_LIBRARIES
  pose_estimation_LIB_DESTINATION
  pose_estimation_INCLUDE_DESTINATION
  pose_estimation_BIN_DESTINATION)

macro(export_pose_estimation_package)
  ament_export_include_directories(include)
  ament_export_libraries(${pose_estimation_LIBRARY})
  ament_package()
endmacro()