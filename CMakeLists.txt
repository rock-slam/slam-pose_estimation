cmake_minimum_required(VERSION 3.5)
project(pose_estimation VERSION "0.1.0")

set(CMAKE_CONFIG_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CONFIG_PATH}")

set(pose_estimation_LIBRARY ${PROJECT_NAME})

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "Setting build type to 'Release' as none was specified.")
  set(CMAKE_BUILD_TYPE "Release" CACHE
    STRING "Choose the type of build." FORCE)

  # Set the possible values of build type for cmake-gui
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS
    "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

if(POLICY CMP0057)
  cmake_policy(SET CMP0057 NEW)
endif()

# include(AddMTK)
# add_subdirectory(extern/mtk)
option(PACKAGE_TESTS "Build the tests" ON)

find_package(ament_cmake QUIET)

if(ament_cmake_FOUND)
  add_definitions(-DUSING_ROS2)
  message(STATUS "------------------------------------------")
  message(STATUS "BehaviourTree is being built using AMENT.")
  message(STATUS "------------------------------------------")
  include(cmake/ament_build.cmake)

elseif(CATKIN_DEVEL_PREFIX OR CATKIN_BUILD_BINARY_PACKAGE)
  add_definitions(-DUSING_ROS)
  message(STATUS "------------------------------------------")
  message(STATUS "BehaviourTree is being built using CATKIN.")
  message(STATUS "------------------------------------------")
  include(cmake/catkin_build.cmake)
  set(catkin_FOUND TRUE)
else()
  message(STATUS "------------------------------------------")
  message(STATUS "BehaviourTree is being built with conan.")
  message(STATUS "------------------------------------------")
  include(cmake/conan_build.cmake)
endif()

list(APPEND pose_estimation_SOURCE
  src/GeographicProjection.cpp
  src/pose_with_velocity/PoseUKF.cpp
  src/orientation_estimator/OrientationUKF.cpp
)
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON) # ? - https://github.com/BehaviorTree/BehaviorTree.CPP/blob/master/CMakeLists.txt#L147
add_library(${pose_estimation_LIBRARY} SHARED
  ${pose_estimation_SOURCE}
)

target_link_libraries(${pose_estimation_LIBRARY}
  PRIVATE
  ${CMAKE_DL_LIBS}
  ${pose_estimation_EXTRA_LIBRARIES}
)

target_include_directories(${pose_estimation_LIBRARY}
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  PRIVATE
  ${pose_estimation_EXTRA_INCLUDE_DIRS}
)

target_compile_definitions(${pose_estimation_LIBRARY} PUBLIC pose_estimation_LIBRARY_VERSION="${CMAKE_PROJECT_VERSION}")

target_compile_features(${pose_estimation_LIBRARY} PUBLIC cxx_std_17)

INSTALL(TARGETS ${pose_estimation_LIBRARY}
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION ${pose_estimation_LIB_DESTINATION}
  LIBRARY DESTINATION ${pose_estimation_LIB_DESTINATION}
  RUNTIME DESTINATION ${pose_estimation_BIN_DESTINATION}
  INCLUDES DESTINATION ${pose_estimation_INCLUDE_DESTINATION}
)

INSTALL(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/
  DESTINATION ${pose_estimation_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h*")

export_pose_estimation_package()