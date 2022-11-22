cmake_minimum_required(VERSION 3.5)
project(pose_estimation VERSION "0.1.0")

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")

include(GNUInstallDirs)
include(CMakeDependentOption)
include(CMakePackageConfigHelpers)
include(src)

option(PACKAGE_TESTS "Build the tests" ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release")
endif()

find_package(PkgConfig)

# pkg_check_modules(Eigen3 REQUIRED eigen3)
find_package(Eigen3 REQUIRED)
find_package(GDAL REQUIRED)

add_library(pose_estimation SHARED
  POSE_ESTIMATION_SOURCES
)
target_include_directories(pose_estimation PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/include/${PROJECT_NAME}>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}> # <prefix>/include/${PROJECT_NAME}
)

target_link_libraries(pose_estimation PUBLIC Eigen3::Eigen GDAL::GDAL mtk)

target_compile_features(pose_estimation PUBLIC cxx_std_17)

# set_target_properties(uwv_docking PROPERTIES POSITION_INDEPENDENT_CODE ON) # This is necessery for whatever reason

# set_target_properties(uwv_docking PROPERTIES OUTPUT_NAME uwv_docking) # Do I need this?
# if(PACKAGE_TESTS)
# enable_testing()
# include(AddGTest)
# include(GoogleTest)
# add_subdirectory(tests)
# endif()
set(${PROJECT_NAME}_INSTALL_CMAKEDIR "${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}" CACHE STRING "Path to install ${PROJECT_NAME} Config*.cmake files to.")
set(${PROJECT_NAME}_MODULE_INSTALL_DIR "${CMAKE_INSTALL_LIBDIR}/cmake" CACHE STRING "Path to install ${PROJECT_NAME}'s .cmake  module files to.")

write_basic_package_version_file(
  ${PROJECT_NAME}ConfigVersion.cmake # The name of the version file needed by find_package.
  VERSION ${PROJECT_VERSION} # The version of the project, already set by the `project` command at the top of this file
  COMPATIBILITY SameMajorVersion # We use semantic versioning, backward compatibity is only guaranteed for a same major version
)
configure_package_config_file(
  ${PROJECT_SOURCE_DIR}/cmake/${PROJECT_NAME}Config.cmake.in # This is your template file
  ${PROJECT_BINARY_DIR}/${PROJECT_NAME}Config.cmake # This is the resulting file
  INSTALL_DESTINATION ${${PROJECT_NAME}_INSTALL_CMAKEDIR} # This is where the file will be installed

  # List of paths that needs to be relocated once installed
  # For example if the variable containing the path is named MY_PATH, all instances of @PACKAGE_MY_PATH@ in the template will be replaced by the relocated version of the path
  # This is mostly only needed when you want to install cmake modules or have an unusual layout that cmake is not aware of.
  PATH_VARS ${PROJECT_NAME}_MODULE_INSTALL_DIR # This will be exposed as @PACKAGE_BoilerPlate_MODULE_INSTALL_DIR@ in the template file

  # Imported targets do not require the following macros
  NO_SET_AND_CHECK_MACRO
  NO_CHECK_REQUIRED_COMPONENTS_MACRO
)
install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
  FILES_MATCHING PATTERN "*.hpp"
)

install(TARGETS pose_estimation
  EXPORT ${PROJECT_NAME}_Targets
  LIBRARY DESTINATION ${CMAKE_INSTALL_BINDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_LIBDIR}
  INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(EXPORT ${PROJECT_NAME}_Targets
  NAMESPACE ${PROJECT_NAME}::
  FILE ${PROJECT_NAME}Targets.cmake
  DESTINATION ${${PROJECT_NAME}_INSTALL_CMAKEDIR}
)

install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
  DESTINATION
  ${${PROJECT_NAME}_INSTALL_CMAKEDIR}
)

# SET(PKG_CONFIG_EXPORT_LIBS "-luwv_docking")
set(PKG_REQUIRES "eigen3 GDAL mtk")

# SET(PKG_CONFIG_INCLUDE_DIRS "-I")
configure_file(${PROJECT_NAME}.pc.in ${PROJECT_NAME}.pc @ONLY)
install(FILES ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.pc
  DESTINATION lib/pkgconfig
)