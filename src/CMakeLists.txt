rock_library(pose_estimation
    SOURCES Dummy.cpp
    HEADERS Dummy.hpp)

rock_executable(pose_estimation_bin Main.cpp
    DEPS pose_estimation)

