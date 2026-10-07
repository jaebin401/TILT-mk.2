cmake_minimum_required(VERSION 3.16)
project(tilt_rocking_bridge CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(tilt_rocking_gait SHARED
    rocking_gait_bridge.cpp
    ../firmware/components/tilt_gait/RockingGait.cpp)

target_include_directories(tilt_rocking_gait PRIVATE
    ../firmware/components/tilt_gait/include
    ../firmware/components/tilt_config/include)
