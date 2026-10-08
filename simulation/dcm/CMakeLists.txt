cmake_minimum_required(VERSION 3.16)
project(tilt_dcm_bridge CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(FW ${CMAKE_CURRENT_SOURCE_DIR}/../../firmware/components)

add_library(tilt_dcm SHARED
    dcm_estimator_bridge.cpp
    ${FW}/tilt_kinematics/kinematics.cpp
    ${FW}/tilt_estimation/CenterOfMass.cpp
    ${FW}/tilt_estimation/DcmEstimator.cpp)

target_include_directories(tilt_dcm PRIVATE
    ${FW}/tilt_estimation/include
    ${FW}/tilt_kinematics/include
    ${FW}/tilt_config/include)
