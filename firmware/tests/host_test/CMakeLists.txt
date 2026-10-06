cmake_minimum_required(VERSION 3.16)
project(tilt_host_test CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_executable(test_kinematics
    test_kinematics.cpp
    ../../components/tilt_kinematics/kinematics.cpp)

target_include_directories(test_kinematics PRIVATE
    ../../components/tilt_kinematics/include
    ../../components/tilt_config/include)

add_executable(test_filter
    test_filter.cpp
    ../../components/tilt_MPU6050/complementary.cpp)

target_include_directories(test_filter PRIVATE
    ../../components/tilt_MPU6050/include
    ../../components/tilt_config/include)
