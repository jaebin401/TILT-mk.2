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

add_executable(test_pose_test
    test_pose_test.cpp)

target_include_directories(test_pose_test PRIVATE
    ../pose_test/main
    ../../components/tilt_config/include)

add_executable(test_rocking_gait
    test_rocking_gait.cpp
    ../../components/tilt_gait/RockingGait.cpp)

target_include_directories(test_rocking_gait PRIVATE
    ../../components/tilt_gait/include
    ../../components/tilt_config/include)

add_executable(test_estimation
    test_estimation.cpp
    ../../components/tilt_kinematics/kinematics.cpp
    ../../components/tilt_estimation/CenterOfMass.cpp
    ../../components/tilt_estimation/DcmEstimator.cpp)

target_include_directories(test_estimation PRIVATE
    ../../components/tilt_estimation/include
    ../../components/tilt_kinematics/include
    ../../components/tilt_config/include)

add_executable(test_dcm_stepping
    test_dcm_stepping.cpp
    ../../components/tilt_dcm_gait/DcmStepping.cpp)

target_include_directories(test_dcm_stepping PRIVATE
    ../../components/tilt_dcm_gait/include
    ../../components/tilt_estimation/include
    ../../components/tilt_kinematics/include
    ../../components/tilt_config/include)
