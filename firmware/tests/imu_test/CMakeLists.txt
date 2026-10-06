cmake_minimum_required(VERSION 3.16)

set(IDF_TARGET "esp32s3")

set(EXTRA_COMPONENT_DIRS
    "${CMAKE_CURRENT_LIST_DIR}/../../components/tilt_config"
    "${CMAKE_CURRENT_LIST_DIR}/../../components/tilt_MPU6050"
)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(tilt_imu_test)
