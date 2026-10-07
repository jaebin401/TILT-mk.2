cmake_minimum_required(VERSION 3.16)

set(IDF_TARGET "esp32s3")

set(EXTRA_COMPONENT_DIRS
    "../../components/tilt_config"
    "../../components/tilt_sts3215"
    "../../components/tilt_MPU6050"
    "../../components/tilt_gait"
)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(tilt_rocking_test)
