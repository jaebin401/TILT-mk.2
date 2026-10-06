cmake_minimum_required(VERSION 3.16)

set(IDF_TARGET "esp32s3")

# Keep these paths relative so ESP-IDF does not have to reinterpret this
# repository's space-containing absolute path as a legacy space-separated list.
set(EXTRA_COMPONENT_DIRS
    "../../components/tilt_config"
    "../../components/tilt_sts3215"
)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(tilt_pose_test)
