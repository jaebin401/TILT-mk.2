cmake_minimum_required(VERSION 3.16)

# 공용 컴포넌트(tilt_config, tilt_sts3215)를 firmware/components 에서 가져온다.
set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/../../firmware/components")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(tilt_servo_tool)
