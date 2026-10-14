idf_component_register(
    SRCS "DcmStepping.cpp" "GaitFeedback.cpp" "WalkStride.cpp"
    INCLUDE_DIRS "include"
    REQUIRES tilt_estimation tilt_kinematics tilt_config
)
