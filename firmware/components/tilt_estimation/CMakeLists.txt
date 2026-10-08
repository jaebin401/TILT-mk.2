idf_component_register(
    SRCS "CenterOfMass.cpp" "DcmEstimator.cpp"
    INCLUDE_DIRS "include"
    REQUIRES tilt_kinematics tilt_config
)
