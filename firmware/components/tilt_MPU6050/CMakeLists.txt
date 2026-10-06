idf_component_register(
    SRCS "mpu6050.cpp" "complementary.cpp"
    INCLUDE_DIRS "include"
    REQUIRES tilt_config esp_driver_i2c
)
