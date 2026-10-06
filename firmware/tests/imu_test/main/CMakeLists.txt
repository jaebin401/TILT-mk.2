idf_component_register(
    SRCS "main.cpp"
    INCLUDE_DIRS "."
    REQUIRES tilt_MPU6050 esp_timer freertos
)
