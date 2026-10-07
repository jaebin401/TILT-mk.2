idf_component_register(
    SRCS "main.cpp"
    INCLUDE_DIRS "."
    REQUIRES
        tilt_config
        tilt_sts3215
        tilt_MPU6050
        tilt_gait
        esp_driver_usb_serial_jtag
        esp_timer
        freertos
)
