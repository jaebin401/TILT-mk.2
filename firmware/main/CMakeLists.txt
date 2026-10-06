idf_component_register(
    SRCS "main.cpp"
    INCLUDE_DIRS "."
    REQUIRES tilt_config tilt_sts3215 esp_driver_usb_serial_jtag freertos
)
