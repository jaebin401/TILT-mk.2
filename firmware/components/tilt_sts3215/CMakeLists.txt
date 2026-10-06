idf_component_register(
    SRCS "Sts3215Bus.cpp"
    INCLUDE_DIRS "include"
    REQUIRES esp_driver_gpio esp_driver_uart freertos
)
