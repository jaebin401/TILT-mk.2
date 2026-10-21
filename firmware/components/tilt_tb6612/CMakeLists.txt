idf_component_register(
    SRCS "Tb6612Driver.cpp"
    INCLUDE_DIRS "include"
    REQUIRES esp_driver_gpio esp_driver_mcpwm
)
