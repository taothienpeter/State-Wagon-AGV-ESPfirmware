idf_component_register(
    SRCS "dwm1000_driver.cpp"
    INCLUDE_DIRS "."
    REQUIRES "esp_driver_spi" "esp_driver_gpio"
)
