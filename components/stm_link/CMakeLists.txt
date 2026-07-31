idf_component_register(
    SRCS "stm_uart_link.cpp"
    INCLUDE_DIRS "." "protocol"
    REQUIRES "esp_driver_uart"
)
