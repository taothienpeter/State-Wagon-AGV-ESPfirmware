idf_component_register(SRCS "main.cpp"
                    INCLUDE_DIRS "."
                    REQUIRES "comm" "stm_link" "kinematics" "trajectory" "imu" "uwb" "es_ekf"
                    PRIV_REQUIRES "nvs_flash" "esp_wifi" "esp_event" "esp_netif" "lwip"
)
