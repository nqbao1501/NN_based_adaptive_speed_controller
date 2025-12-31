idf_component_register(SRCS "web_dashboard.c" "external_setpoint.c" "NN_based_speed_controller.c" "setpoint_task.c" "nn_controller.c" 
                    PRIV_REQUIRES esp_driver_ledc esp_driver_gpio esp_adc esp_timer esp_driver_uart esp_driver_pcnt esp_wifi esp_event esp_netif nvs_flash esp_http_server
                    INCLUDE_DIRS ".")
