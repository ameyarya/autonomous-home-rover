idf_component_register(SRCS "main.c" "cam.c" "mic.c" "frames.c"
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES esp_wifi esp_netif esp_event esp_timer esp_http_server nvs_flash lwip
                                     app_update esp_app_format esp_driver_gpio esp_driver_i2s)
