idf_component_register(
    SRCS "app_main.cpp"
         "arduino_sketch.cpp"
    EMBED_TXTFILES "APT.setup"
    INCLUDE_DIRS "."
    REQUIRES arduino-esp32 esp_timer esp_netif esp_wifi esp_event esp_http_server bt nvs_flash
)
