idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES driver esp_lcd esp_wifi esp_netif esp_event nvs_flash lwip esp_timer
)
