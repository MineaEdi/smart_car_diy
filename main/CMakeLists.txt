idf_component_register(
    SRCS    "hello_world_main.c" 
            "ble_server.c"
    PRIV_REQUIRES spi_flash driver esp_timer
    REQUIRES bt nvs_flash
    INCLUDE_DIRS ".")
