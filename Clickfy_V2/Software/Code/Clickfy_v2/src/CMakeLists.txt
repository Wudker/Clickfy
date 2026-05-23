idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES
        bt
        nvs_flash
        esp_event
        esp_system
        esp_hw_support
        driver
        freertos
        log
)