idf_component_register(
    SRCS 
        "ESP32-HUB75-MatrixPanel-I2S-DMA.cpp"
        "ESP32-HUB75-MatrixPanel-leddrivers.cpp"
        "gdma_lcd_parallel16.cpp"
    INCLUDE_DIRS "."
    REQUIRES driver esp_lcd adafruit_gfx
    PRIV_REQUIRES spi_flash esp_driver_gpio
)
