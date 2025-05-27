idf_component_register(
    SRCS 
        "main.cpp"
        "ESP32-HUB75-MatrixPanel-I2S-DMA.cpp"
        "ESP32-HUB75-MatrixPanel-leddrivers.cpp"
        "Print.cpp"
        "WString.cpp"
        "Adafruit_GFX.cpp"
        "glcdfont.c"
        "stdlib_noniso"
        "gdma_lcd_parallel16.cpp"
    INCLUDE_DIRS "."
    REQUIRES driver esp_lcd   
    PRIV_REQUIRES spi_flash esp_driver_gpio 
)

