idf_component_register(SRCS "SD_card_SPI.c" "my_SPI.c" "ssd1306_I2C.c" "mpu6050_I2C.c" "main.c" "my_I2C.c"
                       PRIV_REQUIRES spi_flash
                       INCLUDE_DIRS ""
                       REQUIRES driver) #"driver" is for GPIO functionality, esp32 for clock
