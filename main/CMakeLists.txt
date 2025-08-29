idf_component_register(SRCS "main.c" "ssd1306_I2C.h" "my_I2C.h"
                       PRIV_REQUIRES spi_flash
                       INCLUDE_DIRS ""
                       REQUIRES driver) #"driver" is for GPIO functionality
