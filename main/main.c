#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "esp_rtc_time.h"

// custom libraries
// #include "my_I2C.h"
#include "ssd1306_I2C.h"
#include "mpu6050_I2C.h"

void app_main(void)
{

    I2C_init();
    printf("Looking for OLED: %d\n", (int)find_device(SSD1306_ADDRESS));
    printf("Looking for MPU: %d\n", (int)find_device(MPU6050_ADDRESS));
    printf("OLED init success: %d\n", (int)ssd1306_init()); // could catch the value for checks
    printf("MPU init success: %d\n", (int)mpu6050_init(MPU6050_RANGE_2_G, MPU6050_RANGE_250_DEG)); // could catch the value for checks

    mpu6050_xyz_data acceleration, gyro;
    float temperature;
    char disp_str[100] = "";
    while (1) {
        if (!mpu6050_read_all(&acceleration, &gyro, &temperature)) return;
        snprintf(disp_str, sizeof(disp_str), "Temp: %02.1f C",temperature);
        ssd1306_write_string_size8x8p(disp_str, 0, 0, 0);
        snprintf(disp_str, sizeof(disp_str), "X: %+02.1f %+02.1f ", acceleration.x, gyro.x);
        ssd1306_write_string_size8x8p(disp_str, 0, 0, 2);
        snprintf(disp_str, sizeof(disp_str), "Y: %+02.1f %+02.1f ", acceleration.y, gyro.y);
        ssd1306_write_string_size8x8p(disp_str, 0, 0, 3);
        snprintf(disp_str, sizeof(disp_str), "Z: %+02.1f %+02.1f ", acceleration.z, gyro.z);
        ssd1306_write_string_size8x8p(disp_str, 0, 0, 4);
        ssd1306_refresh_display();

        vTaskDelay(pdMS_TO_TICKS(50));
    }
    return;
}
