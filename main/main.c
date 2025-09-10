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
// #include "my_SPI.h"
#include "ssd1306_I2C.h"
#include "mpu6050_I2C.h"
#include "SD_card_SPI.h"

void app_main(void)
{
    // gpio_set_direction(SPI_CLK, 0);
    // int temp_read = gpio_get_level(SPI_CLK);
    // printf("expecting 0, read: %d\n", temp_read);
    // gpio_set_direction(SPI_CLK, 1);
    // temp_read = gpio_get_level(SPI_CLK);
    // printf("expecting 1, read: %d\n", temp_read);

    if (!SD_card_init(5)) {
        printf("Could not init SD card\n");
        return;
    } else {
        printf("SD card init successful\n");
    }
    printf("OLED init success: %d\n", (int)ssd1306_init()); // could catch the value for checks
    printf("MPU init success: %d\n", (int)mpu6050_init(MPU6050_RANGE_8_G, MPU6050_RANGE_1000_DEG)); // could catch the value for checks
    // printf("Looking for OLED: %d\n", (int)I2C_find_device(SSD1306_ADDRESS));
    // printf("Looking for MPU: %d\n", (int)I2C_find_device(MPU6050_ADDRESS));
    
    int64_t start = esp_rtc_get_time_us(); // returns time in microseconds 
    ssd1306_refresh_display();
    int64_t end = esp_rtc_get_time_us();
    int64_t elapsed = end - start;
    float bits = 9288.0; // estimate
    printf("Elapsed time transmitting %.0f bits with I2C bus: %lld us (%.3f sec)\n", bits, elapsed, (elapsed) / 1e6);
    printf("Estimated I2C speed: %.4lf bits/sec\n", bits / (elapsed / 1e6));

    byte* block_data = malloc(512);
    if (block_data == NULL) {
        printf("block data storage buffer could not malloc\n");
        return;
    }
    start = esp_rtc_get_time_us();
    int block_to_read = 500000;
    if (!SD_read_block(block_to_read, block_data)) {
        printf("Read of block %d failed\n", block_to_read);
        free(block_data);
        return;
    }
    end = esp_rtc_get_time_us();
    elapsed = end - start;
    printf("Read took %.4f ms\n", elapsed / 1000.0);

    printf("Read of block %d:\n", block_to_read);
    for (int i = 0; i < 512; i++) {
        printf("%x ", block_data[i]);
    }
    printf("\n");


    printf("reading 10 blocks\n");
    block_data = realloc(block_data, 512 * 10);
    if (block_data == NULL) {
        printf("block data storage buffer could not malloc\n");
        return;
    }
    start = esp_rtc_get_time_us();
    if (!SD_read_many_blocks(block_to_read, block_data, 10)) {
        printf("10 block read of block %d failed\n", block_to_read);
        free(block_data);
        return;
    }
    end = esp_rtc_get_time_us();
    elapsed = end - start;
    printf("Read took %.4f ms\n", elapsed / 1000.0);

    for (int i = 0; i < 10; i++) {
        printf("Read of block %d:\n", block_to_read + i);
        for (int j = 0; j < 512; j++) {
            printf("%x ", block_data[512 * i + j]);
        }
        printf("\n");
    }

    mpu6050_xyz_data acceleration, gyro;
    float temperature;
    char disp_str[100] = "";
    free(block_data);
    while (1) {
        if (!mpu6050_read_all(&acceleration, &gyro, &temperature)) {
            printf("MPU read_all() error\n");
            return;
        }
        snprintf(disp_str, sizeof(disp_str), "Temp: %02.1f C",temperature);
        if (!ssd1306_write_string_size8x8p(disp_str, 0, 0, 0)) {printf("OLED ERROR\n"); return;}
        snprintf(disp_str, sizeof(disp_str), "X: %+02.1f %+02.1f ", acceleration.x, gyro.x);
        if (!ssd1306_write_string_size8x8p(disp_str, 0, 0, 2)) {printf("OLED ERROR\n"); return;}
        snprintf(disp_str, sizeof(disp_str), "Y: %+02.1f %+02.1f ", acceleration.y, gyro.y);
        if (!ssd1306_write_string_size8x8p(disp_str, 0, 0, 3)) {printf("OLED ERROR\n"); return;}
        snprintf(disp_str, sizeof(disp_str), "Z: %+02.1f %+02.1f ", acceleration.z, gyro.z);
        if (!ssd1306_write_string_size8x8p(disp_str, 0, 0, 4)) {printf("OLED ERROR\n"); return;}
        if (!ssd1306_refresh_display()) {printf("OLED ERROR\n"); return;}
        // 20 refreshs/sec -- refresh_display() takes about 14 ms
        vTaskDelay(pdMS_TO_TICKS(50 - (elapsed) / 1000));
    }
    return;
}
