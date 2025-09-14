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
#include "ssd1306_I2C.h"
#include "mpu6050_I2C.h"
#include "SD_card_SPI.h"


void app_main(void) {
    char SD_string[512] = "";
    if (!SD_card_init(5)) {
        printf("Could not init SD card\n");
        return;
    } else {
        printf("SD card init successful\n");
    }
    printf("OLED init success: %d\n", (int)ssd1306_init());
    printf("MPU init success: %d\n", (int)mpu6050_init(MPU6050_RANGE_8_G, MPU6050_RANGE_1000_DEG));
    
    int64_t start = esp_rtc_get_time_us(); // returns time in microseconds 
    ssd1306_refresh_display();
    int64_t end = esp_rtc_get_time_us();
    int64_t elapsed = end - start;
    float bits = 9288.0; // estimate
    printf("Elapsed time transmitting %.0f bits with I2C bus: %lld us (%.3f sec)\n", bits, elapsed, (elapsed) / 1e6);
    printf("Estimated I2C speed: %.4lf bits/sec\n", bits / (elapsed / 1e6));

    byte* block_data = malloc(512 * 10);
    if (block_data == NULL) {
        printf("block data storage buffer could not malloc\n");
        return;
    }
    start = esp_rtc_get_time_us();
    int block_to_read = 0;
    if (!SD_read_block(block_to_read, block_data)) {
        printf("Read of block %d failed\n", block_to_read);
        free(block_data);
        return;
    }
    end = esp_rtc_get_time_us();
    elapsed = end - start;
    printf("Single block read took %.4f ms\n", elapsed / 1000.0);

    printf("Read of block %d:\n", block_to_read);
    for (int i = 0; i < 512; i++) {
        printf("%x ", block_data[i]);
    }
    printf("\nIs block 0 empty? -- %s\n", SD_is_block_empty(0) ? "YES" : "NO");
    if (!SD_clear_block(500000)) return;
    printf("\nIs block 500 000 empty? -- %s\n", SD_is_block_empty(500000) ? "YES" : "NO");

    block_to_read = 500000;
    printf("reading 10 blocks\n");
    start = esp_rtc_get_time_us();
    if (!SD_read_many_blocks(block_to_read, block_data, 10)) {
        printf("10 block read of block %d failed\n", block_to_read);
        free(block_data);
        return;
    }
    end = esp_rtc_get_time_us();
    elapsed = end - start;
    printf("10 block read took %.4f ms\n", elapsed / 1000.0);

    printf("\n\nCopying block 0 to block 500 000\n");
    SD_read_block(0, block_data);
    start = esp_rtc_get_time_us();
    if (!SD_write_block(500000, (const byte*)block_data)) {
        free(block_data);
        printf("SD write failed\n");
        return;
    }
    end = esp_rtc_get_time_us();
    elapsed = end - start;
    printf("Single block write took %.4f ms\n", elapsed / 1000.0);
    printf("contents of block 0 should be in block 500 000 now:\n\n");
    if (!SD_read_block(500000, block_data)) {free(block_data); return;}
    printf("Read of block %d:\n", 500000);
    for (int i = 0; i < 512; i++) {
        printf("%02x ", block_data[i]);
    }
    printf("\nSetting block 500 000 to \"Hello world!\"...\n");
    strcpy(SD_string, "Hello World!");
    if (!SD_write_block(500000, (const byte*)SD_string)) {free(block_data); return;}
    char testing_str[512];
    if (!SD_read_block(500000, (byte*)testing_str)) {free(block_data); return;}
    printf("Block 500 000 says: %s\n", testing_str);
    
    byte first[512] = {0};
    first[0] = 0xAA;

    for (int i = 0; i < 5; i++) {
        memcpy(&(block_data[512* i]), first, sizeof(first));
    }
    start = esp_rtc_get_time_us();
    if (!SD_write_many_blocks(500001, block_data, 5)) {printf("Write failure\n"); return;}
    end = esp_rtc_get_time_us();
    elapsed = end - start;
    printf("5 block write took %.4f ms\n", elapsed / 1000.0);
    // printf("Wrote 5 blocks successfully. Reading 6 blocks:\n");
    byte *six_blocks = malloc(512 * 6);
    if (!SD_read_many_blocks(500000, six_blocks, 6)) {printf("Read failure\n"); return;}

    // for (int i=0; i < 6; i++) {
    //     printf("Block %d data:\n", block_to_read + i);
    //     for(int j = 0; j < 512; j++) {
    //         printf("%02x ", six_blocks[512 * i + j]);
    //         if (j == 511) {
    //             printf("\n");
    //         }
    //     }
    // }
    if(!SD_clear_many_blocks(500000, 6)) return;
    free(six_blocks);

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
        if (!ssd1306_refresh_display()) {printf("OLED ERROR -- I2C FAILED\n"); return;}
        // 20 refreshs/sec -- refresh_display() takes about 14 ms
        vTaskDelay(pdMS_TO_TICKS(50 - 14));
    }
    return;
}

void time_fun(void(*function)(void* args), void* args) {
    function(args);
    return;
}
