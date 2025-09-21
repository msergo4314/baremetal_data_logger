#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "esp_rtc_time.h"

#include <math.h>
// custom libraries
#include "ssd1306_I2C.h"
#include "mpu6050_I2C.h"
#include "SD_card_SPI.h"

void insert_into_window(float value, float* graph_window, size_t number_of_window_elements);
float xyz_2_norm(float x, float y, float z);

byte accel_to_scaled_pixel(float accleration_reading, float max_accel_magnitude);
byte gyro_to_scaled_pixel(float gyro_reading, float max_gyro_magnitude);

#define graph_x_axis_length_px 51
#define graph_y_axis_height_px 38

void app_main(void) {
    char SD_string[512] = "";

    const MPU6050_ACCELEROMETER_RANGE accel_range = MPU6050_RANGE_2_G;
    const MPU6050_GYROSCOPE_RANGE gyro_range = MPU6050_RANGE_250_DEG;

    float max_accel = 2 * powf(2.0, accel_range);
    float max_deg_per_sec = 250 * powf(2.0, gyro_range);

    float max_accel_mag = sqrt(3 * (max_accel * max_accel));
    float max_gyro_mag = sqrt(3 * (max_deg_per_sec * max_deg_per_sec));

    printf("max acceleration is %.0f g with max magnitude %.2f\n", max_accel, max_accel_mag);
    printf("max gyro is %.0f deg/sec with max magnitude %.2f\n", max_deg_per_sec, max_gyro_mag);

    if (!SD_card_init(5)) {
        printf("Could not init SD card\n");
        return;
    } else {
        printf("SD card init successful\n");
    }
    printf("OLED init success: %d\n", (int)ssd1306_init());
    printf("MPU init success: %d\n", (int)mpu6050_init(accel_range, gyro_range));
    
    ssd1306_pixel_coordinate temp = {.x=0, .y=0};
    // fill GDDRAM with 1s and update display to test transmission speed
    if (!ssd1306_draw_rectangle(temp, 128, 64, 1, true, false)) return;
    int64_t start = esp_rtc_get_time_us(); // returns time in microseconds
    bool success = ssd1306_refresh_display();
    int64_t end = esp_rtc_get_time_us();
    ssd1306_clear_screen();
    if (!success) {printf("SSD1306 I2C test transmission failed\n"); return;}
    int64_t elapsed = end - start;
    printf("Testing I2C transmission: %s\n", success ? "success" : "failure");
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
        printf("%02x ", block_data[i]);
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
    if (!SD_write_many_blocks(500001, block_data, 5)) {free(block_data); printf("Write failure\n"); return;}
    end = esp_rtc_get_time_us();
    elapsed = end - start;
    printf("5 block write took %.4f ms\n", elapsed / 1000.0);
    // printf("Wrote 5 blocks successfully. Reading 6 blocks:\n");
    byte *six_blocks = malloc(512 * 6);
    if (!SD_read_many_blocks(500000, six_blocks, 6)) {free(block_data); free(six_blocks); printf("Read failure\n"); return;}

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
    if (!ssd1306_write_string_size8x8p("Starting OLED display!", 10, 0, 0, true)) {printf("OLED ERROR\n"); return;}
    vTaskDelay(pdMS_TO_TICKS(750));
    start = esp_rtc_get_time_us();
    if (!ssd1306_set_pixel_xy(50, 40, PIXEL_SET, true)) return;
    elapsed = esp_rtc_get_time_us() - start;
    printf("Time to set one pixel and display on OLED: %.2f ms\n", (float)elapsed / 1000.0);
    if (!ssd1306_clear_screen()) return;

    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(50);

    float accel_window[graph_x_axis_length_px - 1] = {-1.0};
    float gyro_window[graph_x_axis_length_px - 1] = {-1.0};


    // draw graph axes for accel/gyro magnitudes
    ssd1306_draw_hline(63, 0, graph_x_axis_length_px, false);
    ssd1306_draw_hline(63, graph_x_axis_length_px + 20, 2 * graph_x_axis_length_px + 20, false);

    ssd1306_draw_vline(0, 63, 63 - graph_y_axis_height_px, false);
    ssd1306_draw_vline(graph_x_axis_length_px + 20, 63, 63 - graph_y_axis_height_px, false);
    ssd1306_refresh_display();

    while (1) {
        start = esp_rtc_get_time_us();
        if (!mpu6050_read_all(&acceleration, &gyro, &temperature)) {
            printf("MPU read_all() error\n");
            return;
        }
        snprintf(disp_str, sizeof(disp_str), "Temp: %-3.1f C    ",temperature);
        ssd1306_write_string_size8x8p(disp_str, 0, 0, 0, false);


        snprintf(disp_str, sizeof(disp_str), "accel    gyro");
        ssd1306_write_string_size8x8p(disp_str, 7, 0, 2, false);

        // snprintf(disp_str, sizeof(disp_str), "X: %-+2.1f  %-+3.0f  ", acceleration.x, gyro.x);
        // if (!ssd1306_write_string_size8x8p(disp_str, 0, 0, 2, false)) {printf("OLED ERROR\n"); return;}
        
        // snprintf(disp_str, sizeof(disp_str), "Y: %-+2.1f  %-+4.0f  ", acceleration.y, gyro.y);
        // if (!ssd1306_write_string_size8x8p(disp_str, 0, 0, 3, false)) {printf("OLED ERROR\n"); return;}
        
        // snprintf(disp_str, sizeof(disp_str), "Z: %-+2.1f  %-+4.0f  ", acceleration.z, gyro.z);
        // if (!ssd1306_write_string_size8x8p(disp_str, 0, 0, 4, false)) {printf("OLED ERROR\n"); return;}

        float accel_mag = xyz_2_norm(acceleration.x, acceleration.y, acceleration.z);
        float gyro_mag = xyz_2_norm(gyro.x, gyro.y, gyro.z);

        // snprintf(disp_str, sizeof(disp_str), "|accl| = %-+2.02f ", accel_mag);
        // if (!ssd1306_write_string_size8x8p(disp_str, 0, 0, 6, false)) {printf("OLED ERROR\n"); return;}

        // snprintf(disp_str, sizeof(disp_str), "|gyro| = %-+3.0f ", gyro_mag);
        // if (!ssd1306_write_string_size8x8p(disp_str, 0, 0, 7, false)) {printf("OLED ERROR\n"); return;}
        
        // update the moving windows
        insert_into_window(accel_mag, accel_window, graph_x_axis_length_px - 1);
        insert_into_window(gyro_mag, gyro_window, graph_x_axis_length_px - 1);

        // Draw acceleration graph
        ssd1306_pixel_coordinate coord_1, coord_2;
        for (size_t i = 0; i < graph_x_axis_length_px - 2; i++) {
            if (accel_window[i] != -1.0 && accel_window[i + 1] != -1.0) {
                byte y0 = accel_to_scaled_pixel(accel_window[i], max_accel_mag);
                byte y1 = accel_to_scaled_pixel(accel_window[i + 1], max_accel_mag);
                coord_1.x = i + 1;
                coord_1.y = y0;
                coord_2.x = i + 2;
                coord_2.y = y1;
                ssd1306_draw_line(coord_1, coord_2, false);
            }
        }
        coord_2.x++;
        ssd1306_set_pixel(coord_2, PIXEL_SET, false);

        // Draw gyro graph
        for (size_t i = 0; i < graph_x_axis_length_px - 2; i++) {
            if (gyro_window[i] != -1.0 && gyro_window[i + 1] != -1.0) {
                byte y0 = gyro_to_scaled_pixel(gyro_window[i], max_gyro_mag);
                byte y1 = gyro_to_scaled_pixel(gyro_window[i + 1], max_gyro_mag);
                coord_1.x = i + 21 + graph_x_axis_length_px;
                coord_1.y = y0;
                coord_2.x = i + 22 + graph_x_axis_length_px;
                coord_2.y = y1;
                ssd1306_draw_line(coord_1, coord_2, false);
            }
        }
        coord_2.x++;
        ssd1306_set_pixel(coord_2, PIXEL_SET, false);

        if (!ssd1306_refresh_display()) {printf("OLED ERROR -- I2C FAILED\n"); return;}
        //20 Hz --> 50 ms delay between loops
        // printf("Time to do OLED operations in ms: %.2f\n", (float)((esp_rtc_get_time_us() - start) / 1000));
        xTaskDelayUntil(&last_wake_time, period);

        // clear the internal GDDRAM but don't update the display yet to get ready for the next iteration
        coord_1.x = 1;
        coord_1.y = 63 - graph_y_axis_height_px;
        // uint64_t start_2 = esp_rtc_get_time_us();
        ssd1306_clear_rectangle(coord_1, graph_x_axis_length_px, graph_y_axis_height_px, false);
        coord_2.x = graph_x_axis_length_px + 21;
        coord_2.y = 63 - graph_y_axis_height_px;
        ssd1306_clear_rectangle(coord_2, graph_x_axis_length_px, graph_y_axis_height_px, false);
        // printf("time for drawing clear rectangles: %.2f ms\n", (float)((esp_rtc_get_time_us() - start_2) / 1000.0));
    }
    return;
}

float xyz_2_norm(float x, float y, float z) {
    return sqrtf(x * x + y * y + z * z);
}

void insert_into_window(float value, float* graph_window, size_t number_of_window_elements) {
    if (number_of_window_elements <= 1) {
        printf("need > 1 window elements\n");
        return;
    }

    // shift everything left
    for (size_t i = 0; i < number_of_window_elements - 1; i++) {
        graph_window[i] = graph_window[i + 1];
    }

    // put newest value at the far right
    graph_window[number_of_window_elements - 1] = value;
}

// normalize acceleration readings
byte accel_to_scaled_pixel(float accleration_reading, float max_accel_magnitude) {
    float ratio = (accleration_reading / max_accel_magnitude);
    ratio = (ratio > 1.0) ? 1.0 : ratio; // should never be > 1.0 but just in case
    // minimum value will be pixel 62 (just above the x axis)
    byte scaled_accel = 62;

    scaled_accel -= (byte)((ratio) * (graph_y_axis_height_px - 1));
    return scaled_accel;
}

// normalize gyroscope readings
byte gyro_to_scaled_pixel(float gyro_reading, float max_gyro_magnitude) {
    float ratio = (gyro_reading / max_gyro_magnitude);
    ratio = (ratio > 1.0) ? 1.0 : ratio; // should never be > 1.0 but just in case
    // minimum value will be pixel 62 (just above the x axis)
    byte scaled_gyro = 62;

    scaled_gyro -= (byte)((ratio) * (graph_y_axis_height_px - 1));
    return scaled_gyro;
}