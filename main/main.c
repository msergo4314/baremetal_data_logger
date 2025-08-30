#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "esp_rtc_time.h"

// special library
#include "ssd1306_I2C.h"

void app_main(void)
{
    printf("init success: %d\n", (int)ssd1306_init()); // could catch the value for checks
    printf("Testing the OLED display...\n");

    ssd1306_write_string_size8x8p("3...", 5*8, 0, 0);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    ssd1306_write_string_size8x8p("2...", 5*8, 0, 0);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    ssd1306_write_string_size8x8p("1...", 5*8, 0, 0);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    ssd1306reset_page(0);
    
    printf("trying to print string...status: %d\n", (int)ssd1306_write_string_size8x8p("Hello world", 20, 0, 0));
    vTaskDelay(500 / portTICK_PERIOD_MS);
    printf("trying to print string...status: %d\n", (int)ssd1306_write_string_size8x8p("Testing with a bigass centered string...", 15, 15, 2));
    vTaskDelay(3000 / portTICK_PERIOD_MS);

    printf("drawing lines\n");
    ssd1306_pixel_coordinate p1 = {0, 63};
    ssd1306_pixel_coordinate p2 = {120, 50};

    printf("trying to print vline...status: %d\n", (int)ssd1306_draw_vline(0, 0, 63, true));
    vTaskDelay(300 / portTICK_PERIOD_MS);
    printf("trying to print hline...status: %d\n", (int)ssd1306_draw_hline(63, 0, 127, true));
    vTaskDelay(300 / portTICK_PERIOD_MS);
    printf("trying to print line...status: %d\n", (int)ssd1306_draw_line(p1, p2, true));
    // printf("Screen clear: %d\n", (int)ssd1306_clear_screen());
    // vTaskDelay(1000 / portTICK_PERIOD_MS);
    
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    byte x = 20, y = 31;
    ssd1306_clear_screen();
    printf("trying to turn on pixel at (%d, %d): %d\n", (int)x, (int)y, (int)ssd1306_set_pixel_xy(x, y, ON, true));
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    ssd1306_clear_screen();
    ssd1306_pixel_coordinate A = {.x=0, .y=0}, B = {.x=1, .y=1};

    ssd1306_draw_rectangle(A, 8, 8, 1, false);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    ssd1306_draw_rectangle(B, 3, 3, 1, true);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    ssd1306_write_string_size8x8p("Scale", 8, 0, 0);

    printf("testing transmission speeds\n");
    printf("CPU frequency: %d MHz\n", CONFIG_ESP32_DEFAULT_CPU_FREQ_MHZ);
    ssd1306_clear_screen();
    p1.x = 0; p1.y = 0;
    ssd1306_draw_rectangle(p1, 128, 64, 2, false);
    vTaskDelay(1000/portTICK_PERIOD_MS);
    ssd1306_write_string_size8x8p("All Done!", 25, 0, 3);
    ssd1306_refresh_display();

    printf("testing transmission speeds\n");
    printf("CPU frequency: %d MHz\n", CONFIG_ESP32_DEFAULT_CPU_FREQ_MHZ);
    
    for (int i = 0; i < 128; i++) {
        ssd1306_draw_vline(i, 0, 63 - i / 2, false); 
    }
    int64_t start = esp_rtc_get_time_us(); // returns time in microseconds 
    ssd1306_refresh_display();
    int64_t end = esp_rtc_get_time_us();
    int64_t elapsed = end - start;
    printf("Elapsed time for Vlines of diminishing length: %lld us (%.3f sec)\n", elapsed, (elapsed) / 1e6);
    printf("Estimated time per vline: %.4lf s\n", (elapsed) / 128e6);
    ssd1306_clear_screen();
    return;
}
