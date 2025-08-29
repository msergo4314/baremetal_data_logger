#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"


// special library
#include "ssd1306_I2C.h"

void app_main(void)
{
    ssd1306_init();
    bool success;

    printf("Testing on the OLED display...\n");
    printf("Attempting to turn all pixels on...\n");

    byte on_cmd[] = {SSD1306_CONTROL_BYTE(0,0), 0xAF};
    success = ssd1306_write_bytes(on_cmd, 2, true, true);
    printf("Turning on OLED first: %d\n", (int)success);

    // Display OFF
    byte off_cmd[] = {SSD1306_CONTROL_BYTE(0,0), 0xAE};
    success = ssd1306_write_bytes(off_cmd, 2, true, true);
    printf("Turning off: %d\n", (int)success);

    // Display ON
    success = ssd1306_write_bytes(on_cmd, 2, true, true);
    printf("Turning on OLED: %d\n", (int)success);

    printf("Setting entire display on: %d\n", (int)ssd1306_entire_display_on());
    printf("Sweeping brightness...\n");
    while (1) {
        for (short cont = 0; cont <= 0xFF; cont++) {
            if (!ssd1306_set_contrast((byte)cont)) {
                printf("Failed to set contrast value: %x\n", cont);
                break;
            }
            if (cont % 50 == 0) {
                printf("Current contrast: %d\n", cont);
            }
            if (cont % 2 == 0) {
                ssd1306_invert_display();
            } else {
                ssd1306_normal_display();
            }
            // wait 30 ms
            vTaskDelay(30/ portTICK_PERIOD_MS);
        }
        for (short cont = 0xFF; cont >= 0; cont--) {
            if (!ssd1306_set_contrast((byte)cont)) {
                printf("Failed to set contrast value: %x\n", cont);
                break;
            }
            if (cont % 50 == 0) {
                printf("Current contrast: %d\n", cont);
            }
            // wait 30 ms
            vTaskDelay(30/ portTICK_PERIOD_MS);
        }
    }
    // printf("Setting entire display on: %d\n", (int)ssd1306_entire_display_on());
    return;
}
