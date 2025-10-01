#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_system.h"
#include "esp_rtc_time.h"
#include "driver/gptimer.h" // hardware timer

#include <math.h>
// custom libraries
#include "ssd1306_I2C.h"
#include "mpu6050_I2C.h"

#define USE_HW_SPI 1

#if USE_HW_SPI
    #include "SD_card.h"
#else 
    #include "SD_card_SPI.h"
#endif


void insert_into_window(float value, float* graph_window, size_t number_of_window_elements);
float xyz_2_norm(float x, float y, float z);
void reset_512_byte_block_buffer(byte* buffer, size_t number_of_blocks);

byte accel_to_scaled_pixel(float accleration_reading, float max_accel_magnitude);
byte gyro_to_scaled_pixel(float gyro_reading, float max_gyro_magnitude);

#define GRAPH_X_AXIS_LENGTH_PX 51
#define GRAPH_y_AXIS_HEIGHT_PX 35

#define SD_NUMBER_OF_BLOCKS 10
byte SD_memory_block[512 * SD_NUMBER_OF_BLOCKS] = {0};

const MPU6050_ACCELEROMETER_RANGE accel_range = MPU6050_RANGE_2_G;
const MPU6050_GYROSCOPE_RANGE gyro_range = MPU6050_RANGE_250_DEG;

typedef struct {
    float timestamp; //timestamp since program start
    mpu6050_xyz_data accel;
    mpu6050_xyz_data gyro;
    float temperature; // MPU die temp
} queue_data;

QueueHandle_t OLED_queue, SD_queue;

TaskHandle_t mpu_task_handle = NULL;

void setup_task(void* pvParameters);
void MPU_task(void* pvParameters);
void OLED_task(void* pvParameters);
void SD_task(void* pvParameters);
static bool alarm_cb(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx);

void setup_task(void* pvParameters) {

    /*
    used more to test drivers than actual setup
    */

    TaskHandle_t main_handle = (TaskHandle_t)pvParameters;

    float max_accel, max_deg_per_sec, max_accel_mag, max_gyro_mag;

    max_accel = 2 * powf(2.0, accel_range);
    max_deg_per_sec = 250 * powf(2.0, gyro_range);

    max_accel_mag = sqrt(3 * (max_accel * max_accel));
    max_gyro_mag = sqrt(3 * (max_deg_per_sec * max_deg_per_sec));

    printf("max acceleration is %.0f g with max magnitude %.2f\n", max_accel, max_accel_mag);
    printf("max gyro is %.0f deg/sec with max magnitude %.2f\n", max_deg_per_sec, max_gyro_mag);

    if (!SD_card_init(GPIO_NUM_5)) {
        printf("Could not init SD card\n");
        goto exit_failure;
    }
    #if !USE_HW_SPI
        printf("SPI clock speed: %d Hz\n", SPI_get_clock_speed_Hz());
    #endif
    printf("SD card init successful\n");
    bool init = ssd1306_init();
    printf("OLED init success: %d\n", (int)init);
    if (init == false) {
        goto exit_failure;
    }
    init = mpu6050_init(accel_range, gyro_range);
    printf("MPU init success: %d\n", (int)init);
    if (init == false) {
        goto exit_failure;
    }
    
    ssd1306_pixel_coordinate temp = {.x=0, .y=0};
    // fill GDDRAM with 1s and update display to test transmission speed
    if (!ssd1306_draw_rectangle(temp, 128, 64, 1, true, false)) goto exit_failure;
    int64_t start = esp_rtc_get_time_us(); // returns time in microseconds
    bool success = ssd1306_refresh_display();
    int64_t end = esp_rtc_get_time_us();
    ssd1306_clear_screen();
    if (!success) {printf("SSD1306 I2C test transmission failed\n"); goto exit_failure;}
    int64_t elapsed = end - start;
    printf("Testing I2C transmission: %s\n", success ? "success" : "failure");
    float bits = 9288.0; // estimate
    printf("Elapsed time transmitting %.0f bits with I2C bus: %lld us (%.3f sec)\n", bits, elapsed, (elapsed) / 1e6);
    printf("Estimated I2C speed: %.4lf bits/sec\n", bits / (elapsed / 1e6));

    start = esp_rtc_get_time_us();
    int block_to_read = 0;
    if (!SD_read_block(block_to_read, SD_memory_block)) {
        printf("Read of block %d failed\n", block_to_read);
        goto exit_failure;
    }
    end = esp_rtc_get_time_us();
    elapsed = end - start;
    printf("Single block read took %.4f ms\n", elapsed / 1000.0);

    printf("Read of block %d:\n", block_to_read);
    for (int i = 0; i < 512; i++) {
        printf("%02x ", SD_memory_block[i]);
    }
    printf("\nIs block 0 empty? -- %s\n", SD_is_block_empty(0) ? "YES" : "NO");
    if (!SD_clear_block(500000)) goto exit_failure;
    printf("\nIs block 500 000 empty? -- %s\n", SD_is_block_empty(500000) ? "YES" : "NO");

    block_to_read = 500000;
    printf("reading 10 blocks\n");
    start = esp_rtc_get_time_us();
    if (!SD_read_many_blocks(block_to_read, SD_memory_block, SD_NUMBER_OF_BLOCKS)) {
        printf("%d block read of block %d failed\n",(int)SD_NUMBER_OF_BLOCKS, block_to_read);
        goto exit_failure;
    }
    end = esp_rtc_get_time_us();
    elapsed = end - start;
    printf("10 block read took %.4f ms\n", elapsed / 1000.0);

    printf("\n\nCopying block 0 to block 500 000\n");
    SD_read_block(0, SD_memory_block);
    start = esp_rtc_get_time_us();
    if (!SD_write_block(500000, (const byte*)SD_memory_block)) {
        printf("SD write failed\n");
        goto exit_failure;
    }
    end = esp_rtc_get_time_us();
    elapsed = end - start;
    printf("Single block write took %.4f ms\n", elapsed / 1000.0);
    printf("contents of block 0 should be in block 500 000 now:\n\n");
    if (!SD_read_block(500000, SD_memory_block)) goto exit_failure;
    printf("Read of block %d:\n", 500000);
    for (int i = 0; i < 512; i++) {
        printf("%02x ", SD_memory_block[i]);
    }
    printf("\nSetting block 500 000 to \"Hello world!\"...\n");
    strcpy((char*)SD_memory_block, "Hello World!");
    if (!SD_write_block(500000, (const byte*)SD_memory_block)) goto exit_failure;
    char testing_str[512] = "INITIAL_STATE";
    if (!SD_read_block(500000, (byte*)testing_str)) goto exit_failure;
    printf("Block 500 000 says: %s\n", testing_str);

    memset(SD_memory_block, 0x0, sizeof(SD_memory_block)); // reset the string by clearing the old bytes
    SD_memory_block[0] = 0xAA;
    start = esp_rtc_get_time_us();
    if (!SD_write_many_blocks(500001, SD_memory_block, 10)) {printf("Write failure\n"); goto exit_failure;}
    end = esp_rtc_get_time_us();
    elapsed = end - start;
    printf("10 block write took %.4f ms\n", elapsed / 1000.0);
    // printf("Wrote 5 blocks successfully. Reading 6 blocks:\n");
    if (!SD_read_many_blocks(500000, SD_memory_block, 6)) {printf("Read failure\n"); goto exit_failure;}

    // for (int i=0; i < 6; i++) {
    //     printf("Block %d data:\n", block_to_read + i);
    //     for(int j = 0; j < 512; j++) {
    //         printf("%02x ", block_data[512 * i + j]);
    //     }
    //     printf("\n");
    // }
    if(!SD_clear_many_blocks(500000, 11)) goto exit_failure;
    for (int i = 0; i < 11; i++) {
        int block_to_check = 500000 + i;
        if (!SD_is_block_empty(block_to_check)) {
            printf("Block %d was not cleared properly!\n", block_to_check);
            goto exit_failure;
        }
    }
    printf("Cleared all used blocks. Giving notification...\n");
    // notify app_main that we are done with setup testing
    xTaskNotify(main_handle, 1, eSetValueWithOverwrite);
    vTaskDelete(NULL);

    exit_failure:
    xTaskNotify(main_handle, 0, eSetValueWithOverwrite);
    vTaskDelete(NULL);
}

static bool alarm_cb(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx) {
    // General process for handling event callbacks:
    // 1. Retrieve user context data from user_ctx (passed in from gptimer_register_event_callbacks)
    // 2. Get alarm event data from edata, such as edata->count_value
    // 3. Perform user-defined operations
    // 4. Return whether a high-priority task was awakened during the above operations to notify the scheduler to switch tasks
    
    // Notify the MPU task
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    TaskHandle_t task_to_notify = *(TaskHandle_t*)user_ctx;
    if (task_to_notify) {
        vTaskNotifyGiveFromISR(task_to_notify, &xHigherPriorityTaskWoken);
    }
    return ((bool)(xHigherPriorityTaskWoken == pdTRUE)); // return true if a higher-priority task should run
}

void MPU_task(void* pvParameters) {
    //  poll the MPU at 100 Hz, then send data to OLED
    gptimer_handle_t gptimer = NULL;
    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT, // Select the default clock source
        .direction = GPTIMER_COUNT_UP,      // Counting direction is up
        .resolution_hz = 1 * 1000 * 1000,   // Resolution is 1 MHz, i.e., 1 tick equals 1 microsecond
    };

    // Create a timer instance
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &gptimer));

    gptimer_alarm_config_t alarm_config = {
        .reload_count = 0,      // When the alarm event occurs, the timer will automatically reload to 0
        .alarm_count = 10000, // Set the actual alarm period, since the resolution is 1us, 10000 represents 10 ms
        .flags.auto_reload_on_alarm = true, // Enable auto-reload function
    };

    // Set the timer's alarm action
    ESP_ERROR_CHECK(gptimer_set_alarm_action(gptimer, &alarm_config));

    gptimer_event_callbacks_t cbs = {
        .on_alarm = alarm_cb, // Call the user callback function when the alarm event occurs
    };
    // Register timer event callback functions, allowing user context to be carried
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(gptimer, &cbs, (void*)&mpu_task_handle)); // set the context to the task handle we want to notify
    // Enable the timer
    ESP_ERROR_CHECK(gptimer_enable(gptimer));
    // Start the timer
    ESP_ERROR_CHECK(gptimer_start(gptimer));

    // grab a reading at a fixed rate
    queue_data data;
    for (;;) {
        // wait for the hardware timer alarm to trigger (every 10 ms)
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        // read accel, gyro, and die temp
        // printf("calling mpu read all\n");
        mpu6050_read_all(&(data.accel), &(data.gyro), &(data.temperature));
        // printf("finished mpu read all\n");
        data.timestamp = esp_rtc_get_time_us() / 1e6f; // get current timestamp in seconds
        xQueueSendToBack(SD_queue, &data, pdMS_TO_TICKS(10)); // wait up to 10 ms if queue is filled
        xQueueOverwrite(OLED_queue, &data); // put most recent data in OLED queue
    }
}

void OLED_task(void* pvParameters) {

    float accel_window[GRAPH_X_AXIS_LENGTH_PX - 1] = {-1.0};
    float gyro_window[GRAPH_X_AXIS_LENGTH_PX - 1] = {-1.0};

    float max_accel, max_deg_per_sec, max_accel_mag, max_gyro_mag;

    max_accel = 2 * powf(2.0, accel_range);
    max_deg_per_sec = 250 * powf(2.0, gyro_range);

    // 2 norm of 3D vectors
    max_accel_mag = sqrt(3 * (max_accel * max_accel));
    max_gyro_mag = sqrt(3 * (max_deg_per_sec * max_deg_per_sec));

    // draw graph axes for accel/gyro magnitudes
    ssd1306_draw_hline(63, 0, GRAPH_X_AXIS_LENGTH_PX, false);
    ssd1306_draw_hline(63, GRAPH_X_AXIS_LENGTH_PX + 20, 2 * GRAPH_X_AXIS_LENGTH_PX + 20, false);

    ssd1306_draw_vline(0, 63, 63 - GRAPH_y_AXIS_HEIGHT_PX, false);
    ssd1306_draw_vline(GRAPH_X_AXIS_LENGTH_PX + 20, 63, 63 - GRAPH_y_AXIS_HEIGHT_PX, false);
    if (!ssd1306_refresh_display()) {
        printf("failed to draw graph axes\n");
        vTaskDelete(NULL);
    }

    char disp_str[100] = "";
    mpu6050_xyz_data acceleration, gyro;
    float temperature;
    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(50); // 20 Hz refresh rate
    
    queue_data last_data;
    for (;;) {
        // grab the most recent readings
        xQueueReceive(OLED_queue, &last_data, pdMS_TO_TICKS(10));
        acceleration = last_data.accel;
        gyro = last_data.gyro;
        temperature = last_data.temperature;

        snprintf(disp_str, sizeof(disp_str), "Temp: %-3.1f C",temperature);
        ssd1306_write_string_size8x8p(disp_str, 0, 0, 0, false);
        snprintf(disp_str, sizeof(disp_str), "accel    gyro");
        ssd1306_write_string_size8x8p(disp_str, 7, 0, 2, false);

        float accel_mag = xyz_2_norm(acceleration.x, acceleration.y, acceleration.z);
        float gyro_mag = xyz_2_norm(gyro.x, gyro.y, gyro.z);
        
        // update the moving windows
        insert_into_window(accel_mag, accel_window, GRAPH_X_AXIS_LENGTH_PX - 1);
        insert_into_window(gyro_mag, gyro_window, GRAPH_X_AXIS_LENGTH_PX - 1);

        // Draw acceleration graph
        ssd1306_pixel_coordinate coord_1, coord_2;
        for (size_t i = 0; i < GRAPH_X_AXIS_LENGTH_PX - 2; i++) {
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
        for (size_t i = 0; i < GRAPH_X_AXIS_LENGTH_PX - 2; i++) {
            if (gyro_window[i] != -1.0 && gyro_window[i + 1] != -1.0) {
                byte y0 = gyro_to_scaled_pixel(gyro_window[i], max_gyro_mag);
                byte y1 = gyro_to_scaled_pixel(gyro_window[i + 1], max_gyro_mag);
                coord_1.x = i + 21 + GRAPH_X_AXIS_LENGTH_PX;
                coord_1.y = y0;
                coord_2.x = i + 22 + GRAPH_X_AXIS_LENGTH_PX;
                coord_2.y = y1;
                ssd1306_draw_line(coord_1, coord_2, false);
            }
        }
        coord_2.x++;
        ssd1306_set_pixel(coord_2, PIXEL_SET, false);

        if (!ssd1306_refresh_display()) {
            printf("OLED ERROR -- I2C FAILED\n");
            vTaskDelete(NULL);
        }
        // clear the internal GDDRAM but don't update the display yet to get ready for the next iteration
        coord_1.x = 1;
        coord_1.y = 63 - GRAPH_y_AXIS_HEIGHT_PX;
        ssd1306_clear_rectangle(coord_1, GRAPH_X_AXIS_LENGTH_PX, GRAPH_y_AXIS_HEIGHT_PX, false);
        coord_2.x = GRAPH_X_AXIS_LENGTH_PX + 21;
        coord_2.y = 63 - GRAPH_y_AXIS_HEIGHT_PX;
        ssd1306_clear_rectangle(coord_2, GRAPH_X_AXIS_LENGTH_PX, GRAPH_y_AXIS_HEIGHT_PX, false);
        xTaskDelayUntil(&last_wake_time, period);
    }
}

void SD_task(void* pvParameters) {
    // make sure we don't have an address with important data
    const uint32_t starting_address = 500000;
    uint32_t current_address = starting_address;

    // we will have 5000 blocks of available space to use (133 min)
    const uint32_t max_address = starting_address + 5000;

    queue_data data;
    const byte entries_per_block = 512 / sizeof(queue_data);

    byte current_block = 0;
    byte count = 0;
    for (;;) {
        if (xQueueReceive(SD_queue, &data, pdMS_TO_TICKS(100)) != pdTRUE) {
            continue; // keep waiting for data
        }
        int offset = current_block * 512 + sizeof(data) * count++;
        memcpy(&(SD_memory_block[offset]), &data, sizeof(queue_data));

        if (count >= entries_per_block) {
            count = 0;
            // we have filled one of the 512 byte blocks
            current_block++;
            if (current_block >= SD_NUMBER_OF_BLOCKS) {
                // the whole buffer is full, so write it to the card
                if (!SD_write_many_blocks(current_address, SD_memory_block, SD_NUMBER_OF_BLOCKS)) {
                    printf("failed to write to SD card\n");
                    vTaskDelete(NULL);
                }
                if ((current_address += SD_NUMBER_OF_BLOCKS) > max_address) {
                    current_address = starting_address;
                }
                current_block = 0;
            }
        }
    }
}

void app_main(void) {
    
    SD_queue = xQueueCreate(100, sizeof(queue_data));
    if (SD_queue == NULL) {
        printf("Failed to create MPU queue!\n");
        return;
    }

    // OLED only needs the most recent reading
    OLED_queue = xQueueCreate(1, sizeof(queue_data));
    if (SD_queue == NULL) {
        printf("Failed to create MPU queue!\n");
        return;
    }

    BaseType_t xReturned;
    TaskHandle_t xHandle = NULL;
    xReturned = xTaskCreatePinnedToCore(
                setup_task,                            /* Function that implements the task. */
                "SETUP TASK",                          /* Text name for the task. */
                1024 * 10,                             /* Stack size in bytes. (1 word = 4 bytes)*/
                (void*)xTaskGetCurrentTaskHandle(),    /* Parameter passed into the task. */
                2,                                     /* Priority at which the task is created. */
                &xHandle,                              /* handle for task being created */
                tskNO_AFFINITY                         /* Core to run task */
                );      

    if(xReturned != pdPASS) {
        printf("could not create the setup task\n");
        return;   
    }
    // wait for setup task to finish
    uint32_t setupStatus;
    // read the notification value and wait forever
    if (xTaskNotifyWait(0, 0, &setupStatus, portMAX_DELAY) == pdTRUE) {
        if (setupStatus == 0) {
            // setup failed → return from main
            printf("setup task failed\n");
            return;
        }
    }
    // setup succeeded → start forever tasks
    xReturned = xTaskCreatePinnedToCore(
                MPU_task,                             /* Function that implements the task. */
                "MPU TASK",                           /* Text name for the task. */
                1024 * 2,                             /* Stack size in bytes. (1 word = 4 bytes)*/
                NULL,                                 /* Parameter passed into the task. */
                2,                                    /* Priority at which the task is created. */
                &mpu_task_handle,                     /* handle for task being created */
                0                                     /* Core to run task. */
                );

    if(xReturned != pdPASS) {
        printf("could not create the MPU task\n");
        return;
    }

    xReturned = xTaskCreatePinnedToCore(
                OLED_task,                            /* Function that implements the task. */
                "OLED TASK",                          /* Text name for the task. */
                1024 * 5,                             /* Stack size in bytes. (1 word = 4 bytes)*/
                NULL,                                 /* Parameter passed into the task. */
                2,                                    /* Priority at which the task is created. */
                &xHandle,                             /* handle for task being created */
                0                                     /* Core to run task */
                );

    if(xReturned != pdPASS) {
        printf("could not create the OLED task\n");
        return;
    }

    xReturned = xTaskCreatePinnedToCore(
                SD_task,                              /* Function that implements the task. */
                "SD TASK",                            /* Text name for the task. */
                1024 * 10,                            /* Stack size in bytes. (1 word = 4 bytes)*/
                NULL,                                 /* Parameter passed into the task. */
                2,                                    /* Priority at which the task is created. */
                &xHandle,                             /* handle for task being created */
                0                                     /* Core to run task */
                );

    if(xReturned != pdPASS) {
        printf("could not create the SD task\n");
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

    scaled_accel -= (byte)((ratio) * (GRAPH_y_AXIS_HEIGHT_PX - 1));
    return scaled_accel;
}

// normalize gyroscope readings
byte gyro_to_scaled_pixel(float gyro_reading, float max_gyro_magnitude) {
    float ratio = (gyro_reading / max_gyro_magnitude);
    ratio = (ratio > 1.0) ? 1.0 : ratio; // should never be > 1.0 but just in case
    // minimum value will be pixel 62 (just above the x axis)
    byte scaled_gyro = 62;

    scaled_gyro -= (byte)((ratio) * (GRAPH_y_AXIS_HEIGHT_PX - 1));
    return scaled_gyro;
}

void reset_512_byte_block_buffer(byte* buffer, size_t number_of_blocks) {
    memset(buffer, 0x0, 512 * number_of_blocks);
}
