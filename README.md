| Supported Targets | ESP32 |

# Baremetal data logger

A project that uses a ssd1306 OLED display, MPU6050 accelerometer, and 4 GB SD card to log acceleration data in real time using only user-created libraries and manual implementation of I2C and SPI protocols using the ESP32-WROOM from FREENOVE. The main goal is to achieve a deep understanding of the protocols used for the sensors by bit-banging the implementations.

![final design layout](https://github.com/msergo4314/baremetal_data_logger/blob/master/images/demo.jpg)

## Hardware required

-ESP32 development board. I have an ESP32-WROOM from Freenove which I got [on amazon](https://www.amazon.ca/Freenove-ESP32-WROOM-Compatible-Wireless-Detailed/dp/B0C9THDPXP?crid=198OSIGBJDI3Rdib=eyJ2IjoiMSJ9_RNb2EnB-vTx26Y_kdThalmX3FU6JsrCFgCe6Pp3BjdiJ3rSPFwcWXmg7JhW_k-Uhs2DIxjkTxl8TVqrOkIhPIqTTykvuwskMtEYbAexgBFr3vn79kzCbFaOEE2WeHtZuk5Cvj0ZxAG3_Hio0AwUnQYg39VCaFLM_aYXhMgUg0kfK9B_xmNGQGi6__Nx8_OPiArDBP2Ogq6ts5TjK4jN0t__8Jy_Hw-jO6xWCmEnsvrDnKnvkxo9IEEFMf8WkpjH2lU23Cohr_um5Q_q5nYJDwVvmRvKmMY2realUr6lHNI.UVQG43AL5AnkkFRP_ahrdaAwv5G_Ul68BJW8abd-D4w&dib_tag=se&keywords=esp32&qid=1756687508&sprefix=es%2Caps%2C149&sr=8-7&th=1)

These are the sensors I used. The project is less about the sensors than interfacing with them from scratch, so in theory you can use just about any I2C sensor and SPI sensor with the same I2C and SPI libraries. But you would have to make your own drivers to wrap the protocol layer, and that means consulting the datasheets. Having a breakout board for each sensor is not necessary in theory but will make this project much simpler. The devices on their own are tiny and would be basically impossible to work with alone (the MPU measures about 2x2 mm). So unless you can fabricate your own PCBs, just get breakout boards.

-SSD1306 OLED display. I got one from Adafruit ([here](https://www.adafruit.com/product/326)).  The STEMMA QT cables that Adafruit makes were really useful for keeping the wiring clean and sturdy so I reccomend those too.

-MPU6050 Accelerometer/Gyroscope. I got a breakout board from Adafruit ([here](https://www.adafruit.com/product/3886)) which also uses the STEMMA cables.

-4 GB SPI SD card. [link](https://www.adafruit.com/product/6039). The STEMMA cables are for I2C, so this breakout board can't use them. I had to solder in the provided header pins which makes the project a little messier.

## How to use

You will need the ESP-idf since this an ESP-idf project file. I used the VScode extension, which works nicely. You just need the files in the main folder and the rest will be done automatically. I specifically chose to use the ESP-idf environment instead of the Arduino IDE because it is closer to proffessional development. Additionally, ESP-idf exclusively uses C, while the Arduino IDE uses a mix of C/C++. I have only ever used C and prefer it, so ESP-idf is more attractive in that way. 

One drawback of the ESP-idf is that Adafruit does NOT provide their own libraries for these devices for ESP-idf. They provide Arduino libraries and some other options but not ESP-idf components. This is not an issue for this project, since every driver is 100% custom made, but it means that working with the devices using ESP-idf would require downloading some wrapper components if you wanted to use pre built libraries (you normally do).

## main folder contents

```
├── CMakeLists.txt
├── main
│   ├── CMakeLists.txt
│   ├── main.c
│   ├── mpu6050_I2C.c
│   ├── mpu6050_I2C.h
│   ├── my_I2C.c
│   ├── my_I2C.h
│   ├── my_SPI.c
│   ├── my_SPI.h
│   ├── SD_card_SPI.c
│   ├── SD_card_SPI.h
│   ├── SD_card.c
│   ├── SD_card.h
│   ├── ssd1306_I2C.c
│   └── ssd1306_I2C.h
└── README.md           <--This is the file you are currently reading
```

The project **baremetal_data_logger** contains one source file in C language [main.c](main/main.c). The file is located in folder [main](main). However, main.c is only responsible for the high level abstraction -- The I2C and SPI protocols are located in my_I2C.h, my_SPI.h, and the corresponding C files (my_I2C.c and my_SPI.c). These provide a bit banged implementation of each protocol. Each of the devices has it's own header and C file which depend on the corresponding protocol headers. The OLED and MPU use I2C while the SD card uses SPI. The files for the devices contain wrappers for some of the protocol functions and provide high level functionality which is used in main (see below for each).

Below are the explanations of the files in the main folder with implementation details.

# main.c

The main.c file brings together all custom drivers into a real-time data logging system using FreeRTOS. The system is organized into three concurrent tasks synchronized by queues and a hardware timer:

- MPU_task — polls the MPU6050 at 100 Hz using a hardware timer ISR and pushes data (accel, gyro, temperature, timestamp) to the SD and OLED queues.

- OLED_task — refreshes the SSD1306 display at 20 Hz, showing real-time accel/gyro magnitudes on scrolling graphs and die temperature text.

- SD_task — buffers queue_data entries into 512 B blocks and performs multi-block writes to the SD card for efficient, continuous logging ($>$2 hours before overwrite).

A setup_task handles driver initialization, verifies I²C and SPI performance, and tests read/write operations before the application starts. Communication between tasks uses FreeRTOS queues and direct notifications to guarantee deterministic timing.

Measured performance:

I²C stable up to 600 kHz vs the 400 KHz "fast" mode.

SPI transfers at 3.2 MHz with reliable multi-block read/write. This is significantly slower than the 26 MHz SPI speeds available using the on-board hardware peripheral, and is a direct result of the number of instructions executed per transmission.

End-to-end latency $\sim$10 ms for IMU sampling and $<$50 ms for OLED refresh.

# my_I2C.h and my_I2C.c

This module implements a lightweight I²C communication driver for the ESP32.

-Provides initialization, single-byte reads/writes, and multi-byte transfer functions.

-Low-level ESP-IDF driver functions are wrapped into a simpler interface for common use.

-Acknowledge handling, stop/restart conditions, and transfer retries are managed internally.

-Used as the base layer for higher-level devices such as the MPU6050 IMU and SSD1306 drivers.

# SSD1306_I2C.h and SSD1306_I2C.c

This module controls an SSD1306-based OLED display over I²C.

-Uses the my_I2C interface for communication with the display hardware.

-Initializes the display with the correct sequence of commands (addressing mode, contrast, scan direction, etc.).

-Provides functions for sending commands and pixel data, drawing to the screen, and refreshing the display.

-Provides a very basic graphics library with functions for drawing rectangles, lines, and setting pixels

-Utilizes a GDDRAM image to store and update the memory of the OLED instead of reading from the OLED (would be slow)

# my_SPI.h and my_SPI.c

This module provides a simplified SPI driver interface for the ESP32.

-Initializes the SPI bus with configurable clock, polarity, and phase (SPI modes).

-Functions to send and receive single or multiple bytes.

-Used by higher-level peripherals that communicate over SPI, such as SD cards.

-Abstracts ESP-IDF SPI driver calls into a cleaner C API for embedded applications.

# mpu6050_I2C.h and mpu6050_I2C.c

This module interfaces with the MPU6050 6-axis IMU (3-axis accelerometer + 3-axis gyroscope).

-Supports initialization, register configuration, and data acquisition.

-Provides APIs to set accelerometer/gyroscope ranges, digital low-pass filter frequency, and sample rate.

-Implements conversion of raw register values into floating-point acceleration (g), rotation rate (°/s), and temperature (°C).

-Relies on my_I2C for communication with the MPU6050 over I²C.

-Includes timing considerations: e.g., after issuing a reset, the driver uses esp_rom_delay_us() instead of FreeRTOS delays to avoid NACKs during reboot.

# SD_card_SPI.h and SD_card_SPI.c

This module implements raw SD card communication over SPI.

-Supports initialization of SD/SDHC cards into SPI mode.

-Implements single-block (CMD17, CMD24) and multi-block (CMD18, CMD25) read/write operations.

-Handles command/response tokens, data blocks, CRCs (simplified/disabled in SPI mode), and stop tokens for multi-block writes.

-Provides polling for busy/ready states during writes, ensuring data integrity.

-Uses the my_SPI interface as the transport layer.

-Designed for raw block access (no FAT/exFAT filesystem layer). This makes it suitable for logging raw data or as a building block for a future filesystem implementation.

# SD_card.h and SD_card.c

-This is the same as above but uses the hardware peripherals of the ESP32 instead of software for the SPI transfers. The speed of the SCL line is 20 MHz and transfers are about 2x faster than the Bit banged version (more optimization likely needed)

-Note that the CmakeLists.txt file in main must be updated to include "SD_card.c" as a source if you want to use these files

[clutch link](https://elm-chan.org/docs/mmc/mmc_e.html)