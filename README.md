| Supported Targets | ESP32 |

# Baremetal data logger

A project that uses a ssd1306 OLED display, MPU6050 accelerometer, and 4 GB SD card to log acceleration data in real time using only user-created libraries and manual implementation of I2C and SPI protocols using the ESP32-WROOM from FREENOVE. The main goal is to achieve a deep understanding of the protocols used for the sensors by bit-banging the implementations.

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
│   ├── ssd1306_I2C.c
│   └── ssd1306_I2C.h
└── README.md                  This is the file you are currently reading
```

The project **baremetal_data_logger** contains one source file in C language [main.c](main/main.c). The file is located in folder [main](main). However, main.c is only responsible for the high level abstraction -- The I2C and SPI protocols are located in my_I2C.h, my_SPI.h, and the corresponding C files (my_I2C.c and my_SPI.c). These provide a bit banged implementation of each protocol. Each of the devices has it's own header and C file which depend on the corresponding protocol headers. The OLED and MPU use I2C while the SD card uses SPI. The files for the devices contain wrappers for some of the protocol functions and provide high level functionality which is used in main (see below for each).

Below are the explanations of the files in the main folder with implementation details.

# my_I2C.h and my_I2C.c

# SSD1306_I2C.h and SSD1306_I2C.c

# my_SPI.h and .c

# mpu6050.h and mpu6050.c

# SD_card.h and SD_card.c

the datasheet for the SD card is essentially useless because the SD card uses a protocol defined by the SD association. Unfortuantely, the documentation pdf was 500 pages long, and I wasn't going through all that

[clutch link](https://elm-chan.org/docs/mmc/mmc_e.html)