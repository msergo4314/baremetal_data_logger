/*
A baremetal I2C library. Highly optimized for up to ~730K bits/sec speeds (writing the whole display at once)
*/
#ifndef MY_I2C_H // header guard
#define MY_I2C_H
#include "driver/gpio.h"
#include "esp_rom_sys.h"

typedef uint8_t byte;

// these are the normal SDA/SCL pins, but will be used as any other GPIO pins
#define I2C_SDA GPIO_NUM_21
#define I2C_SCL GPIO_NUM_22
#define _NOP() __asm__ __volatile__ ("nop")

typedef enum {
    READ = 0x1,
    WRITE = 0x0
} READ_OR_WRITE;

void I2C_init(void);
byte I2C_read_byte(bool ack);
bool I2C_send_byte_stream(byte slave_address, const byte *stream_of_bytes,
                          size_t number_of_bytes_to_send, READ_OR_WRITE rw,
                          bool start_transmission, bool end_transmission);
bool I2C_read_one(byte slave_address, byte register_to_read, byte* value);
bool I2C_read_many(byte slave_address, byte starting_register, size_t number_of_bytes_to_read, byte* read_bytes);
bool find_device(byte address_of_device);

#endif // MY_I2C_H