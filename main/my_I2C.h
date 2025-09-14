/*
A baremetal I2C library. Highly optimized for up to ~730K bits/sec speeds
*/
#ifndef MY_I2C_H // header guard
#define MY_I2C_H
#include "driver/gpio.h"
#include "esp_rom_sys.h"

typedef uint8_t byte;

// these are the normal SDA/SCL pins, but will be used as any other GPIO pins
#define I2C_SDA GPIO_NUM_21
#define I2C_SCL GPIO_NUM_22

typedef enum {
    READ = 0x1,
    WRITE = 0x0
} READ_OR_WRITE;

/**
 * @brief Initialize the I2C bus lines (SDA, SCL) on the ESP32.
 *
 * Configures SDA and SCL pins for open-drain mode with pull-ups enabled.
 * Forces the bus into the STOP (idle) condition to ensure the first transfer works reliably.
 */
void I2C_init(void);

/**
 * @brief Read a single byte from the I2C slave.
 *
 * @param ack  If true, sends ACK after reading (request more data).
 *             If false, sends NACK (end of transmission).
 * @return The byte read from the slave.
 */
byte I2C_read_byte(bool ack);

/**
 * @brief Send a sequence of bytes to an I2C slave.
 *
 * @param slave_address       7-bit address of the I2C slave.
 * @param stream_of_bytes     Pointer to the array of bytes to send.
 * @param number_of_bytes_to_send  Number of bytes to send.
 * @param rw                  READ or WRITE flag (determines operation).
 * @param start_transmission  If true, sends a START condition first.
 * @param end_transmission    If true, sends a STOP condition after transfer.
 * @return true if all bytes were successfully sent, false otherwise.
 */
bool I2C_send_byte_stream(byte slave_address, const byte *stream_of_bytes,
                        size_t number_of_bytes_to_send, READ_OR_WRITE rw,
                        bool start_transmission, bool end_transmission);

/**
 * @brief Read one byte from a specific register of an I2C slave.
 *
 * Performs a write to set the register pointer, followed by a repeated START
 * and read operation for one byte.
 *
 * @param slave_address   7-bit address of the I2C slave.
 * @param register_to_read Register address to read from.
 * @param value           Pointer to store the read byte.
 * @return true if the read succeeded, false if NACK or error occurred.
 */
bool I2C_read_one(byte slave_address, byte register_to_read, byte* value);

/**
 * @brief Read multiple bytes starting from a register of an I2C slave.
 *
 * Performs a write to set the starting register, then a repeated START
 * and sequential read of the requested number of bytes.
 *
 * @param slave_address        7-bit address of the I2C slave.
 * @param starting_register    Register address to start reading from.
 * @param number_of_bytes_to_read Number of bytes to read.
 * @param read_bytes           Buffer to store the read bytes.
 * @return true if all bytes were successfully read, false otherwise.
 */
bool I2C_read_many(byte slave_address, byte starting_register, size_t number_of_bytes_to_read, byte* read_bytes);

/**
 * @brief Check if an I2C device responds at a given address.
 *
 * @param address_of_device  7-bit address of the I2C device.
 * @return true if the device ACKs, false otherwise.
 */
bool I2C_find_device(byte address_of_device);

#endif // MY_I2C_H