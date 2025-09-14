#ifndef MY_SPI_H
#define MY_SPI_H
#include "driver/gpio.h"
#include "esp_rtc_time.h" // to estimate frequency
#include "soc/gpio_struct.h"
#include "soc/gpio_reg.h"

#define SPI_MAX_ATTACHED_DEVICES 8 // how many devices can be used at once

typedef uint8_t byte;

// Uses the default pins, but should work with any GPIO pins in the range of 0-31
#define SPI_CLK 18
#define SPI_MISO 19
#define SPI_MOSI 23

typedef enum mode {
    MODE_0 = 0b00,
    MODE_1 = 0b01,
    MODE_2 = 0b10,
    MODE_3 = 0b11,
} SPI_MODE;

/**
* @brief Set the SPI chip select (CS) pin low.
*
* @param cs GPIO number for the CS pin. Must be in the range 0-31
*/
inline void SPI_cs_low(gpio_num_t CS) {GPIO.out_w1tc = 1U << CS;}
/**
* @brief Set the SPI chip select (CS) pin high.
*
* @param cs GPIO number for the CS pin. Must be in the range 0-31
*/
inline void SPI_cs_high(gpio_num_t CS) {GPIO.out_w1ts = 1U << CS;}

/**
* @brief Initialize the SPI bus for attatched devices.
*
* Configures the SPI bus with default pins and sets the SPI frequency to the max possible value
* @return True on success, else false
*/
bool SPI_init(void);
/**
* @brief Attatched a device to use for SPI communication
* @param cs the chip select GPIO number (must be 0-31)
* @param mode SPI mode of the device (0-3)
*/
void SPI_attach_device(gpio_num_t cs, SPI_MODE mode);

/**
* @brief Sends a stream of bytes of length number_of_bytes via SPI and stores the response data into rx_buffer
*
* Does not interact with the chip select pins at all
* @param tx_buffer the stream of bytes to transmit
* @param rx_buffer the buffer that will hold the recieved bytes from the MISO line (must have same length as tx or more)
* @param number_of_bytes the number of bytes to send from tx_buffer
* @param mode SPI mode of the communication
*/
void SPI_transfer_block(const byte* tx_buffer, byte* rx_buffer, size_t number_of_bytes, SPI_MODE mode);
/**
* @brief Transmit data to an SPI slave device.
*
* @param data Pointer to the data buffer to send.
* @param length Number of bytes to send.
* @param mode SPI mode (0–3).
*/
void SPI_transmit_to_slave(const byte* tx_buffer, size_t number_of_bytes, SPI_MODE mode);
/**
* @brief Receive data from an SPI slave device.
*
* @param data Pointer to the buffer to store received data.
* @param length Number of bytes to receive.
* @param mode SPI mode (0–3).
*/
void SPI_receive_from_slave(byte* rx_buffer, size_t number_of_bytes, SPI_MODE mode);
/**
* @brief set the MOSI line to a logic level
* @param mosi_logic_level 0 for logic 0, 1 for logic 1
**/
void SPI_set_mosi(bool mosi_logic_level);
// return (estimated) clock speed in Hz by sending 1000 bytes
/**
 * @brief returns the current SPI clock speed in Hz
 * 
 * times transmission of a small stream of bytes to estimate clock speeds
 * @return the current clock frequency in Hz
 **/
size_t SPI_get_clock_speed_Hz(void);
/**
 * @brief returns the highest possible SPI clock speed for the current CPU speed
 * 
 * @return maximum clock frequency in Hz
 **/
size_t SPI_get_max_frequency(void);
/**
* @brief Transfer one byte over SPI.
*
* Sends one byte to the SPI slave while receiving one byte in return.
*
* @param data Byte to send.
* @param mode SPI mode (0–3).
* @return The received byte.
*/
byte SPI_transfer_byte(byte data, SPI_MODE mode);
/**
* @brief Set the SPI clock to a target frequency in kHz
*
* approximates by calling SPI_get_clock_speed_Hz() and converging on the desired value.
* Becomes less accurate as the desired frequency increases due to hardware limitations.
* @param desired_frequency_kHz the target frequency measured in kHz. 
* If greater than the max frequency, the max frequency will be used instead
*/
void SPI_set_frequency(uint16_t desired_frequency_kHz);
#endif