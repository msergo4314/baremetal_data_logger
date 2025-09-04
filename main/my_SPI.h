#ifndef MY_SPI_H
#define MY_SPI_H
#include "driver/gpio.h"
#include "esp_rtc_time.h" // to estimate frequency
#include "soc/gpio_struct.h"
#include "soc/gpio_reg.h"

#define SPI_MAX_ATTACHED_DEVICES 8

typedef uint8_t byte;

#define SPI_CLK 18
#define SPI_MISO 19
#define SPI_MOSI 23

// #define SPI_CS_0 5

typedef enum mode {
    MODE_0 = 0b00,
    MODE_1 = 0b01,
    MODE_2 = 0b10,
    MODE_3 = 0b11,
} SPI_MODE;

// if the chip select for a device is LOW or HIGH when idle
typedef enum {
    CS_ACTIVE_LOW,
    CS_ACTIVE_HIGH
} SPI_CS_ACTIVE;

// stores the functions a SPI slave device will require to work
typedef struct {
    gpio_num_t cs_pin;
    SPI_MODE mode;
} SPI_device_t;

// NOTE: CS must be in range 0-31
inline void SPI_cs_low(gpio_num_t CS) {GPIO.out_w1tc = 1U << CS;}
inline void SPI_cs_high(gpio_num_t CS) {GPIO.out_w1ts = 1U << CS;}

bool SPI_init(void);
// attatch a SPI device to utilize SPI functions
void SPI_attach_device(gpio_num_t cs, SPI_MODE mode);
void SPI_transfer_block(const byte* tx_buffer, byte* rx_buffer, size_t number_of_bytes, SPI_MODE mode);
void SPI_transmit_to_slave(const byte* tx_buffer, size_t number_of_bytes, SPI_MODE mode);
void SPI_receive_from_slave(byte* rx_buffer, size_t number_of_bytes, SPI_MODE mode);
void SPI_set_mosi(bool mosi_logic_level);
bool SPI_wait_for_value(byte target_value, byte dummy_value, size_t max_iterations, SPI_MODE mode);
// return (estimated) clock speed in Hz by sending 1000 bytes
size_t SPI_get_clock_speed_Hz(void);
size_t SPI_get_max_frequency(void);
byte SPI_transfer_byte(byte data, SPI_MODE mode);
void SPI_set_frequency(uint16_t desired_frequency_kHz);
#endif