#ifndef MY_SPI_H
#define MY_SPI_H
#include "driver/gpio.h"

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
    void (*idle_clock)(void);
    void (*capture_data)(void);
    void (*shift_data)(void);
    void (*cs_assert)(gpio_num_t);
    void (*cs_idle)(gpio_num_t);
    void (*mosi_idle)(void);
} SPI_device_t;


bool SPI_init(void);
// attatch a SPI device to utilize SPI functions
SPI_device_t* SPI_attach_device(gpio_num_t cs, SPI_MODE mode, SPI_CS_ACTIVE idle);
void SPI_transfer_block(gpio_num_t cs, const byte* tx_buffer, byte* rx_buffer, size_t number_of_bytes);
void SPI_transmit_to_slave(gpio_num_t cs, const byte* bytes_to_send, size_t number_of_bytes);
void SPI_receive_from_slave(gpio_num_t cs, byte* read_data, size_t number_of_bytes);
void SPI_clock_toggle(gpio_num_t cs, size_t num_cycles);
void SPI_set_slow_mode(bool enable_slow_mode);
void SPI_set_mosi(bool mosi_logic_level);
#endif