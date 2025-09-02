#include "my_SPI.h"
#define _NOP() __asm__ __volatile__ ("nop")

static inline void cs_low(gpio_num_t CS) {gpio_set_level(CS, 0);}
static inline void cs_high(gpio_num_t CS) {gpio_set_level(CS, 1);}
static inline void clk_low(void) {gpio_set_level(SPI_CLK, 0);}
static inline void clk_high(void) {gpio_set_level(SPI_CLK, 1);}

static inline void mosi_low(void) {gpio_set_level(SPI_MOSI, 0);}
static inline void mosi_high(void) {gpio_set_level(SPI_MOSI, 1);}


typedef struct {
    gpio_num_t pin_number;
    bool was_used_last;
} device_tracker;

static SPI_device_t devices[SPI_MAX_ATTACHED_DEVICES];
static size_t device_count = 0;

/*
Unlike I2C, SPI is a full duplex protocol, and both MISO and MOSI are used at the same time.
This means that even if we were only writing to the slave device, we still receive bytes from MISO
*/

// test with different values
static inline void SPI_delay(void) {for(int i = 0; i < 1; i++) {_NOP();}}

static byte SPI_transfer_byte(gpio_num_t cs, const byte data_out);
static byte get_device_index_from_cs(gpio_num_t cs);
static bool SPI_slow_mode_enabled = false;

SPI_device_t* SPI_attach_device(gpio_num_t cs, SPI_MODE mode, SPI_CS_ACTIVE idle) {
    if (device_count >= SPI_MAX_ATTACHED_DEVICES) return NULL;
    SPI_device_t* dev = &devices[device_count++];
    dev->cs_pin = cs;

    // Assign idle_clock, capture_data, shift_data based on mode
    switch(mode) {
        case MODE_0: 
            dev->idle_clock = &clk_low;
            dev->capture_data = &clk_high;
            dev->shift_data = &clk_low;
            break;
        case MODE_1: 
            dev->idle_clock = &clk_low;
            dev->capture_data = &clk_low;
            dev->shift_data = &clk_high;
            break;
        case MODE_2: 
            dev->idle_clock = &clk_high;
            dev->capture_data = &clk_low;
            dev->shift_data = &clk_high;
            break;
        case MODE_3: 
            dev->idle_clock = &clk_high;
            dev->capture_data = &clk_high;
            dev->shift_data = &clk_low;
            break;
    }

    // Assign chip select behavior
    dev->cs_idle   = (idle == CS_ACTIVE_HIGH) ? &cs_low : &cs_high;
    dev->cs_assert = (idle == CS_ACTIVE_HIGH) ? &cs_high : &cs_low;
    dev->mosi_idle = &mosi_high;
    return dev;
}

bool SPI_init(void) {
    if (device_count == 0) {
        printf("Error: cannot start SPI without any attatched devices!\n");
        return false;
    }
    for (int i = 0; i < device_count; i++) {
        gpio_num_t current = devices[i].cs_pin;
        gpio_reset_pin(current);
        gpio_set_direction(current, GPIO_MODE_OUTPUT);
        devices[i].cs_idle(current);
    }
    gpio_reset_pin(SPI_CLK);
    gpio_reset_pin(SPI_MISO);
    gpio_reset_pin(SPI_MOSI);

    gpio_set_direction(SPI_CLK, GPIO_MODE_OUTPUT);
    gpio_set_direction(SPI_MOSI, GPIO_MODE_OUTPUT);
    gpio_set_direction(SPI_MISO, GPIO_MODE_INPUT);
    // assume we are in mode 0
    mosi_high();
    return true;
}

// assume that we have already pulle the CS low/high and set correct functions for writing/latching bits
// send MSB first
static byte SPI_transfer_byte(gpio_num_t cs, const byte data_out) {
    byte index = get_device_index_from_cs(cs);
    if (index == 255) {
        printf("Error: Device was not added with SPI_attatch_device()!\n");
        return 0;
    }
    SPI_device_t device = devices[index];
    byte data_in = 0;

    for (int i = 7; i >= 0; i--) {
        // Set MOSI for this bit before it gets read
        (data_out & (1 << i)) ? mosi_high() : mosi_low();
        
        // Short delay for slave setup time
        // SPI_delay();

        // sample MISO with clock rising/falling edge
        device.capture_data();
        // SPI_delay();

        // Read input bit on MISO
        if (gpio_get_level(SPI_MISO)) {
            data_in |= (1 << i);
        }
        // NOP for the slave hold time
        // SPI_delay();
        // shift/latch next bit with falling/rising edge
        device.shift_data();
    }

    return data_in;
}


void SPI_transfer_block(gpio_num_t cs, const byte* tx_buffer, byte* rx_buffer, size_t number_of_bytes) {
    byte index = get_device_index_from_cs(cs);
    if (index == 255) {
        printf("Error: Device was not added with SPI_attatch_device()!\n");
        return;
    }
    SPI_device_t device = devices[index];
    device.cs_assert(device.cs_pin);
    for (size_t i = 0; i < number_of_bytes; i++) {
        byte out = tx_buffer ? tx_buffer[i] : 0xFF; // send dummy if NULL
        byte in  = SPI_transfer_byte(cs, out);
        if (rx_buffer) rx_buffer[i] = in;
    }
    device.mosi_idle();
    device.idle_clock();
    device.cs_idle(cs);
}

// for sending data but discarding the incoming data from the slave
void SPI_transmit_to_slave(gpio_num_t cs, const byte* bytes_to_send, size_t number_of_bytes) {
    SPI_transfer_block(cs, bytes_to_send, (byte*)NULL, number_of_bytes);
}

// for reading data but sending junk values to a slave device
void SPI_receive_from_slave(gpio_num_t cs, byte* read_data, size_t number_of_bytes) {
    SPI_transfer_block(cs, (const byte*)NULL, read_data, number_of_bytes);
}

// toggle the clock without touching MISO/MOSI lines
void SPI_clock_toggle(gpio_num_t cs, size_t num_cycles) {
    byte index = get_device_index_from_cs(cs);
    if (index == 255) {
        printf("Error: Device was not added with SPI_attatch_device()!\n");
        return;
    }
    SPI_device_t device = devices[index];
    // assume the clock is in the idle state
    for (size_t i = 0; i < num_cycles; i++) {
        device.capture_data();
        device.shift_data();
    }
    device.idle_clock();
}

void SPI_set_slow_mode(bool enable_slow_mode) {
    SPI_slow_mode_enabled = enable_slow_mode;
}

// returns 255 if not found
static byte get_device_index_from_cs(gpio_num_t cs) {
    for (int i = 0; i < device_count; i++) {
        if (devices[i].cs_pin == cs) {
            return (byte)i;
        }
    }
    return 255;
}

void SPI_set_mosi(bool mosi_logic_level) {
    mosi_logic_level ? mosi_high() : mosi_low();
}