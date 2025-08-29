#ifndef my_I2C // header guard

#define my_I2C
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "rom/ets_sys.h"

typedef uint8_t byte;

// these are the normal SDA/SCL pins, but will be used as any other GPIO pins
#define I2C_SDA GPIO_NUM_21
#define I2C_SCL GPIO_NUM_22
#define _NOP() __asm__ __volatile__ ("nop");

typedef enum {
    READ = 0x1,
    WRITE = 0x0
} READ_OR_WRITE;

/*

SEND MSB first for data transmissions

DATA line can never change when SCL goes high/low -- SDA transitions when clock is pulled low
if SDA transitions when SDA is HIGH we have a start/stop condition
START - SDA transitions LOW, SCL transitions LOW
STOP - SCL transitions HIGH and remains HIGH. SDA transitions HIGH and remanins HIGH

I2C steps:

1 - START condition -- master claims the bus
2 - master sends slave address (6:0)
3 - master sends R/W bit -- 0 for read, 1 for write
4 - slave responds with ACK when ready
5 - data is transmitted
6 - ACK sent to confirm transmission
7 - STOP condition terminates the transmission

SDA and SCL are both HIGH when idle (pulled up)

for each byte of data received by the slave (including address), an ACK is sent. 0 indicates ACK, 1 is NACK (receiver must avtively pull SDA low)

NOTE: one frame can have multiple bytes of data and therefore multiple ACKs

When bus is idle, SCL is inactive (pulled HIGH, remains HIGH)

*/

// make functions static if they won't be used in external files
static inline void sda_high(void){ gpio_set_level(I2C_SDA, 1); } // releases line in OD mode
static inline void sda_low(void){ gpio_set_level(I2C_SDA, 0); }
static inline void scl_high(void){ gpio_set_level(I2C_SCL, 1); }
static inline void scl_low(void){ gpio_set_level(I2C_SCL, 0); }

// static inline void I2C_delay(void) {_NOP();} // can also do fixed time interval using ets_delay_us() or vTaskDelay()

static inline void I2C_delay(void) {ets_delay_us(5);} // standard I2C uses 4 microsecond wait times

static void I2C_start(void) {
    // give the lines time to fully rise to 3.3V (1 us works in testing, best to do more)
    sda_high(); I2C_delay();
    scl_high(); I2C_delay();

    sda_low(); I2C_delay();
    scl_low();
    I2C_delay();
    return;
}

static void I2C_stop(void) {
    sda_low(); I2C_delay();

    scl_high(); I2C_delay();
    sda_high(); I2C_delay();
    return;
}

void I2C_init(void) {
    gpio_reset_pin(I2C_SCL);
    gpio_reset_pin(I2C_SDA);
    gpio_config_t I2C_config = {
        // both SDA and SCL lines have the same settings
        .pin_bit_mask = (1ULL << I2C_SDA) | (1ULL << I2C_SCL),
        // we need INPUT and OUTPUT modes since we are reading the lines but also setting them
        .mode = GPIO_MODE_INPUT_OUTPUT_OD, // open drain mode since logic 1s are not driven high
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_ENABLE // pull up both lines to 3.3V
        // .intr_type = GPIO_INTR_ANYEDGE // no need for interrupts since we will manually implement START/STOP conditions
    };
    gpio_config(&I2C_config); // sets up the lines
    // drive both lines high for idle state
    sda_high();
    scl_high();
    I2C_stop(); // force the bus to be idle. Without this, the first communication attempt will not work (but second will)
    return;
}

bool I2C_write_byte(byte byte_to_write) {
    // NOTE: SDA can only transition when SCL is LOW and must be held when SCL is HIGH

    // write MSBs first --> 7 down to 0
    for(int i=7; i>=0; i--) {
        // bitwise AND with left shifted 1 to pick a single bit
        (byte_to_write & (1 << i)) ? sda_high() : sda_low(); // write SDA HIGH/LOW depending on the bits
        I2C_delay(); // delay to let the value of SDA propagate
        scl_high(); I2C_delay(); // set SCL high, delay helps for rise time. At this point, the slave will read SDA
        scl_low(); // clock must be low when SDL transitions
        I2C_delay();
    }
    sda_high(); // release SDA for slave ACK to pull it low
    scl_high(); I2C_delay(); // set SCL high, then read SDA for ACK/NACK
    bool ack = (gpio_get_level(I2C_SDA) == 0);
    scl_low(); // set SCL low if we need to write more bits using this function
    // I2C_delay();
    return ack;
}

// Note: does not have any START/STOP conditions, just sends the byte
static bool transmit_address_and_RW(byte address_of_slave, READ_OR_WRITE rw) {
    // the address needs to be 7 bits long. Left shift and insert read/write bit as the LSB
    return I2C_write_byte((address_of_slave << 1) | rw);
}

// ACK is used to indicate if we want to read further, NACK indicates no more transmission
byte read_byte(bool ack) {
    byte data = 0x0;
    sda_high(); // release SDA so slave can drive it
    for (byte i = 0; i < 8; i++) {
        data <<= 1; // left shift the data first
        // wait for slave to set SCL low (it may need more time)
        do {
            scl_high();
        } while (gpio_get_level(I2C_SCL) == 0); // clock stretching
        I2C_delay();
        if (gpio_get_level(I2C_SDA) == 1) data |= 1; // append a 1 on the right if SDA is high
        I2C_delay();
        scl_low();
    }
    ack ? sda_low() : sda_high(); // pull SDA low if ACK is true
    scl_high(); // toggle SCL to clock in the ACK/NACK into the slave
    I2C_delay();
    scl_low();
    sda_high();
    return (byte)data;
}

bool I2C_send_byte_stream(byte slave_address, const byte* stream_of_bytes, size_t number_of_bytes_to_send, READ_OR_WRITE rw, bool start_transmission, bool end_transmission) {
    if (start_transmission) I2C_start();
    if (!transmit_address_and_RW(slave_address, rw)) {
        printf("transmitting address and R/W resulted in NACK! Address given: %x\n", slave_address);
        return false;
    }
    for (unsigned int i = 0; i < number_of_bytes_to_send; i++) {
        if (!I2C_write_byte(stream_of_bytes[i])) {
            return false;
        }
    }
    if (end_transmission) I2C_stop();
    return true;
}

#endif // my_I2C.h