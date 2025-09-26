#include "my_I2C.h"
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
*/

#define _NOP() __asm__ __volatile__ ("nop")

// helpers not to be used outside of this file
static inline void sda_high(void);
static inline void sda_low(void);
static inline void scl_high(void);
static inline void scl_low(void);
static inline void I2C_delay(void);

/**
 * @brief Generate an I2C START condition.
 *
 * SDA transitions from HIGH to LOW while SCL remains HIGH.
 * Leaves the bus ready for data transmission.
 */
static void I2C_start(void);

/**
 * @brief Generate an I2C STOP condition.
 *
 * SDA transitions from LOW to HIGH while SCL remains HIGH.
 * Releases the bus to idle state.
 */
static void I2C_stop(void);

/**
 * @brief Write one byte to the I2C bus.
 *
 * Sends bits MSB-first, followed by the slave ACK/NACK.
 *
 * @param byte_to_write  The data byte to transmit.
 * @return true if the slave acknowledged, false otherwise.
 */
static bool I2C_write_byte(byte byte_to_write);

/**
 * @brief Transmit a 7-bit slave address with the R/W bit.
 *
 * @param address_of_slave  7-bit I2C slave address.
 * @param rw                READ or WRITE operation.
 * @return true if the slave acknowledged, false otherwise.
 */
static inline bool transmit_address_and_RW(byte address_of_slave, READ_OR_WRITE rw);

// make functions static if they won't be used in external files ("private")
static inline void sda_high(void){ gpio_set_level(I2C_SDA, 1); } // releases line in OD mode
static inline void sda_low(void){ gpio_set_level(I2C_SDA, 0); }
static inline void scl_high(void){ gpio_set_level(I2C_SCL, 1); }
static inline void scl_low(void){ gpio_set_level(I2C_SCL, 0); }

// 5 NOPs is the lowest possible delay we can have before the SSD1306 NACKs consistently
// more NOPs safer -- especially for longer wires
static inline void I2C_delay(void) {for (volatile int i = 0; i < 8; i++) { _NOP(); }}
// static inline void I2C_delay(void) {esp_rom_delay_us(2);} // standard I2C uses 4 microsecond wait times

static bool had_init = false;

void I2C_init(void) {
    if (had_init) {
        return;
    }
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
    I2C_stop(); // force the bus to be idle. Without this, the first communication attempt will not work (but second will)
    had_init = true;
    return;
}

byte I2C_read_byte(bool ack) {
    byte data = 0x0;
    sda_high(); // release SDA so slave can drive it
    for (byte i = 0; i < 8; i++) {
        data <<= 1; // left shift the data first
        // wait for slave to set SCL low (it may need more time)
        do {
            scl_high();
        } while (gpio_get_level(I2C_SCL) == 0); // clock stretching
        I2C_delay(); // may not need this since SCL just got set to 0
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

bool I2C_send_byte_stream(byte slave_address, const byte *stream_of_bytes,
                          size_t number_of_bytes_to_send, READ_OR_WRITE rw,
                          bool start_transmission, bool end_transmission) {
    if (!stream_of_bytes) {
        printf("Passed NULL pointer to I2C_send_byte_stream\n");
        return false;
    }
    if (start_transmission) {
        I2C_start();
        if (!transmit_address_and_RW(slave_address, rw)) {
            printf("transmitting address and R/W resulted in NACK! Address given: %x\n", slave_address);
            return false;
        }
    }
    for (unsigned int i = 0; i < number_of_bytes_to_send; i++) {
        if (!I2C_write_byte(stream_of_bytes[i])) {
            printf("Failed to write byte %u\n", i);
            return false;
        }
    }
    if (end_transmission) {
        I2C_stop();
    }
    return true;
}


bool I2C_read_one(byte slave_address, byte register_to_read, byte* value) {
    if (!value) {
        printf("passed NULL pointer\n");
        return false;
    }

    I2C_start();
    if (!transmit_address_and_RW(slave_address, WRITE)) { I2C_stop(); return false; }
    // slave must ACK on the register we want to read
    if (!I2C_write_byte(register_to_read)) { I2C_stop(); return false; }
    // send a new start condition for the read (bus is still held because no STOP)
    I2C_start();
    if (!transmit_address_and_RW(slave_address, READ)) { I2C_stop(); return false; }

    *value = I2C_read_byte(false); // false ==> NACK after single byte read
    I2C_stop();
    return true;
}

bool I2C_read_many(byte slave_address, byte starting_register, size_t number_of_bytes_to_read, byte* read_bytes) {
    if (!read_bytes) {
        printf("passed NULL pointer\n");
        return false;
    }

    I2C_start();
    if (!transmit_address_and_RW(slave_address, WRITE)) { I2C_stop(); return false; }
    // slave must ACK on the register we want to read
    if (!I2C_write_byte(starting_register)) { I2C_stop(); return false; }
    // send a new start condition for the read (bus is still held because no STOP)
    I2C_start();
    if (!transmit_address_and_RW(slave_address, READ)) { I2C_stop(); return false; }

    // ACK all bytes except the last
    for (size_t i = 0; i < number_of_bytes_to_read - 1; i++) {
        read_bytes[i] = I2C_read_byte(true);
    }
    // NACK the final byte to indicate we are done reading
    read_bytes[number_of_bytes_to_read] = I2C_read_byte(false);
    I2C_stop();
    return true;
}

bool I2C_find_device(byte address_of_device) {
    I2C_start();
    bool success = transmit_address_and_RW(address_of_device, WRITE);
    I2C_stop();
    return success;
}

static void I2C_start(void) {
    /*
    START condition is defined as SDA transitioning HIGH to LOW while SCL remains HIGH
    */
    sda_high();
    scl_high();
    // give the lines time to fully rise to 3.3V (1 us works in testing)
    esp_rom_delay_us(1);

    sda_low();

    // setting SCL low is not part of the start but is necessary for the subsequent data transmissions
    scl_low();
    return;
}

static void I2C_stop(void) {
    /*
    STOP condition is defined as SDA transitioning from LOW to HIGH while SCL remains HIGH.
    delays are not necessary here since the function is to spec regardless
    */
    sda_low(); //I2C_delay();
    scl_high(); //I2C_delay();
    sda_high(); 
    
    // the bus should be free for a small period before we can START again
    esp_rom_delay_us(1);
    return;
}

static bool I2C_write_byte(byte byte_to_write) {
    /* 
    NOTE: SDA can only transition when SCL is LOW and must be held when SCL is HIGH
    write MSBs first --> 7 down to 0
    SCL MUST be LOW when this function is called
    */
    for(int i = 7; i >= 0; i--) {
        // bitwise AND with left shifted 1 to pick a single bit
        (byte_to_write & (1 << i)) ? sda_high() : sda_low(); // write SDA HIGH/LOW depending on the bits
        /*
        can only delay if you write a 1 to SDA since rise times >> fall times
        However, this is not to spec and a little risky for small performance gains.
        */
        /*
        set SCL high for fixed time period. At this point, the slave will read SDA
        SDA must be stable at this point.
        */
        scl_high();
        I2C_delay(); 
        scl_low(); // clock must be low when SDL transitions

        /*
        Without the delay, the high period is longer than the low period
        I2C actually doesn't care about the duty cycle of the 
        */
        // I2C_delay(); // UNCOMMENT FOR EVEN CLOCK DUTY CYCLE. 
    }
    sda_high(); // release SDA for slave ACK to pull it low
    scl_high(); 
    I2C_delay(); // set SCL high, then read SDA for ACK/NACK
    bool ack = (gpio_get_level(I2C_SDA) == 0);
    scl_low(); // set SCL low if we need to write more bits using this function
    return ack;
}

// Note: does not have any START/STOP conditions, just sends the byte
static inline bool transmit_address_and_RW(byte address_of_slave, READ_OR_WRITE rw) {
    // the address needs to be 7 bits long. Left shift and insert read/write bit as the LSB
    return I2C_write_byte((address_of_slave << 1) | rw);
}
