#include "my_SPI.h"
#define _NOP() __asm__ __volatile__ ("nop")

// write directly to the registers instead of gpio_set_level which is slow
static inline void clk_low(void) {GPIO.out_w1tc = 1U << SPI_CLK;}
static inline void clk_high(void) {GPIO.out_w1ts = 1U << SPI_CLK;}

static inline void mosi_low(void) {GPIO.out_w1tc = 1U << SPI_MOSI;}
static inline void mosi_high(void) {GPIO.out_w1ts = 1U << SPI_MOSI;}

// shift right to get the pin of interest, then grab LSB
static inline bool miso_read(void) {return ((GPIO.in >> SPI_MISO) & 0x1);}

static SPI_device_t devices[SPI_MAX_ATTACHED_DEVICES];
static size_t device_count = 0;
static size_t half_cyle_NOP_delay_global = 0; // setting to 0 will still result in 1 NOP due to sample and hold times

// set both in init
static double NOP_time_ns_global = 0.0;
static double current_Hz_global = 0;
static double max_Hz_global = 0.0;
static double overhead_time_ns_global = 0.0;

static inline byte send_byte_mode0(byte data_out);
static inline byte send_byte_mode1(byte data_out);
static inline byte send_byte_mode2(byte data_out);
static inline byte send_byte_mode3(byte data_out);
/*
Unlike I2C, SPI is a full duplex protocol, and both MISO and MOSI are used at the same time.
This means that even if we were only writing to the slave device, we still receive bytes from MISO
*/

// used to slow down SPI speeds when required by drivers
static inline void SPI_half_cycle_delay(void) {for(size_t i = 0; i < half_cyle_NOP_delay_global; i++) {_NOP();}}
// single NOP if needed
static inline void SPI_NOP(void) {_NOP();}

// static byte SPI_transfer_byte(const byte data_out, void (*capture_data)(void), void (*shift_data)(void));

static byte get_device_index_from_cs(gpio_num_t cs);

void SPI_attach_device(gpio_num_t cs, SPI_MODE mode) {
    if (device_count >= SPI_MAX_ATTACHED_DEVICES) {
        printf("Too many devices attached\n");
        return;
    }
    if (cs >= GPIO_NUM_32) {
        printf("must use pins 0-31 for chip select pins!");
        return;
    }
    if (get_device_index_from_cs(cs) != 255) {
        printf("Device already added\n");
        return;
    }
    SPI_device_t* dev = &devices[device_count++];
    dev->cs_pin = cs;
    return;
}

bool SPI_init(void) {
    if (device_count == 0) {
        printf("Error: cannot start SPI without any attached devices!\n");
        return false;
    }
    for (int i = 0; i < device_count; i++) {
        gpio_num_t current = devices[i].cs_pin;
        gpio_reset_pin(current);
        gpio_set_direction(current, GPIO_MODE_OUTPUT);
    }
    gpio_reset_pin(SPI_CLK);
    gpio_reset_pin(SPI_MISO);
    gpio_reset_pin(SPI_MOSI);

    gpio_set_direction(SPI_CLK, GPIO_MODE_OUTPUT);
    gpio_set_direction(SPI_MOSI, GPIO_MODE_OUTPUT);
    gpio_set_direction(SPI_MISO, GPIO_MODE_INPUT);
    
    current_Hz_global = SPI_get_clock_speed_Hz();
    max_Hz_global = current_Hz_global;

    /*
    right now we have 0 extra NOPs in the clock cycle, and the period is the lowest possible.
    thus, f(0 NOPS) = current_Hz_global and T(0 NOPs) = 1 / fmax;

    However, the scaling is not fully linear since for a small # of NOPs loop functions (bit shifting, reads, writes) dominate
    while for high numbers NOP times dominate

    therefore the formula we should use for transmission clock periods is not T (n NOPS) = (NOPs * nop_time)
    but instead T (n NOPS) = (NOPs * nop_time) + fixed loop time (y = mx + b)
    we just need two points to find the equation so we will do it quickly in the init using NOPs = 0 and 50

    y = mx + b = (y2-y1) / (x2-x1) + b --> sub in a point to solve for b

    */    
    double T1, T2; // use ns for all

    // we already timed 0 NOPs above
    T1 = (1.0 / current_Hz_global) * 1e9;

    // set NOPs to 50 quickly (100 per loop)
    half_cyle_NOP_delay_global = 50;
    T2 = (1.0 / SPI_get_clock_speed_Hz()) * 1e9;
    half_cyle_NOP_delay_global = 0; // back to 0
    // NOP time is the slope
    NOP_time_ns_global = (((T2 - T1)) / (2 *(50 - 0))); // we do each NOP twice per clock period

    // T1 = m * (0) + b --> T1 = b
    overhead_time_ns_global = T1;
    return true;
}

void SPI_transfer_block(const byte* tx_buffer, byte* rx_buffer, size_t number_of_bytes, SPI_MODE mode) {
    switch(mode) {
        case MODE_0:
            if (tx_buffer && rx_buffer) {
                for (size_t i = 0; i < number_of_bytes; i++) {
                    byte out = tx_buffer[i];
                    byte in  = send_byte_mode0(out);
                    rx_buffer[i] = in;
                }
            } else if (tx_buffer && !rx_buffer) {
                for (size_t i = 0; i < number_of_bytes; i++) {
                    byte out = tx_buffer[i];
                    send_byte_mode0(out);
                }
            } else if (!tx_buffer && rx_buffer) {
                for (size_t i = 0; i < number_of_bytes; i++) {
                    byte in  = send_byte_mode0(0xFF);
                    rx_buffer[i] = in;
                }
            } else {
                // both tx and rx NULL --> send pulses only
                for (size_t i = 0; i < number_of_bytes; i++) {
                    send_byte_mode0(0xFF);
                }
            }
            clk_low();
            break;
        case MODE_1:
            if (tx_buffer && rx_buffer) {
                for (size_t i = 0; i < number_of_bytes; i++) {
                    byte out = tx_buffer[i];
                    byte in  = send_byte_mode1(out);
                    rx_buffer[i] = in;
                }
            } else if (tx_buffer && !rx_buffer) {
                for (size_t i = 0; i < number_of_bytes; i++) {
                    byte out = tx_buffer[i];
                    send_byte_mode1(out);
                }
            } else if (!tx_buffer && rx_buffer) {
                for (size_t i = 0; i < number_of_bytes; i++) {
                    byte in  = send_byte_mode1(0xFF);
                    rx_buffer[i] = in;
                }
            } else {
                // both tx and rx NULL --> send pulses only
                for (size_t i = 0; i < number_of_bytes; i++) {
                    send_byte_mode1(0xFF);
                }
            }
            clk_low();
            break;
        case MODE_2:
            if (tx_buffer && rx_buffer) {
                for (size_t i = 0; i < number_of_bytes; i++) {
                    byte out = tx_buffer[i];
                    byte in  = send_byte_mode2(out);
                    rx_buffer[i] = in;
                }
            } else if (tx_buffer && !rx_buffer) {
                for (size_t i = 0; i < number_of_bytes; i++) {
                    byte out = tx_buffer[i];
                    send_byte_mode2(out);
                }
            } else if (!tx_buffer && rx_buffer) {
                for (size_t i = 0; i < number_of_bytes; i++) {
                    byte in  = send_byte_mode2(0xFF);
                    rx_buffer[i] = in;
                }
            } else {
                // both tx and rx NULL --> send pulses only
                for (size_t i = 0; i < number_of_bytes; i++) {
                    send_byte_mode2(0xFF);
                }
            }
            clk_high();
            break;
        case MODE_3:
            if (tx_buffer && rx_buffer) {
                for (size_t i = 0; i < number_of_bytes; i++) {
                    byte out = tx_buffer[i];
                    byte in  = send_byte_mode3(out);
                    rx_buffer[i] = in;
                }
            } else if (tx_buffer && !rx_buffer) {
                for (size_t i = 0; i < number_of_bytes; i++) {
                    byte out = tx_buffer[i];
                    send_byte_mode3(out);
                }
            } else if (!tx_buffer && rx_buffer) {
                for (size_t i = 0; i < number_of_bytes; i++) {
                    byte in  = send_byte_mode3(0xFF);
                    rx_buffer[i] = in;
                }
            } else {
                // both tx and rx NULL --> send pulses only
                for (size_t i = 0; i < number_of_bytes; i++) {
                    send_byte_mode3(0xFF);
                }
            }
            clk_high();
            break;
    }
}

// for sending data but discarding the incoming data from the slave
void SPI_transmit_to_slave(const byte* tx_buffer, size_t number_of_bytes, SPI_MODE mode) {
    SPI_transfer_block(tx_buffer, (byte*)NULL, number_of_bytes, mode);
}

// for reading data but sending junk values to a slave device
void SPI_receive_from_slave(byte* rx_buffer, size_t number_of_bytes, SPI_MODE mode) {
    SPI_transfer_block((const byte*)NULL, rx_buffer, number_of_bytes, mode);
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

size_t SPI_get_clock_speed_Hz() {
    // printf("Time per NOP (assuming in a loop): %.2lf ns\n", SPI_time_NOP());

    // just use mode 0 here and don't touch any CS pins to make sure we don't send anything
    clk_low();
    int num_bytes = 600;
    uint32_t start = esp_rtc_get_time_us();
    // simulate a transmission of num_bytes
    for (int i = 0; i < num_bytes; i++) {
        send_byte_mode0(0x0);
    }
    uint32_t elapsed = esp_rtc_get_time_us() - start;
    // printf("Elapsed time for %d bytes sent with SPI: %lu us (%.3f sec)\n", num_bytes, elapsed, (elapsed) * 1e-6);
    size_t frequency_estimate =  (size_t)((num_bytes * 8) / (elapsed * 1e-6));
    // printf("Estimated SPI speed: %u kHz\n\n", frequency_estimate / 1000);
    return frequency_estimate;
}

// input is frequency in kHz
void SPI_set_frequency(uint16_t desired_frequency_kHz) {
    
    // CLOCK Period = FIXED OFFSET DUE TO LOOPS
    
    if (desired_frequency_kHz * 1000 > max_Hz_global) {
        printf("Cannot exceed %.0lf Hz SPI speeds. Setting to %.0lf MHz\n", max_Hz_global, max_Hz_global);
        half_cyle_NOP_delay_global = 0;
        current_Hz_global = max_Hz_global;
        return;
    }
    if (desired_frequency_kHz < 100) {
        printf("Cannot set to < 100 KHz. Setting to 100 kHz\n");
        desired_frequency_kHz = 100;
    }

    double desired_Hz = desired_frequency_kHz * 1000;
    double desired_period_ns = (1.0 / (desired_Hz)) * 1e9;
    // T(n) = overhead_time_ns_global + (n) * 2 * (NOP_time_ns_global)

    double nop_count_f = (desired_period_ns - overhead_time_ns_global) / (2 * NOP_time_ns_global);

    if (nop_count_f < 3.0) {
        size_t best_nops = 0;
        double best_error = 1e9;

        for (size_t test = 0; test <= 2; test++) {
            half_cyle_NOP_delay_global = test;
            double test_Hz = (double)SPI_get_clock_speed_Hz();
            double error = ((test_Hz - desired_Hz) / desired_Hz);
            if (error < 0.0) error *= -1;
            if (error < best_error) {
                best_error = error;
                best_nops = test;
            }
        }

        half_cyle_NOP_delay_global = best_nops;
        current_Hz_global = (double)SPI_get_clock_speed_Hz();
        // printf("High-speed quantized region. Picked %u NOPs (%.0lf Hz)\n", best_nops, current_Hz_global);
        printf("Changed speed to %.0lf kHz\n", current_Hz_global / 1000);
        return;
    }

    half_cyle_NOP_delay_global = (size_t)(nop_count_f); // cast to int
    
    double real_Hz = (double)SPI_get_clock_speed_Hz();
    double error = ((real_Hz - desired_Hz) / desired_Hz);
    if (error < 0.0) error *= -1;
    // drive relative error to 0
    int i = 0;
    while((error >= 0.03) && i < 5) {
        if (real_Hz > desired_Hz) {
            half_cyle_NOP_delay_global++;
        } else {
            half_cyle_NOP_delay_global--;
        }
        real_Hz = (double)SPI_get_clock_speed_Hz();
        // printf("new frequency: %d kHz\n", (int)real_Hz / 1000);
        error = ((real_Hz - desired_Hz) / desired_Hz);
        if (error < 0.0) error *= -1;
        i++;
    }
    // printf("New frequency set: %lf\n", real_Hz);
    current_Hz_global = real_Hz;
    printf("Changed speed to %.0lf kHz\n", current_Hz_global / 1000);
    return;
}

size_t SPI_get_max_frequency(void) {
    return (size_t)max_Hz_global;
}

static inline byte send_byte_mode0(byte data_out) {
    byte data_in = 0;
    if (half_cyle_NOP_delay_global == 0) {
        for (int i = 7; i >= 0; i--) {
            (data_out & (1 << i)) ? mosi_high() : mosi_low();
            clk_high(); // capture
            if (miso_read()) data_in |= (1 << i);
            clk_low(); // shift
        }
    } else {
        for (int i = 7; i >= 0; i--) {
            (data_out & (1 << i)) ? mosi_high() : mosi_low();
            clk_high(); // capture
            SPI_half_cycle_delay();
            if (miso_read()) data_in |= (1 << i);
            clk_low(); // shift
            SPI_half_cycle_delay();
        }
    }
    return data_in;
}

static inline byte send_byte_mode1(byte data_out) {
    byte data_in = 0;
    if (half_cyle_NOP_delay_global == 0) {
        for (int i = 7; i >= 0; i--) {
            (data_out & (1 << i)) ? mosi_high() : mosi_low();
            clk_low(); // capture
            if (gpio_get_level(SPI_MISO)) data_in |= (1 << i);
            clk_high(); // shift
        }
    } else {
        for (int i = 7; i >= 0; i--) {
            (data_out & (1 << i)) ? mosi_high() : mosi_low();
            clk_low(); // capture
            SPI_half_cycle_delay();
            if (gpio_get_level(SPI_MISO)) data_in |= (1 << i);
            clk_high(); // shift
            SPI_half_cycle_delay();
        }
    }
    return data_in;
}

static inline byte send_byte_mode2(byte data_out) {
    byte data_in = 0;
    if (half_cyle_NOP_delay_global == 0) {
        for (int i = 7; i >= 0; i--) {
            (data_out & (1 << i)) ? mosi_high() : mosi_low();
            clk_low(); // capture
            if (gpio_get_level(SPI_MISO)) data_in |= (1 << i);
            clk_high(); // shift
        }
    } else {
        for (int i = 7; i >= 0; i--) {
        (data_out & (1 << i)) ? mosi_high() : mosi_low();
        clk_low(); // capture
        SPI_half_cycle_delay();
        if (gpio_get_level(SPI_MISO)) data_in |= (1 << i);
        clk_high(); // shift
        SPI_half_cycle_delay();
    }
    }
    return data_in;
}

static inline byte send_byte_mode3(byte data_out) {
    byte data_in = 0;
    if (half_cyle_NOP_delay_global == 0) {
        for (int i = 7; i >= 0; i--) {
            (data_out & (1 << i)) ? mosi_high() : mosi_low();
            clk_high(); // capture
            if (gpio_get_level(SPI_MISO)) data_in |= (1 << i);
            clk_low(); // shift
        }
    } else {
        for (int i = 7; i >= 0; i--) {
            (data_out & (1 << i)) ? mosi_high() : mosi_low();
            clk_high(); // capture
            SPI_half_cycle_delay();
            if (gpio_get_level(SPI_MISO)) data_in |= (1 << i);
            clk_low(); // shift
            SPI_half_cycle_delay();
        }
    }
    return data_in;
}
/*
waits for a certain byte value on MISO. Iterates up to max_iterations bytes
does not change or set the chip select
*/
bool SPI_wait_for_value(byte target_value, byte dummy_value, size_t max_iterations, SPI_MODE mode) {
    switch(mode) {
        case MODE_0:
            for (size_t i = 0; i < max_iterations; i++) {
                byte rx;
                rx = send_byte_mode0(dummy_value);
                if (rx == target_value) {
                    return true;
                }
            }
            break;
        case MODE_1:
            for (size_t i = 0; i < max_iterations; i++) {
                byte rx;
                rx = send_byte_mode1(dummy_value);
                if (rx == target_value) {
                    return true;
                }
            }
            break;
        case MODE_2:
            for (size_t i = 0; i < max_iterations; i++) {
                byte rx;
                rx = send_byte_mode2(dummy_value);
                if (rx == target_value) {
                    return true;
                }
            }
            break;
        case MODE_3:
            for (size_t i = 0; i < max_iterations; i++) {
                byte rx;
                rx = send_byte_mode3(dummy_value);
                if (rx == target_value) {
                    return true;
                }
            }
            break;
    }
    return false; // Timed out
}

byte SPI_transfer_byte(byte data, SPI_MODE mode) {
    switch (mode) {
    case MODE_0:
        return send_byte_mode0(data);
        break;
    case MODE_1:
        return send_byte_mode1(data);
        break;
    case MODE_2:
        return send_byte_mode2(data);
        break;
    case MODE_3:
        return send_byte_mode3(data);
    default:
        printf("ERROR: wrong SPI mode\n");
        return 0x0;
    }
}