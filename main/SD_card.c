#include "SD_card.h"
#include <esp_rom_sys.h> // for timing
#include <driver/spi_master.h>
#include <string.h> // memset

/*

Uses VSPI pins

ASSUMES SD version 2.0 or higher

Communication with the SD card is performed by sending commands to it and receiving responses from it.
A valid SD card command consists of 48 bits. The leftmost two bits are the start bits
which we set to (01). They are followed by a 6-bit command number and a 32-bit argument where
additional information may be provided. Next, there are 7 bits containing a Cyclic Redundancy Check
(CRC) code, followed by a single stop bit (1)

*/

// SD protocol specifies MOSI is high when not transmitting 
#define SD_MOSI_IDLE_BITS 0xFF
#define SD_RESPONSE_TIMEOUT 8 // max number of bytes to wait for the SD card response

// masks for R1 response error bits
#define R1_RESPONSE_IDLE_ERROR              1U
#define R1_RESPONSE_ERASE_RESET_ERROR       1U << 1
#define R1_RESPONSE_ILLEGAL_COMMAND_ERROR   1U << 2
#define R1_RESPONSE_COMMAND_CRC_ERROR       1U << 3
#define R1_RESPONSE_ERASE_SEQUENCE_ERROR    1U << 4
#define R1_RESPONSE_ADDRESS_ERROR           1U << 5
#define R1_RESPONSE_PARAMETER_ERROR         1U << 6

#define SPI_MAXIMUM_BUFFER_SIZE 512 + 2 // maximum transfer size in bytes

// Send a 6 byte command to the SD card
static byte SD_send_command_r1(byte cmd, const byte *args, bool done);
static uint16_t SD_send_command_r2(byte cmd, const byte *args, bool done);
static bool SD_send_command_r3(byte cmd, const byte *args, byte response[5], bool done);
static bool SD_send_command_r7(byte cmd, const byte *args, byte response[5], bool done);
static void build_sd_command(byte cmd, const byte *args, byte *out_cmd);
static inline void deassert_cs(void);

static inline byte SD_send_byte(byte data);
static inline byte SD_send_idle_byte(void);
static bool verify_voltage_and_version(gpio_num_t SD_card_chip_select);
static uint32_t get_CSD_slice(const uint8_t* csd, byte upper_index, byte lower_index);

typedef enum {
    SDSC_TYPE, // SDSC (standard capacity)
    SDHC_SDXC_TYPE, //SDHC/ SDXC (High capacity or very high capacity)
    UNKNOWN_TYPE = -1
} SD_CARD_TYPE;

// data response tokens for write commands (CMD 24 and 25)
typedef enum {
    DATA_ACCEPTED = 0b010,
    DATA_REJECTED_DUE_TO_CRC_ERROR = 0b101,
    DATA_REJECTED_DUE_TO_WRITE_ERROR = 0b110
} DATA_RESPONSE_TOKEN_TYPE;

static SD_CARD_TYPE SD_card_type_global = UNKNOWN_TYPE;
static gpio_num_t SD_CS_global = GPIO_NUM_NC; // chip select for SD card
static uint32_t SD_number_of_blocks_global = 0; // number of 512 byte blocks
static bool had_init = false;

static spi_device_handle_t SD_handle = NULL;
// there are 3 SPI controllers numbered from 1-3 but only 3 and 4 are usable
static spi_host_device_t host_device = SPI3_HOST; // pick SPI3

// global DMA tx and rx buffers
static byte* SD_DMA_buffer_tx = NULL;
static byte* SD_DMA_buffer_tx_dummy = NULL; // used to send MOSI_IDLE_BITS (0xFF)
static byte* SD_DMA_buffer_rx = NULL;

bool SD_card_init(gpio_num_t SD_card_chip_select) {
    if (SD_card_chip_select == 0) {
        printf("GPIO pin must not be pin 0\n");
        return false;
    }
    
    if (had_init) {
        return true;
    }
    
    /*
    IMPORTANT: USE Direct memory access (DMA) or else we can only send up to 64 btyes of data per transfer
    */
    spi_dma_chan_t dma_channel = SPI_DMA_CH_AUTO;

    spi_bus_config_t bus_config = {
        .isr_cpu_id = ESP_INTR_CPU_AFFINITY_AUTO, // defaults to core 0
        .miso_io_num = SPI_MISO,
        .mosi_io_num = SPI_MOSI,
        .sclk_io_num = SPI_CLK,
        // .data_io_default_level = 1, // not supported by bus init function
        .max_transfer_sz = SPI_MAXIMUM_BUFFER_SIZE
    };

    // setup the SPI bus
    ESP_ERROR_CHECK(spi_bus_initialize(host_device, &bus_config, dma_channel));

    spi_device_interface_config_t SD_device_interface_config = {
        // SPI clock rate should be 100-400 KHz for initialization
        .clock_speed_hz = 350 * 1000,
        .clock_source = SPI_CLK_SRC_APB, // APB by default (80 Mhz)
        .duty_cycle_pos = 128, // duty cycle of positive clock in increments of 256 (50% duty cycle)
        .spics_io_num = (int)SD_card_chip_select,
        .queue_size = 1,
        .mode = 0 // can also use 3 but 0 is simplest
    };

    // add the SD card device
    ESP_ERROR_CHECK(spi_bus_add_device(host_device, &SD_device_interface_config, &SD_handle));

    // after power reaches > 2.2 V, wait at least 1 ms.
    esp_rom_delay_us(1000); // likely not needed but cheap to do
    SD_CS_global = SD_card_chip_select;

    ESP_ERROR_CHECK(spi_device_acquire_bus(SD_handle, portMAX_DELAY));

    int real_frequency;
    spi_device_get_actual_freq(SD_handle, &real_frequency);
    printf("SPI frequency set to %d\n", real_frequency * 1000);
    
    // fill with 0xFF
    byte empty[10] = {SD_MOSI_IDLE_BITS};
    for (int i = 0; i < 10; i++) {
        empty[i] = SD_MOSI_IDLE_BITS;
    }
    spi_transaction_t transaction_data = {
        .flags = (SPI_TRANS_CS_KEEP_ACTIVE),
        .length = 80, // 80 bits
        .tx_buffer = empty,
        .rx_buffer = NULL // no need to check the output here
    };

    // send at least 74 clock pulses (we do 80)
    ESP_ERROR_CHECK(spi_device_transmit(SD_handle, &transaction_data));

    // we should be in SPI mode now
    // send CMD0 (reset) command
    byte response = SD_send_command_r1(0, NULL, true);
    
    // we expect to be put in the idle state
    if (response != R1_RESPONSE_IDLE_ERROR) {
        printf("CMD0 failed!\n");
        goto return_with_failure;
    }
    // CMD8
    if (!verify_voltage_and_version(SD_card_chip_select)) {
        return false;
    }
    // Send CMD55 and ACMD41
    byte args[4] = {0x40, 0, 0, 0};  // HCS = 1 (for SDHC/SDXC support)
    int i = 0;
    do {
        // CMD55 to indicate next CMD is application specific
        response = SD_send_command_r1(55, NULL, true);
        if (response != 0x01) {
            printf("CMD55 failed with response %x!\n", response);
            goto return_with_failure;
        }

        response = SD_send_command_r1(41, (const byte *)args, true);
        if (i++ > 10) {
            printf("Failed too many attempts at init sequence\n");
            goto return_with_failure;
        }

    } while (response == 0x01);  // keep polling until card leaves idle
    
    // increase SPI clock speed. SD cards can handle up to 50 MHz
    spi_device_release_bus(SD_handle);
    ESP_ERROR_CHECK(spi_bus_remove_device(SD_handle));
    // 26 Mhz is max acceptable speed but causes the wrong R1 response
    SD_device_interface_config.clock_speed_hz = 20 * 1000 * 1000;
    SD_handle = NULL;
    ESP_ERROR_CHECK(spi_bus_add_device(host_device, &SD_device_interface_config, &SD_handle));
    ESP_ERROR_CHECK(spi_device_acquire_bus(SD_handle, portMAX_DELAY));

    spi_device_get_actual_freq(SD_handle, &real_frequency);
    printf("SPI frequency set to %.2f Mhz\n", real_frequency / 1000.0f);

    deassert_cs();

    // send >= 80 clocks using 0xFF
    spi_transaction_t t = {
        .flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_CS_KEEP_ACTIVE,
        .length = 8,            // bits per transfer (8 bits -> 1 byte -> 8 clocks)
        .tx_data = {0xFF}       // txdata is used because flags USE_TXDATA
    };
    // clock 80 dummy bits
    for(int i = 0; i < 10; i++) {
        ESP_ERROR_CHECK(spi_device_transmit(SD_handle, &t));
    }
    // determine if SDSC (byte addressing) or SDXC by reading OCR with CMD58
    uint8_t r3[5];
    if (!SD_send_command_r3(58, NULL, r3, true) || r3[0] != 0x0) {
        printf("CMD 58 fail: R1=%02x, OCR=%02x%02x%02x%02x\n", r3[0], r3[1], r3[2], r3[3], r3[4]);
        return false;
    }
    if (r3[1] == 0x80) {
        // SDSC
        printf("SDSC with byte addressing\n");
        SD_card_type_global = SDSC_TYPE;
    } else if (r3[1] == 0xC0) {
        // SDXC/SDHC
        printf("SDXC / SDHC with block addressing\n");
        SD_card_type_global = SDHC_SDXC_TYPE;
    } else {
        printf("Error with CMD58 command. Response:\n");
        for (int i = 0; i < 5; i++) {
            printf("Byte %d: %02x\n", i, r3[i]);
        }
        goto return_with_failure;
    }
    if (SD_card_type_global == SDSC_TYPE) {
        // make sure SET_BLOCKLEN is 512 using CMD16
        ;
    }
    // SDXC/SDHC cards always use 512 byte blocks
    
    // release bus because the next function will acquire it
    spi_device_release_bus(SD_handle);

    SD_number_of_blocks_global = SD_get_number_of_512_byte_blocks();
    had_init = true;
    // we need DMA capable memory from the heap
    SD_DMA_buffer_tx = heap_caps_malloc(SPI_MAXIMUM_BUFFER_SIZE, MALLOC_CAP_DMA);
    SD_DMA_buffer_tx_dummy = heap_caps_malloc(SPI_MAXIMUM_BUFFER_SIZE, MALLOC_CAP_DMA);
    SD_DMA_buffer_rx = heap_caps_malloc(SPI_MAXIMUM_BUFFER_SIZE, MALLOC_CAP_DMA);

    if (!SD_DMA_buffer_tx || !SD_DMA_buffer_rx || !SD_DMA_buffer_tx_dummy) {
        printf("failed to malloc one of the 514 byte DMA blocks");
        goto return_with_failure;
    }
    // Set CRCs to dummy value (0xFF)
    SD_DMA_buffer_tx[512] = SD_MOSI_IDLE_BITS;
    SD_DMA_buffer_tx[513] = SD_MOSI_IDLE_BITS;
    // Set tx_dummy buffer to be all idle bits
    memset(SD_DMA_buffer_tx_dummy, SD_MOSI_IDLE_BITS, SPI_MAXIMUM_BUFFER_SIZE);
    return true;

    return_with_failure:
    spi_device_release_bus(SD_handle);
    spi_bus_remove_device(SD_handle);
    spi_bus_free(host_device);
    return false;
}

// Build a 6-byte SD command: [0x40|cmd][arg0][arg1][arg2][arg3][crc]
static void build_sd_command(byte cmd, const byte *args, byte *out_cmd) {
    out_cmd[0] = 0x40 | (cmd & 0x3F);   // Command index with start+transmission bits
    out_cmd[1] = args ? args[0] : 0x00;
    out_cmd[2] = args ? args[1] : 0x00;
    out_cmd[3] = args ? args[2] : 0x00;
    out_cmd[4] = args ? args[3] : 0x00;

    // Only CMD0 and CMD8 require a valid CRC in SPI mode during init
    if (cmd == 0) {
        out_cmd[5] = 0x95;  // Precomputed CRC for CMD0 + argument 0x00000000
    } else if (cmd == 8) { 
        out_cmd[3] = 0x1;
        out_cmd[4] = 0xAA;
        out_cmd[5] = 0x87;  // Precomputed CRC for CMD8 + argument 0x000001AA
    } else {
        out_cmd[5] = 0xFF;  // CRC is ignored after init if SPI mode enabled
    }
}

/*
* sends one byte of SD_MOSI_IDLE_BITS and gets the response
* useful for polling to await a certain value
* NOTE: keeps CS active after sending
*/
static inline byte SD_send_idle_byte(void) {
    return SD_send_byte(SD_MOSI_IDLE_BITS);
}

/*
* sends one byte and gets the response
* useful for polling to await a certain value
* NOTE: keeps CS active after sending
*/
static inline byte SD_send_byte(byte data) {
    spi_transaction_t transaction_data = {
        .flags = (SPI_TRANS_CS_KEEP_ACTIVE | SPI_TRANS_USE_RXDATA | SPI_TRANS_USE_TXDATA),
        .length = 8,
        .tx_data[0] = data
    };
    ESP_ERROR_CHECK(spi_device_transmit(SD_handle, &transaction_data));
    return transaction_data.rx_data[0];
}

static inline void deassert_cs(void) {
    spi_transaction_t t = {
        .flags = 0,          // no KEEP_ACTIVE
        .length = 0,         // no bits transferred
    };
    spi_device_transmit(SD_handle, &t); // this will release CS
}

/**
 * Sends an SD card command and waits up to SD_RESPONSE_TIMEOUT bytes for an R1 response.
 * @param cmd         Command index (0–63).
 * @param args        Pointer to 4-byte argument array (or NULL for zeros).
 * @param done        keeps cs active if false, else disables cs when finished
 * @return            First byte with the MSB = 0, or 0xFF if timeout.
 */
static byte SD_send_command_r1(byte cmd, const byte *args, bool done) {
    byte cmd_to_send[6];
    build_sd_command(cmd, args, cmd_to_send);


    spi_transaction_t transaction_data = {
        .flags = (SPI_TRANS_CS_KEEP_ACTIVE | SPI_TRANS_USE_RXDATA),
        .length = 48, // 6 byte cmd
        .tx_buffer = cmd_to_send,
    };

    ESP_ERROR_CHECK(spi_device_transmit(SD_handle, &transaction_data));

    // Poll until we get a valid R1 (MSB = 0)
    byte r1 = 0xFF;
    int attempts = 0;

    // clock one byte of 0xFF at a time to get R1 response
    do {
        r1 = SD_send_idle_byte();
        if (++attempts > SD_RESPONSE_TIMEOUT) {
            printf("Timeout waiting for R1b response\n");
            deassert_cs();
            return 0xFF;
        }
        // wait for MSB = 0
    } while (r1 & 0x80);
    if (done) {
        deassert_cs();
    }
    return r1;
}

// similar to R1 response but receives two bytes instead of just one
static uint16_t SD_send_command_r2(byte cmd, const byte *args, bool done) {
    byte cmd_to_send[6];
    build_sd_command(cmd, args, cmd_to_send);

    spi_transaction_t transaction_data = {
        .flags = (SPI_TRANS_CS_KEEP_ACTIVE),
        .length = 48, // 6 byte cmd
        .tx_buffer = cmd_to_send,
        .rx_buffer = NULL
    };

    ESP_ERROR_CHECK(spi_device_transmit(SD_handle, &transaction_data));

    // Poll until we get a valid R1 (MSB = 0)
    byte r1 = 0xFF;
    int attempts = 0;
    do {
        // do SPI transfer of one byte
        r1 = SD_send_idle_byte();
        if (++attempts == SD_RESPONSE_TIMEOUT) {
            printf("R2 command did not find an r1 response in time\n");
            deassert_cs();
            return 0xFFFF;
        }
    } while (r1 & 0x80);
    // read one more byte after the R1
    uint16_t response =  (uint16_t)(r1 << 4) & SD_send_idle_byte();
    if (done) {
        deassert_cs();
    }
    return response;
}

static byte SD_send_command_r1b(byte cmd, const byte *args) {
    byte cmd_to_send[6];
    build_sd_command(cmd, args, cmd_to_send);

    spi_transaction_t transaction_data = {
        .flags = (SPI_TRANS_CS_KEEP_ACTIVE),
        .length = 48, // 6 byte cmd
        .tx_buffer = cmd_to_send,
        .rx_buffer = NULL
    };

    ESP_ERROR_CHECK(spi_device_transmit(SD_handle, &transaction_data));

    // Special case: CMD12 has a stuff byte after the command
    if (cmd == 12) {
        SD_send_idle_byte();
    }

    // Poll until we get a valid R1 (MSB cleared)
    byte r1 = 0xFF;
    int attempts = 0;
    do {
        // do SPI transfer of one byte
        r1 = SD_send_idle_byte();
        if (++attempts > 8) {
            printf("Timeout waiting for R1b response\n");
            deassert_cs();
            return 0xFF;
        }
    } while (r1 & 0x80);

    if (r1 != 0x00) {
        printf("R1b response was not 0x00 (got %02X)\n", r1);
        return 0xFF;
    }

    // Wait for busy period (MISO held low)
    while (SD_send_idle_byte() == 0x0);
    deassert_cs();
    return r1;
}

/*
reads 5 bytes (40 bits) of the response (R1 + 4 bytes OCR)
Used only for command 58
*/
static bool SD_send_command_r3(byte cmd, const byte *args, byte response[5], bool done) {

    byte tx[6 + SD_RESPONSE_TIMEOUT + 5];   // cmd + polling + payload
    byte rx[sizeof(tx)];

    build_sd_command(cmd, args, tx);

    // Fill trailing dummy bytes
    for (int i = 6; i < sizeof(tx); i++) {
        tx[i] = SD_MOSI_IDLE_BITS;
    }
    spi_transaction_t transaction_data = {
        .flags = (SPI_TRANS_CS_KEEP_ACTIVE),
        .length = 8 * sizeof(tx),
        .tx_buffer = tx,
        .rx_buffer = rx
    };

    spi_device_transmit(SD_handle, &transaction_data);
    if (done) {
        deassert_cs();
    }

    // Find R1 (first non-0xFF after cmd echo)
    int start = 6;
    while (start < sizeof(rx) - 5 && rx[start] & 0x80) {
        start++;
    }
    if (start >= sizeof(rx) - 5) {
        return false; // no response
    }

    // Copy R1 + 4 payload bytes
    for (int i = 0; i < 5; i++) {
        response[i] = rx[start + i];
    }
    return true;
}

/*
Used only for command 8
*/
static bool SD_send_command_r7(byte cmd, const byte *args, byte response[5], bool done) {
    // R7 and R3 have the same length just different meanings
    return SD_send_command_r3(cmd, args, response, done);
}

/*
check the voltage and SD version (should be 2.0+) by sending CMD8
assumes SPI is set up correctly (100-400 KHz)
*/
static bool verify_voltage_and_version(gpio_num_t SD_card_chip_select) {
    byte response[5];
    if (!SD_send_command_r7(8, NULL, response, true)) return false;
    byte expected_response[5] = {0x1, 0, 0, 01, 0xAA};
    for (int i = 0; i < 5; i++) {
        if (response[i] != expected_response[i]) {
            printf("Response for CMD8 does not match expected value.\n");
            return false;
        }
    }
    return true;
}

// reads a block of size 512 bytes. Assumes the address given is valid
bool SD_read_block(uint32_t block_num, byte *block_data) {
    if (block_num >= SD_number_of_blocks_global) {
        printf("Requested block outside of allowed range. Cannot read\n");
        return false;
    }
    uint32_t addr = (SD_card_type_global == SDHC_SDXC_TYPE)
                    ? block_num
                    : block_num * 512;

    byte args[4] = {
        (addr >> 24) & 0xFF,
        (addr >> 16) & 0xFF,
        (addr >> 8) & 0xFF,
        addr & 0xFF
    };

    if (!SD_DMA_buffer_tx_dummy || !SD_DMA_buffer_rx) {
        printf("DMA buffers tx_dummy and rx not malloc'd in SD_read_block()!\n");
        return false;
    }

    ESP_ERROR_CHECK(spi_device_acquire_bus(SD_handle, portMAX_DELAY));
    byte response = SD_send_command_r1(17, args, false);
    if (response != 0x0) {
        printf("expected response 0x0, got %x\n", response);
        return_with_failure:
        deassert_cs();
        spi_device_release_bus(SD_handle);
        return false;
    }

    // Wait for data token 0xFE
    int attempts = 0;
    byte token;
    do {
        token = SD_send_idle_byte();
        if (++attempts > 10000) {
            printf("Timeout waiting for data token\n");
            goto return_with_failure;
        }
    } while (token != 0xFE);
    // printf("Found token response after %d attempts\n", attempts);

    // Read 514 bytes
    spi_transaction_t transaction_data = {
        .flags = (SPI_TRANS_CS_KEEP_ACTIVE),
        .length = 4096 + 16, // include dummy CRCs
        .tx_buffer = SD_DMA_buffer_tx_dummy,
        .rx_buffer = SD_DMA_buffer_rx
    };
    if (esp_ptr_dma_capable(block_data)) {
        transaction_data.length = 4096;
        transaction_data.rx_buffer = block_data;
        ESP_ERROR_CHECK(spi_device_transmit(SD_handle, &transaction_data));
        SD_send_idle_byte();
        SD_send_idle_byte();
    } else {
        ESP_ERROR_CHECK(spi_device_transmit(SD_handle, &transaction_data));
        memcpy(block_data, SD_DMA_buffer_rx, 512);
    }

    if (SD_send_idle_byte() != 0xFF) {
        goto return_with_failure;
    }

    deassert_cs();
    spi_device_release_bus(SD_handle);
    return true;
}

/*
Reads num_blocks blocks of 512 bytes each including the starting block. Stores results in block_data
Strongly reccommended to not read more than ~32 blocks at a time due to stack limitations
*/
bool SD_read_many_blocks(uint32_t starting_block_num, byte *block_data, size_t num_blocks) {
    if (num_blocks == 0) {
        printf("Must read at least one block of memory\n");
        return false;
    }
    // make sure we don't read past the largest block index
    if (starting_block_num + num_blocks > SD_number_of_blocks_global) {
        printf("Requested too many blocks. Cannot read\n");
        return false;
    }
    
    uint32_t addr = (SD_card_type_global == SDHC_SDXC_TYPE)
                        ? starting_block_num
                        : starting_block_num * 512;
    byte args[4] = {
    (addr >> 24) & 0xFF,
    (addr >> 16) & 0xFF,
    (addr >> 8) & 0xFF,
    addr & 0xFF
    };

    if (!SD_DMA_buffer_tx_dummy || !SD_DMA_buffer_rx) {
        printf("DMA buffers tx_dummy and rx not malloc'd in SD_read_many_blocks()!\n");
        return false;
    }

    ESP_ERROR_CHECK(spi_device_acquire_bus(SD_handle, portMAX_DELAY));
    // CMD18 to read multiple blocks
    byte response = SD_send_command_r1(18, args, false);
    if (response != 0x0) {
        printf("CMD18 did not return the correct R1 response. Expected 0x0, got %x\n", response);
        return_with_failure:
        deassert_cs();
        spi_device_release_bus(SD_handle);
        return false;
    }

    int attempts;
    // Read each block
    for (size_t i = 0; i < num_blocks; i++) {
        // Wait for 0xFE token for this block
        attempts = 0;
        byte token;
        do {
            token = SD_send_idle_byte();
            // if (token != 0xFF) {printf("TOKEN: %x (block number %d)\n", token, (int)i);}
            if (++attempts > (int)1e5) {
                printf("Timeout waiting for data token\n");
                goto return_with_failure;
            }
        } while (token != 0xFE);

        // Read 514 bytes (discared the last 2 CRC bytes)
        spi_transaction_t transaction_data = {
            .flags = (SPI_TRANS_CS_KEEP_ACTIVE),
            .length = 4096 + 16, // include dummy CRCs
            .tx_buffer = SD_DMA_buffer_tx_dummy,
            .rx_buffer = NULL // set in next step
        };
        if (esp_ptr_dma_capable(block_data)) {
            transaction_data.length = 4096;
            transaction_data.rx_buffer = &(block_data[512 * i]);
            ESP_ERROR_CHECK(spi_device_transmit(SD_handle, &transaction_data));
            SD_send_idle_byte();
            SD_send_idle_byte();
        } else {
            transaction_data.rx_buffer = SD_DMA_buffer_rx;
            ESP_ERROR_CHECK(spi_device_transmit(SD_handle, &transaction_data));
            memcpy(&(block_data[512 * i]), SD_DMA_buffer_rx, 512);
        }
    }
    // stop transmission with CMD12. Block until card is ready
    if (SD_send_command_r1b(12, NULL) != 0x0) {
        printf("CMD12 did not return the correct R1 response\n");
        goto return_with_failure;
    }
    deassert_cs();
    spi_device_release_bus(SD_handle);
    return true;
}

/**
 * @brief Read the Card-Specific Data (CSD) register (16 bytes).
 *
 * Issues CMD9 and waits for a data token before reading.
 *
 * @param csd Pointer to a 16-byte buffer to store the CSD.
 * @return true if the read succeeds, false otherwise.
 */
bool SD_read_CSD(byte *csd) {
    if (!csd) {
        return false;
    }
    ESP_ERROR_CHECK(spi_device_acquire_bus(SD_handle, portMAX_DELAY));
    if (SD_send_command_r1(9, NULL, false) != 0x0) {
        printf("R1 is incorrect\n");
        return_with_failure:
        deassert_cs();
        spi_device_release_bus(SD_handle);
        return false;
    }
    // wait for data token 0xFE
    int count = 0;
    byte token = 0xFF;
    do {
        token = SD_send_idle_byte();
        if (token == 0xFE) break;
        if (++count > (int)1e5) {
            printf("Timeout waiting for FE token\n");
            goto return_with_failure;
        }
    } while (1);

    for (int i = 0; i < 16; i++) {
        csd[i] = SD_send_idle_byte();
    }

    // 2 CRC bytes
    SD_send_idle_byte();
    SD_send_idle_byte();

    deassert_cs();
    spi_device_release_bus(SD_handle);
    return true;
}

/**
 * @brief Compute the number of 512-byte blocks from the CSD register.
 *
 * Interprets fields differently depending on whether the card is SDSC or SDHC/SDXC.
 *
 * @param csd The 16-byte CSD register.
 * @return The total number of 512-byte blocks.
 */
uint32_t SD_get_block_count(byte *csd) {
    if (!csd) {
        printf("passed NULL pointer to SD_get_block_count()\n");
        return 0x0;
    }
    byte C_SIZE_MULT, READ_BL_LEN;
    uint16_t C_SIZE;
    uint32_t block_length;

    if (SD_card_type_global == SDSC_TYPE) {
        // untested code. Consult the SD physical layer specs
        C_SIZE = (uint16_t)get_CSD_slice(csd, 73, 62);
        C_SIZE_MULT = (byte)get_CSD_slice(csd, 49, 47);
        READ_BL_LEN = (byte)get_CSD_slice(csd, 83, 80);

        block_length = 1;
        while (READ_BL_LEN-- > 0) {
            block_length *= 2;
        }

        uint32_t mult = 1;
        while ((C_SIZE_MULT--) > 0) {
            mult *= 2;
        }
        mult *= 4;

        printf("THIS SD CARD HAS A BLOCK LENGTH OF %ld\n", block_length);
        printf("MULT: %ld\n", mult);
        

        uint32_t number_of_blocks = (C_SIZE + 1) * mult;
        return (number_of_blocks * block_length) / 512; // total size / 512
    } else {
        C_SIZE = get_CSD_slice(csd, 69, 48);
        return (C_SIZE + 1) * 1000; // multiply by 512 for total capacity in bytes
    }
}

// extract a slice of up to 4 bytes from the 16 byte CSD using bit indexes (127 = max, 0 = min)
static uint32_t get_CSD_slice(const uint8_t* csd, byte upper_index, byte lower_index) {
    if (upper_index < lower_index || (upper_index - lower_index + 1) > 32) {
        printf("invalid CSD slice\n");
        return 0;
    }

    uint32_t result = 0;
    for (byte i = 0; i <= (upper_index - lower_index); i++) {
        byte bit_pos = lower_index + i;
        byte byte_index = 15 - (bit_pos / 8);  // MSB first
        byte bit_in_byte = bit_pos % 8;
        byte bit = (csd[byte_index] >> bit_in_byte) & 1;
        result |= (bit << i);  // LSB of result = lower_index bit
    }
    return result;
}

uint32_t SD_get_number_of_512_byte_blocks(void) {
    byte csd[16];
    if (!SD_read_CSD(csd))  {
        printf("Could not read CSD\n");
        return 0x0;
    }
    return SD_get_block_count(csd);
    // printf("Total 512-byte blocks: %.4e\n", (double)blocks);
}

bool SD_is_block_empty(uint32_t block_num) {
    byte block_data[512];
    if (!SD_read_block(block_num, block_data)) return false;
    for (int i = 0; i < 512; i++) {
        if (block_data[i] != 0x0) {
            return false;
        }
    }
    return true;
}

// writes 512 bytes to a single block at the address block_num
bool SD_write_block(uint32_t block_num, const byte *data_to_write) {
    if (block_num >= SD_number_of_blocks_global) {
        printf("Requested too many blocks. Cannot read\n");
        return false;
    }

    uint32_t addr = (SD_card_type_global == SDHC_SDXC_TYPE)
                        ? block_num
                        : block_num * 512;
    byte args[4] = {
    (addr >> 24) & 0xFF,
    (addr >> 16) & 0xFF,
    (addr >> 8) & 0xFF,
    addr & 0xFF
    };

    // we need DMA capable memory from the heap
    if (!SD_DMA_buffer_tx) {
        printf("DMA buffer tx not malloc'd in SD_write_block()!\n");
        return false;
    }

    // CMD24 to write one block of BLOCK_LEN (assumes length is 512)
    ESP_ERROR_CHECK(spi_device_acquire_bus(SD_handle, portMAX_DELAY));
    if (SD_send_command_r1(24, args, false) != 0x0) {
        printf("R1 response for SD_write_block() did not match expected value\n");
        return_with_failure:
        deassert_cs();
        spi_device_release_bus(SD_handle);
        return false;
    }
    // Send start token
    SD_send_byte(0xFE);
    // Send data
    spi_transaction_t transaction_data = {
        .flags = (SPI_TRANS_CS_KEEP_ACTIVE),
        .length = 4096 + 16,
        .tx_buffer = SD_DMA_buffer_tx, // set in next step
        .rx_buffer = NULL
    };
    if (esp_ptr_dma_capable(data_to_write)) {
        transaction_data.length = 4096;
        transaction_data.tx_buffer = data_to_write;
        ESP_ERROR_CHECK(spi_device_transmit(SD_handle, &transaction_data));
        // dummy CRCs
        SD_send_idle_byte();
        SD_send_idle_byte();
    } else {
        memcpy(SD_DMA_buffer_tx, data_to_write, 512);
        ESP_ERROR_CHECK(spi_device_transmit(SD_handle, &transaction_data));
    }

    // read the data response token from the card
    byte response;
    int attempts = 0;
    do {
        response = SD_send_idle_byte();
        attempts++;
        // bit 4 must be 0 and bit 0 must be 1 to indicate a data response
    } while ((response & 0x11) != 0x01 && attempts < SD_RESPONSE_TIMEOUT);
    if (attempts == SD_RESPONSE_TIMEOUT) {
        printf("Timeout waiting for response token in SD_write_block()\n");
        goto return_with_failure;
    }
    byte status = (response >> 1) & 0x07; // extract bits [3:1]
    if (status != DATA_ACCEPTED) {
        if (status == DATA_REJECTED_DUE_TO_CRC_ERROR) {
            printf("Data rejected due to CRC error\n");
        }
        else {
            // DATA_REJECTED_DUE_TO_WRITE_ERROR
            printf("Data rejected due to write error\n");
        }
        // can use CMD13 to see what went wrong here
        uint16_t status_reg = SD_send_command_r2(13, NULL, true);
        printf("Status register: %x\n", status_reg);
        goto return_with_failure;
    }

    // wait for busy period to end before we can transmit again
    attempts = 0;
    while (SD_send_idle_byte() != 0xFF && (++attempts < (int)1e5));
    if (attempts == (int)1e5) {
        printf("timed out waiting for busy period to end in SD_write_block() function\n");
        goto return_with_failure;
    }
    deassert_cs();
    spi_device_release_bus(SD_handle);
    return true;
}

bool SD_write_many_blocks(uint32_t starting_block_num, const byte *data_to_write, size_t num_blocks) {
    if (num_blocks == 0) {
        printf("Must write at least one block of memory\n");
        return false;
    }
    if (starting_block_num + num_blocks > SD_number_of_blocks_global) {
        printf("Requested too many blocks. Cannot write\n");
        return false;
    }

    uint32_t addr = (SD_card_type_global == SDHC_SDXC_TYPE)
                        ? starting_block_num
                        : starting_block_num * 512;
    byte args[4] = {
    (addr >> 24) & 0xFF,
    (addr >> 16) & 0xFF,
    (addr >> 8) & 0xFF,
    addr & 0xFF
    };

// we need DMA capable memory from the heap
    if (!SD_DMA_buffer_tx) {
        printf("DMA buffer tx not malloc'd in SD_write_many_blocks()!\n");
        return false;
    }

    // CMD25 to write multiple block of BLOCK_LEN (assumes length is 512)
    ESP_ERROR_CHECK(spi_device_acquire_bus(SD_handle, portMAX_DELAY));
    if (SD_send_command_r1(25, args, false) != 0x0) {
        printf("R1 response for SD_write_many_blocks() did not match expected value\n");
        goto return_with_failure;
    }
    unsigned int attempts = 0;
    for (int i = 0; i < num_blocks; i++) {
        // Start token (not the same as single block write or reads!)
        SD_send_byte(0xFC);
        // Send data in blocks of 514
        spi_transaction_t transaction_data = {
            .flags = (SPI_TRANS_CS_KEEP_ACTIVE),
            .length = 4096 + 16,
            .tx_buffer = SD_DMA_buffer_tx, // check in next step
            .rx_buffer = NULL
        };
        if (esp_ptr_dma_capable(data_to_write)) {
            transaction_data.length = 4096;
            transaction_data.tx_buffer = &(data_to_write[512 * i]);
            ESP_ERROR_CHECK(spi_device_transmit(SD_handle, &transaction_data));
            // dummy CRCs
            SD_send_idle_byte();
            SD_send_idle_byte();
        } else {
            memcpy(SD_DMA_buffer_tx, &(data_to_write[512 * i]), 512);
            ESP_ERROR_CHECK(spi_device_transmit(SD_handle, &transaction_data));
        }

        // read the data response token from the card
        byte response;
        do {
            // the upper 3 bits of the response are don't care
            response = SD_send_idle_byte() & 0x1F;
            attempts++;
            // bit 4 must be 0 and bit 0 must be 1 to indicate a data response
        } while ((response & 0x11) != 0x01 && attempts < SD_RESPONSE_TIMEOUT);
        if (attempts == SD_RESPONSE_TIMEOUT) {
            printf("Timed out waiting for SD card response token in SD_write_many_blocks()\n");
            goto return_with_failure;
        }
        byte status = (response >> 1) & 0x07; // extract bits [3:1]
        if (status != DATA_ACCEPTED) {
            if (status == DATA_REJECTED_DUE_TO_CRC_ERROR) {
                printf("Data rejected due to CRC error\n");
            }
            else {
                // DATA_REJECTED_DUE_TO_WRITE_ERROR
                printf("Data rejected due to write error\n");
            }
            // can use CMD13 to see what went wrong here
            uint16_t status_reg = SD_send_command_r2(13, NULL, true);
            printf("Status register: %x\n", status_reg);
            goto return_with_failure;
        }
        // wait for busy period to end before we can transmit again
        attempts = 0;
        while (SD_send_idle_byte() !=0xFF && (++attempts < (int)1e5));
        if (attempts == (int)1e5) {
            printf("Timed out waiting for the busy condition to end in SD_write_many_blocks()\n");
            goto return_with_failure;
        }
    }
    // send the STOP TRAN token
    SD_send_byte(0xFD);
    // wait for card to end the busy condition again
    attempts = 0;
    while (SD_send_idle_byte() != 0xFF && (++attempts < (int)1e5));
    if (attempts == (int)1e5) {
        printf("Timed out waiting for the busy condition to end in SD_write_many_blocks()\n");
        goto return_with_failure;
    }
    deassert_cs();
    spi_device_release_bus(SD_handle);
    return true;
    return_with_failure:
    deassert_cs();
    spi_device_release_bus(SD_handle);
    return false;
}

bool SD_clear_block(uint32_t block_num) {
    // if block is already empty then return early
    if (SD_is_block_empty(block_num)) return true;

    // else write 512 0s to the block
    byte zeroes[512] = {0}; // set all elements to 0
    return SD_write_block(block_num, zeroes);
}

bool SD_clear_many_blocks(uint32_t starting_block_num, size_t num_blocks) {
    byte *zeroes = calloc(num_blocks, 512); // num_blocks entries of size 512 bytes
    if (!zeroes) {
        printf("calloc failed inside SD_clear_many_blocks() function\n");
        return false;
    }
    if (!SD_write_many_blocks(starting_block_num, zeroes, num_blocks)) {
        printf("SD_write_many_blocks() failed inside SD_clear_many_blocks()\n");
        free(zeroes);
        return false;
    }
    free(zeroes);
    return true;
}

bool SD_deinit(void) {
    // free the DMA buffers if they were alloced in the init (they should be)
    if (SD_DMA_buffer_tx) heap_caps_free(SD_DMA_buffer_tx);
    if (SD_DMA_buffer_tx_dummy) heap_caps_free(SD_DMA_buffer_tx_dummy);
    if (SD_DMA_buffer_rx) heap_caps_free(SD_DMA_buffer_rx);

    spi_device_release_bus(SD_handle);
    if (spi_bus_remove_device(SD_handle) != ESP_OK) return false;
    if (spi_bus_free(host_device) != ESP_OK) return false;
    return true;
}
