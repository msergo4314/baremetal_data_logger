#include "SD_card_SPI.h"
#include "esp_rom_sys.h" // for timing
#include <string.h>
/*

ASSUMES SD version 2.0 or higher

Communication with the SD card is performed by sending commands to it and receiving responses from it.
A valid SD card command consists of 48 bits. The leftmost two bits are the start bits
which we set to (01). They are followed by a 6-bit command number and a 32-bit argument where
additional information may be provided. Next, there are 7 bits containing a Cyclic Redundancy Check
(CRC) code, followed by a single stop bit (1)

*/

// SDHC

// SD protocol specifies MOSI is high when not transmitting 
#define SD_MOSI_IDLE_BITS 0xFF 

// masks for R1 response error bits
#define R1_RESPONSE_IDLE_ERROR              1U
#define R1_RESPONSE_ERASE_RESET_ERROR       1U << 1
#define R1_RESPONSE_ILLEGAL_COMMAND_ERROR   1U << 2
#define R1_RESPONSE_COMMAND_CRC_ERROR       1U << 3
#define R1_RESPONSE_ERASE_SEQUENCE_ERROR    1U << 4
#define R1_RESPONSE_ADDRESS_ERROR           1U << 5
#define R1_RESPONSE_PARAMETER_ERROR         1U << 6


// static byte CMD0[6] = {0x40 + 0, 0x00, 0x00, 0x00, 0x00, 0x95};
// static byte CMD8[6] = {0x40 + 8, 0x00, 0x00, 0x01, 0xAA, 0x87};
// static byte CMD58[6] = {0x40 + 58, 0x00, 0x00, 0x00, 0x00, 0x01};
// static byte CMD55[6] = {0x40 + 55, 0x00, 0x00, 0x00, 0x00, 0x01};
// static byte ACMD41[6] = {0x40 + 41, 0x40, 0x00, 0x00, 0x00, 0x01};
// static byte CMD17[6] = {0x40 + 17, 0x00, 0x00, 0x00, 0x00, 0x01};

// static byte SD_get_response(gpio_num_t SD_card_cs);

// Send a 6 byte command to the SD card
// Returns the first byte of the response

static byte SD_send_command_r1(byte cmd, const byte *args, bool done);
static byte* SD_send_command_r3(byte cmd, const byte *args, bool done);
static byte* SD_send_command_r7(byte cmd, const byte *args, bool done);
static void build_sd_command(byte cmd, const byte *args, byte *out_cmd);
// static byte create_CRC7(const byte* bytes_before_crc);
static void print_response(const byte* response, size_t length);
static void print_r1_response_flags(byte r1);
static bool verify_voltage_and_version(gpio_num_t SD_card_chip_select);

typedef enum {
    BYTE_ADDRESSING,
    BLOCK_ADDRESSING,
    UNKNOWN_ADDRESSING
} ADDRESSING_MODE;

static ADDRESSING_MODE addressing_mode_global = UNKNOWN_ADDRESSING;
static gpio_num_t SD_CS_global = GPIO_NUM_NC; // chip select for SD card

/*
initialize the SPI mode of the SD card
*/
bool SD_card_init(gpio_num_t SD_card_chip_select) {
    // after power reaches > 2.2 V, wait at least 1 ms.
    esp_rom_delay_us(1000); // likely not needed but cheap to do
    SPI_attach_device(SD_card_chip_select, MODE_0);
    SPI_init();    
    SPI_set_mosi(1); // set MOSI high
    SD_CS_global = SD_card_chip_select;
    SPI_cs_high(SD_CS_global);
    // SPI_cs_low(SD_CS_global);

    // SPI clock rate should be 100-400 KHz for initialization
    SPI_set_frequency(250);
    // send at least 74 clock pulses (we do 80)
    SPI_transfer_block(NULL, NULL, 20, 0);

    // we should be in SPI mode now
    // send CMD0 (reset) command
    byte response = SD_send_command_r1(0, NULL, true);
    
    // we expect to be put in the idle state
    printf("Response to CMD0: %x\n", response);
    if (response != R1_RESPONSE_IDLE_ERROR) {
        printf("CMD0 failed!\n");
        return false;
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
        if (response > 0x01) {
            printf("CMD55 failed with response %x!\n", response);
            return false;
        }

        response = SD_send_command_r1(41, (const byte*)args, true);
        printf("Response to ACMD41: %x\n", response);
        if (i++ > 10) {
            printf("Failed too many attempts at init sequence\n");
            return false;
        }

    } while (response == 0x01);  // keep polling until card leaves idle
    SPI_set_frequency(SPI_get_max_frequency() / 1000);
    SPI_set_mosi(1);

    // determine if SDSC (byte addressing) or SDXC by reading OCR
    byte* response_arr = SD_send_command_r3(58, NULL, true);
    if (response_arr == NULL) {
        printf("SD card did not respond\n");
        return false;
    }
    if (response_arr[0] == 0) {
        // SDSC
        printf("SDSC with byte addressing\n");
        addressing_mode_global = BYTE_ADDRESSING;
    } else if (response_arr[0] == 0x40) {
        // SDXC/SDHC
        printf("SDSC / SDHC with block addressing\n");
        addressing_mode_global = BLOCK_ADDRESSING;
    } else {
        free(response_arr);
        printf("Error with CRC58 command\n");
        return false;
    }
    free(response_arr);
    SD_CS_global = SD_card_chip_select;
    SPI_cs_high(SD_CS_global);
    return true;
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

/**
 * Sends an SD card command and waits for an R1 response.
 * @param cs          Chip select GPIO for SD card.
 * @param cmd         Command index (0–63).
 * @param args        Pointer to 4-byte argument array (or NULL for zeros).
 * @return            First non-0xFF response byte, or 0xFF if timeout.
 */
static byte SD_send_command_r1(byte cmd, const byte *args, bool done) {
    SPI_set_mosi(1);
    byte tx[6 + 8];   // command + up to 8 dummy
    byte rx[6 + 8];   // readback buffer

    build_sd_command(cmd, args, tx);

    // Fill trailing dummy bytes to poll response
    for (int i = 6; i < 14; i++) tx[i] = 0xFF;

    SPI_cs_low(SD_CS_global);
    SPI_transfer_block(tx, rx, sizeof(tx), MODE_0);
    if (done) { SPI_cs_high(SD_CS_global); }

    // Skip first 6 (echo of command), look at the next 8 for response
    for (int i = 6; i < 14; i++) {
        if (rx[i] != 0xFF) return rx[i];
    }
    return 0xFF; // timeout
}

// reads 4 bytes (32 bits) of the response. Caller must free returned array
static byte* SD_send_command_r3(byte cmd, const byte *args, bool done) {
    SPI_set_mosi(1);
    byte tx[6 + 8 + 4];   // command + up to 8 dummy + read 4 bytes = 18
    byte rx[6 + 8 + 4];   // readback buffer

    build_sd_command(cmd, args, tx);

    // Fill trailing dummy bytes to poll response
    for (int i = 6; i < 18; i++) tx[i] = 0xFF;

    // Perform one contiguous transfer with CS active
    SPI_cs_low(SD_CS_global);
    SPI_transfer_block(tx, rx, sizeof(tx), MODE_0);
    if (done) { SPI_cs_high(SD_CS_global); }
    int start = 0;
    // Skip first 6 (echo of command), look at the next 8 + 4 for response start
    for (int i = 6; i < 18; i++) {
        if (rx[i] != 0xFF) {
            start = i;
            break;
        }
    }
    if (start == 0) {
        // all 0xFF
        return NULL;
    }
    byte* response = malloc(4);
    if (!response) {
        printf("Malloc call failed\n");
        return NULL;
    }
    for (int i = 0; i < 4; i++) {
        response[i] = rx[start + i];
    }
    return response;
}

// reads 5 bytes (40 bits) of the response. Caller must free returned array
static byte* SD_send_command_r7(byte cmd, const byte *args, bool done) {
    SPI_set_mosi(1);
    byte tx[6 + 8 + 5];   // command + up to 8 dummy + read 5 bytes = 19
    byte rx[6 + 8 + 5];   // readback buffer

    build_sd_command(cmd, args, tx);

    // Fill trailing dummy bytes to poll response
    for (int i = 6; i < 19; i++) tx[i] = 0xFF;

    // Perform one contiguous transfer with CS active
    SPI_cs_low(SD_CS_global);
    SPI_transfer_block(tx, rx, sizeof(tx), MODE_0);
    if (done) { SPI_cs_high(SD_CS_global); }

    int start = 0;
    // Skip first 6 (echo of command), look at the next 8 + 4 for response start
    for (int i = 6; i < 19; i++) {
        if (rx[i] != 0xFF) {
            start = i;
            break;
        }
    }
    if (start == 0) {
        // all 0xFF
        return NULL;
    }
    byte* response = malloc(5);
    if (!response) {
        printf("Malloc call failed\n");
        return NULL;
    }
    for (int i = 0; i < 5; i++) {
        response[i] = rx[start + i];
    }
    return response;
}

static void print_response(const byte* response, size_t length) {
    for (size_t i = 0; i < length; i++) {
        printf("byte %d: %x\n", i, response[i]);
    }
}

static void print_r1_response_flags(byte r1) {
    if (r1 | R1_RESPONSE_IDLE_ERROR) {printf("IDLE\n");}
    if (r1 | R1_RESPONSE_ERASE_RESET_ERROR) {printf("ERASE RESET\n");}
    if (r1 | R1_RESPONSE_ILLEGAL_COMMAND_ERROR) {printf("ILLEGAL COMMAND\n");}
    if (r1 | R1_RESPONSE_COMMAND_CRC_ERROR) {printf("COMMAND CRC ERROR\n");}
    if (r1 | R1_RESPONSE_ERASE_SEQUENCE_ERROR) {printf("ERASE SEQUENCE ERROR\n");}
    if (r1 | R1_RESPONSE_ADDRESS_ERROR) {printf("ADDRESS ERROR\n");}
    if (r1 | R1_RESPONSE_PARAMETER_ERROR) {printf("PARAMETER ERROR\n");}
}

/*
check the voltage and SD version (should be 2.0+) by sending CMD8
assumes SPI is set up correctly (100-400 KHz)
*/
static bool verify_voltage_and_version(gpio_num_t SD_card_chip_select) {
    SPI_set_mosi(1);
    byte* response = SD_send_command_r7(8, NULL, true);
    if (!response) return false;
    byte expected_response[5] = {0x1, 0, 0, 01, 0xAA};
    bool response_matches = false;
    for (int i = 0; i < 13; i++) {
        if (response[i] == expected_response[0] && i < 10) {
            for (int j = 1; j < 5; j++) {
                if (response[i + j] == expected_response[j]) {
                    if (j == 4) {
                        response_matches = true;
                    }
                } else {
                    break;
                }
            }
        }
    }
    if (!response_matches) {
        printf("CMD8 response did not match what was expected\n");
        print_r1_response_flags(response[0]); // print r1 byte
        free(response);
        return false;
    }
    free(response);
    return true;
}

// reads a block of size 512 bytes
bool SD_read_block(uint32_t block_num, byte* block_data) {
    uint32_t addr = (addressing_mode_global == BLOCK_ADDRESSING)
                    ? block_num
                    : block_num * 512;

    byte args[4] = {
        (addr >> 24) & 0xFF,
        (addr >> 16) & 0xFF,
        (addr >> 8) & 0xFF,
        addr & 0xFF
    };

    byte tx[6], rx[6];
    build_sd_command(17, args, tx);

    SPI_cs_low(SD_CS_global);
    SPI_transfer_block(tx, rx, sizeof(tx), MODE_0);

    // Poll R1 response
    byte r1 = 0xFF;
    int attempts = 0;
    do {
        r1 = SPI_transfer_byte(0xFF, MODE_0);
        if (++attempts > 8) {
            SPI_cs_high(SD_CS_global);
            printf("Timeout waiting for R1\n");
            return false;
        }
    } while (r1 & 0x80); // Wait for MSB=0

    // Wait for data token 0xFE
    attempts = 0;
    byte token;
    do {
        token = SPI_transfer_byte(0xFF, MODE_0);
        if (++attempts > 10000) {
            SPI_cs_high(SD_CS_global);
            printf("Timeout waiting for data token\n");
            return false;
        }
    } while (token != 0xFE);

    // Read 512 bytes
    for (int i = 0; i < 512; i++) {
        block_data[i] = SPI_transfer_byte(0xFF, MODE_0);
    }

    // Read CRC (2 bytes)
    SPI_transfer_byte(0xFF, MODE_0);
    SPI_transfer_byte(0xFF, MODE_0);

    SPI_cs_high(SD_CS_global);
    return true;
}

static byte sd_get_response()
{
    byte response = SPI_transfer_byte(0xFF, MODE_0);
    int count = 0;

    while (response == 0xFF && count < 8)
    {
        response = SPI_transfer_byte(0xff, MODE_0);
        count++;
    }

    return response;
}