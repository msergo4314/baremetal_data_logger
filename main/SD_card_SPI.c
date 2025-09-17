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

// Send a 6 byte command to the SD card
static byte SD_send_command_r1(byte cmd, const byte *args, bool done);
static uint16_t SD_send_command_r2(byte cmd, const byte *args, bool done);
static bool SD_send_command_r3(byte cmd, const byte *args, byte response[5], bool done);
static bool SD_send_command_r7(byte cmd, const byte *args, byte response[5], bool done);
static void build_sd_command(byte cmd, const byte *args, byte *out_cmd);
// static byte create_CRC7(const byte* bytes_before_crc);

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

bool SD_card_init(gpio_num_t SD_card_chip_select) {
    // after power reaches > 2.2 V, wait at least 1 ms.
    esp_rom_delay_us(1000); // likely not needed but cheap to do
    SPI_attach_device(SD_card_chip_select, MODE_0);
    if (!SPI_init()) return false;    
    // SPI_set_mosi(1); // set MOSI high when idle (0xFF)
    SD_CS_global = SD_card_chip_select;
    SPI_cs_high(SD_CS_global);

    // SPI clock rate should be 100-400 KHz for initialization
    SPI_set_frequency(300);
    // send at least 74 clock pulses (we do 80)
    for (int i = 0; i < 10; i++) {
        SPI_transfer_byte(SD_MOSI_IDLE_BITS, MODE_0);
    }

    // we should be in SPI mode now
    // send CMD0 (reset) command
    byte response = SD_send_command_r1(0, NULL, true);
    
    // we expect to be put in the idle state
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
        if (response != 0x01) {
            printf("CMD55 failed with response %x!\n", response);
            return false;
        }

        response = SD_send_command_r1(41, (const byte*)args, true);
        // printf("Response to ACMD41: %x\n", response);
        if (i++ > 10) {
            printf("Failed too many attempts at init sequence\n");
            return false;
        }

    } while (response == 0x01);  // keep polling until card leaves idle
    SPI_set_frequency(SPI_get_max_frequency() / 1000); // set to max speed (SD card can go up to 50 MHz)

    // determine if SDSC (byte addressing) or SDXC by reading OCR with CMD58
    uint8_t r3[5];
    if (!SD_send_command_r3(58, NULL, r3, true)) {
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
        printf("Error with CRC58 command\n");
        return false;
    }
    if (SD_card_type_global == SDSC_TYPE) {
        // make sure SET_BLOCKLEN is 512 using CMD16
        ;
    }
    // SDXC/SDHC cards always use 512 byte blocks
    SD_number_of_blocks_global = SD_get_number_of_512_byte_blocks();
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
 * Sends an SD card command and waits up to SD_RESPONSE_TIMEOUT bytes for an R1 response.
 * @param cs          Chip select GPIO for SD card.
 * @param cmd         Command index (0–63).
 * @param args        Pointer to 4-byte argument array (or NULL for zeros).
 * @return            First byte with the MSB = 0, or 0xFF if timeout.
 */
static byte SD_send_command_r1(byte cmd, const byte *args, bool done) {
    byte cmd_to_send[6];
    build_sd_command(cmd, args, cmd_to_send);

    SPI_cs_low(SD_CS_global);
    SPI_transmit_to_slave(cmd_to_send, sizeof(cmd_to_send), MODE_0);

    // Poll until we get a valid R1 (MSB = 0)
    byte r1 = 0xFF;
    int attempts = 0;
    do {
        r1 = SPI_transfer_byte(SD_MOSI_IDLE_BITS, MODE_0);
        if (++attempts > SD_RESPONSE_TIMEOUT) {
            printf("Timeout waiting for R1b response\n");
            SPI_cs_high(SD_CS_global);
            return 0xFF;
        }
    } while (r1 & 0x80);
    if (done) SPI_cs_high(SD_CS_global);
    return r1;
}

// similar to R1 response but receives two bytes instead of just one
static uint16_t SD_send_command_r2(byte cmd, const byte *args, bool done) {
    byte cmd_to_send[6];
    build_sd_command(cmd, args, cmd_to_send);

    SPI_cs_low(SD_CS_global);
    SPI_transmit_to_slave(cmd_to_send, sizeof(cmd_to_send), MODE_0);

    // Poll until we get a valid R1 (MSB = 0)
    byte r1 = 0xFF;
    int attempts = 0;
    do {
        r1 = SPI_transfer_byte(SD_MOSI_IDLE_BITS, MODE_0);
        if (++attempts > SD_RESPONSE_TIMEOUT) {
            printf("Timeout waiting for R1b response\n");
            SPI_cs_high(SD_CS_global);
            return 0xFF;
        }
    } while (r1 & 0x80);
    // read one more byte after the R1
    uint16_t response =  (uint16_t)(r1 << 4) & SPI_transfer_byte(SD_MOSI_IDLE_BITS, MODE_0);
    if (done) SPI_cs_high(SD_CS_global);
    return response;
}

static byte SD_send_command_r1b(byte cmd, const byte* args) {
    byte cmd_to_send[6];
    build_sd_command(cmd, args, cmd_to_send);

    SPI_cs_low(SD_CS_global);
    SPI_transmit_to_slave(cmd_to_send, sizeof(cmd_to_send), MODE_0);

    // Special case: CMD12 has a stuff byte after the command
    if (cmd == 12) {
        SPI_transfer_byte(SD_MOSI_IDLE_BITS, MODE_0); // discard stuff byte
    }

    // Poll until we get a valid R1 (MSB cleared)
    byte r1 = 0xFF;
    int attempts = 0;
    do {
        r1 = SPI_transfer_byte(SD_MOSI_IDLE_BITS, MODE_0);
        if (++attempts > 8) {
            printf("Timeout waiting for R1b response\n");
            SPI_cs_high(SD_CS_global);
            return 0xFF;
        }
    } while (r1 & 0x80);

    if (r1 != 0x00) {
        printf("R1b response was not 0x00 (got %02X)\n", r1);
    }

    // Wait for busy period (MISO held low)
    while (SPI_transfer_byte(SD_MOSI_IDLE_BITS, MODE_0) == 0x00);
    SPI_cs_high(SD_CS_global);
    return r1;
}

/*
reads 5 bytes (40 bits) of the response (R1 + 4 bytes OCR)
Used only for command 58
*/
static bool SD_send_command_r3(byte cmd, const byte *args, byte response[5], bool done) {
    SPI_set_mosi(1);

    byte tx[6 + SD_RESPONSE_TIMEOUT + 5];   // cmd + polling + payload
    byte rx[sizeof(tx)];

    build_sd_command(cmd, args, tx);

    // Fill trailing dummy bytes
    for (int i = 6; i < sizeof(tx); i++) {
        tx[i] = SD_MOSI_IDLE_BITS;
    }

    SPI_cs_low(SD_CS_global);
    SPI_transfer_block(tx, rx, sizeof(tx), MODE_0);
    if (done) { SPI_cs_high(SD_CS_global); }

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
bool SD_read_block(uint32_t block_num, byte* block_data) {
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
    byte temp = SD_send_command_r1(17, args, false);
    if (temp != 0x0) {
        printf("expected response 0x0, got %x\n", temp);
        SPI_cs_high(SD_CS_global);
        return false;
    }

    // Wait for data token 0xFE
    int attempts = 0;
    byte token;
    do {
        token = SPI_transfer_byte(SD_MOSI_IDLE_BITS, MODE_0);
        if (++attempts > 10000) {
            SPI_cs_high(SD_CS_global);
            printf("Timeout waiting for data token\n");
            return false;
        }
    } while (token != 0xFE);
    // printf("Found token response after %d attempts\n", attempts);

    // Read 512 bytes
    for (int i = 0; i < 512; i++) {
        block_data[i] = SPI_transfer_byte(SD_MOSI_IDLE_BITS, MODE_0);
    }

    // Read CRC (2 bytes)
    SPI_transfer_byte(0xFF, MODE_0);
    SPI_transfer_byte(0xFF, MODE_0);
    // printf("CRC 0: %x\n", SPI_transfer_byte(0xFF, MODE_0));
    // printf("CRC 1: %x\n", SPI_transfer_byte(0xFF, MODE_0));

    // printf("should be ff since all transmission data read: %x\n", SPI_transfer_byte(0xFF, MODE_0));
    if (SPI_transfer_byte(0xFF, MODE_0) != 0xFF) {
        SPI_cs_high(SD_CS_global);
        return false;
    }

    SPI_cs_high(SD_CS_global);
    return true;
}

// static byte sd_get_response() {
//     byte response = SPI_transfer_byte(SD_MOSI_IDLE_BITS, MODE_0);
//     int count = 0;

//     while (response == SD_MOSI_IDLE_BITS && count < 8) {
//         response = SPI_transfer_byte(SD_MOSI_IDLE_BITS, MODE_0);
//         count++;
//     }

//     return response;
// }

/*
Reads num_blocks blocks of 512 bytes each including the starting block. Stores results in block_data
Strongly reccommended to not read more than ~32 blocks at a time due to stack limitations
*/
bool SD_read_many_blocks(uint32_t starting_block_num, byte* block_data, size_t num_blocks) {
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

    // CMD18 to read multiple blocks
    byte response = SD_send_command_r1(18, args, false);
    if (response != 0x0) {
        printf("CMD18 did not return the correct R1 response. Expected 0x0, got %x\n", response);
        SPI_cs_high(SD_CS_global);
        return false;
    }

    int attempts;
    // Read each block
    for (size_t i = 0; i < num_blocks; i++) {
        // Wait for 0xFE token for this block
        attempts = 0;
        byte token;
        do {
            token = SPI_transfer_byte(SD_MOSI_IDLE_BITS, MODE_0);
            // if (token != 0xFF) {printf("TOKEN: %x (block number %d)\n", token, (int)i);}
            if (++attempts > (int)1e5) {
                SPI_cs_high(SD_CS_global);
                printf("Timeout waiting for data token\n");
                return false;
            }
        } while (token != 0xFE);

        // Read 512 bytes
        for (int j = 0; j < 512; j++) {
            block_data[(512 * i) + j] = SPI_transfer_byte(SD_MOSI_IDLE_BITS, MODE_0);
        }

        // Discard CRC
        SPI_transfer_byte(SD_MOSI_IDLE_BITS, MODE_0);
        SPI_transfer_byte(SD_MOSI_IDLE_BITS, MODE_0);
    }
    // stop transmission with CMD12. Block until card is ready
    if (SD_send_command_r1b(12, NULL) != 0x0) {
        printf("CMD12 did not return the correct R1 response\n");
        SPI_cs_high(SD_CS_global);
        return false;
    }
    SPI_cs_high(SD_CS_global);
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
bool SD_read_CSD(byte* csd) {

    if (SD_send_command_r1(9, NULL, false) != 0x0) {
        printf("R1 is incorrect\n");
        SPI_cs_high(SD_CS_global);
        return false;
    }
    // wait for data token 0xFE
    int count = 0;
    byte token = 0xFF;
    do {
        token = SPI_transfer_byte(SD_MOSI_IDLE_BITS, MODE_0);
        if (token == 0xFE) break;
        if (++count > (int)1e5) {
            printf("Timeout waiting for FE token\n");
            SPI_cs_high(SD_CS_global);
            return false;
        }
    } while (1);

    for (int i = 0; i < 16; i++) {
        csd[i] = SPI_transfer_byte(SD_MOSI_IDLE_BITS, MODE_0);
    }

    // 2 CRC bytes
    SPI_transfer_byte(SD_MOSI_IDLE_BITS, MODE_0);
    SPI_transfer_byte(SD_MOSI_IDLE_BITS, MODE_0);

    SPI_cs_high(SD_CS_global);
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
uint32_t SD_get_block_count(byte* csd) {
    
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
bool SD_write_block(uint32_t block_num, const byte* data_to_write) {
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

    // CMD24 to write one block of BLOCK_LEN (assumes length is 512)
    if (SD_send_command_r1(24, args, false) != 0x0) {
        printf("R1 response for SD_write_block() did not match expected value\n");
        SPI_cs_high(SD_CS_global);
        return false;
    }
    // Start token
    SPI_transfer_byte(0xFE, MODE_0);

    // Send data
    for (int i = 0; i < 512; i++) {
        SPI_transfer_byte(data_to_write[i], MODE_0);
    }

    // Dummy CRC
    SPI_transfer_byte(SD_MOSI_IDLE_BITS, MODE_0);
    SPI_transfer_byte(SD_MOSI_IDLE_BITS, MODE_0);

    // read the data response token from the card
    byte response;
    int attempts = 0;
    do {
        response = SPI_transfer_byte(SD_MOSI_IDLE_BITS, MODE_0);
        attempts++;
        // bit 4 must be 0 and bit 0 must be 1 to indicate a data response
    } while ((response & 0x11) != 0x01 && attempts < SD_RESPONSE_TIMEOUT);
    if (attempts == SD_RESPONSE_TIMEOUT) {
        printf("Timeout waiting for response token in SD_write_block()\n");
        SPI_cs_high(SD_CS_global);
        return false;
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
        return false;
    }

    // wait for busy period to end before we can transmit again
    attempts = 0;
    while (SPI_transfer_byte(SD_MOSI_IDLE_BITS, MODE_0) != 0xFF && (++attempts < (int)1e5));
    if (attempts == (int)1e5) {
        printf("timed out waiting for busy period to end in SD_write_block() function\n");
        SPI_cs_high(SD_CS_global);
        return false;
    }
    SPI_cs_high(SD_CS_global);
    return true;
}

bool SD_write_many_blocks(uint32_t starting_block_num, const byte* data_to_write, size_t num_blocks) {
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

    // CMD25 to write multiple block of BLOCK_LEN (assumes length is 512)
    if (SD_send_command_r1(25, args, false) != 0x0) {
        printf("R1 response for SD_write_many_blocks() did not match expected value\n");
        goto exit_with_failure;
    }
    unsigned int attempts = 0;
    for (int i = 0; i < num_blocks; i++) {
        // Start token (not the same as single block write or reads!)
        SPI_transfer_byte(0xFC, MODE_0);

        // Send data
        for (int j = 0; j < 512; j++) {
            SPI_transfer_byte(data_to_write[512 * i + j], MODE_0);
        }

        // Dummy CRC
        SPI_transfer_byte(SD_MOSI_IDLE_BITS, MODE_0);
        SPI_transfer_byte(SD_MOSI_IDLE_BITS, MODE_0);

        // read the data response token from the card
        byte response;
        do {
            // the upper 3 bits of the response are don't care
            response = SPI_transfer_byte(SD_MOSI_IDLE_BITS, MODE_0) & 0x1F;
            attempts++;
            // bit 4 must be 0 and bit 0 must be 1 to indicate a data response
        } while ((response & 0x11) != 0x01 && attempts < SD_RESPONSE_TIMEOUT);
        if (attempts == SD_RESPONSE_TIMEOUT) {
            printf("Timed out waiting for SD card response token in SD_write_many_blocks()\n");
            goto exit_with_failure;
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
            return false;
        }
        // wait for busy period to end before we can transmit again
        attempts = 0;
        while (SPI_transfer_byte(SD_MOSI_IDLE_BITS, MODE_0) !=0xFF && (++attempts < (int)1e5));
        if (attempts == (int)1e5) {
            printf("Timed out waiting for the busy condition to end in SD_write_many_blocks()\n");
            goto exit_with_failure;
        }
    }
    // send the STOP TRAN token
    SPI_transfer_byte(0xFD, MODE_0);
    // wait for card to end the busy condition again
    attempts = 0;
    while (SPI_transfer_byte(SD_MOSI_IDLE_BITS, MODE_0) != 0xFF && (++attempts < (int)1e5));
    if (attempts == (int)1e5) {
        printf("Timed out waiting for the busy condition to end in SD_write_many_blocks()\n");
        goto exit_with_failure;
    }

    SPI_cs_high(SD_CS_global);
    return true;
    exit_with_failure:
    SPI_cs_high(SD_CS_global);
    return false;
}

bool SD_clear_block(uint32_t block_num) {
    // if block is already empty then return early
    if (SD_is_block_empty(block_num)) return true;

    // else write 512 0s to the block
    byte zeroes[512] = {0}; // set all elements to 0
    return SD_write_block(block_num, (const byte*)zeroes);
}

bool SD_clear_many_blocks(uint32_t starting_block_num, size_t num_blocks) {
    byte *zeroes = calloc(num_blocks, 512); // num_blocks entries of size 512 bytes
    if (!zeroes) {
        printf("malloc failed inside SD_clear_many_blocks() function\n");
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
