#include "SD_card_SPI.h"
#include "esp_rom_sys.h" // for timing
#include <string.h>

/*

Communication with the SD card is performed by sending commands to it and receiving responses from it.
A valid SD card command consists of 48 bits. The leftmost two bits are the start bits
which we set to (01). They are followed by a 6-bit command number and a 32-bit argument where
additional information may be provided. Next, there are 7 bits containing a Cyclic Redundancy Check
(CRC) code, followed by a single stop bit (1)

*/

// SD protocol specifies MOSI is high when not transmitting 
#define SD_MOSI_IDLE_BITS 0xFF 

// masks for R1 response error bits
#define R1_RESPONSE_IDLE_ERROR              0
#define R1_RESPONSE_ERASE_RESET_ERROR       1 << 1
#define R1_RESPONSE_ILLEGAL_COMMAND_ERROR   1 << 2
#define R1_RESPONSE_COMMAND_CRC_ERROR       1 << 3
#define R1_RESPONSE_ERASE_SEQUENCE_ERROR    1 << 4
#define R1_RESPONSE_ADDRESS_ERROR           1 << 5
#define R1_RESPONSE_PARAMETER_ERROR         1 << 6

typedef byte SD_COMMAND[6]; // 6 * 8 = 48

// static byte CMD0[6] = {0x40 + 0, 0x00, 0x00, 0x00, 0x00, 0x95};
// static byte CMD8[6] = {0x40 + 8, 0x00, 0x00, 0x01, 0xAA, 0x87};
// static byte CMD58[6] = {0x40 + 58, 0x00, 0x00, 0x00, 0x00, 0x01};
// static byte CMD55[6] = {0x40 + 55, 0x00, 0x00, 0x00, 0x00, 0x01};
// static byte ACMD41[6] = {0x40 + 41, 0x40, 0x00, 0x00, 0x00, 0x01};
// static byte CMD17[6] = {0x40 + 17, 0x00, 0x00, 0x00, 0x00, 0x01};

static byte SD_get_response(gpio_num_t SD_card_cs);

// Send a 6 byte command to the SD card
// Returns the first byte of the response
byte SD_send_command(gpio_num_t SD_card_cs, SD_COMMAND command);

// actually returns a SD_COMMAND (length is 6 bytes)
static byte* create_SD_command_from_code(byte command_number, byte* arguments);

// Note: must free pointer later
static byte* create_SD_command_from_code(byte command_number, byte* arguments) {
    // Allocate 6-byte buffer
    byte* command = (byte*)malloc(6);
    if (!command) return NULL;

    // First byte: start bit (0), transmission bit (1), then 6-bit command index
    command[0] = 0x40 | (command_number & 0x3F);

    // Next four bytes: argument (big-endian)
    if (arguments) {
        memcpy(&command[1], arguments, 4);
    } else {
        memset(&command[1], 0, 4);
    }

    // CRC byte:
    if (command_number == 0) {
        command[5] = 0x95;  // CMD0 must have valid CRC
    } else if (command_number == 8) {
        command[5] = 0x87;  // CMD8 must have valid CRC
    } else {
        command[5] = 0xFF;      // dummy CRC after init
    }

    return command;
}

// sends a command to the SD card and returns first byte of response
byte SD_send_command(gpio_num_t SD_card_cs, SD_COMMAND command) {
    SPI_transfer_block(SD_card_cs, (byte*)command, NULL, 6);
    return SD_get_response(SD_card_cs);
}


/*
Once the SD card receives a command it will begin processing it. To respond to a command, the SD card
requires the SD CLK signal to toggle for at least 8 cycles. The MOSI line must remain high
*/
static byte SD_get_response(gpio_num_t SD_card_cs) {
    byte empty[1] = {SD_MOSI_IDLE_BITS};
    byte response;
    // send 0xFF and catch return byte
    SPI_transfer_block(SD_card_cs, empty, &response, 1);

    int count = 0;
    while (response == 0xFF && count < 10) {
        SPI_transfer_block(SD_card_cs, empty, &response, 1);
        count++;
    }
    return response;
}

/*
initialize the SPI mode of the SD card
*/
void SD_card_init(gpio_num_t SD_card_chip_select) {
    SPI_attach_device(SD_card_chip_select, MODE_0, CS_ACTIVE_LOW);

    // after power reaches > 2.2 V, wait at least 1 ms.
    esp_rom_delay_us(1000); // likely not needed but cheap to do

    // SET MOSI and CS to logic 1
    SPI_set_mosi(1);
    

    // SPI clock rate should be 100-400 KHz
    // SPI_clock_toggle(74);
}