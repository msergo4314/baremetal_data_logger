#ifndef SD_CARD_H
#define SD_CARD_H
/*
Written for XTSD04GLGEAG SD card, but should work for any SPI SD card
SD card protocol is used for SD cards
Uses SPI mode 0

Uses the built in SPI hardware of the ESP32
*/
#include <stdio.h>
#include <driver/gpio.h>
#include <driver/spi_master.h>

// VSPI ports (can also use HSPI)
#define SPI_MOSI (gpio_num_t)23
#define SPI_MISO (gpio_num_t)19
#define SPI_CLK  (gpio_num_t)18

typedef uint8_t byte;

typedef enum {
    SDSC_TYPE, // SDSC (standard capacity)
    SDHC_SDXC_TYPE, //SDHC/ SDXC (High capacity or very high capacity)
    UNKNOWN_TYPE = -1
} SD_CARD_TYPE;

/**
 * @brief Initialize the SD card in SPI mode.
 *
 * Configures SPI at a safe low frequency, sends reset and version commands,
 * verifies card voltage, and identifies the card type (SDSC or SDHC/SDXC).
 * Also determines the total number of 512-byte blocks.
 *
 * @param SD_card_chip_select GPIO pin used for chip select (CS).
 * @return true if initialization succeeds, false otherwise.
 */
bool SD_card_init(gpio_num_t SD_card_chip_select);

/**
 * @brief Read a single 512-byte block from the SD card.
 *
 * Issues CMD17 and waits for the 0xFE data token, then reads one block.
 *
 * @param block_num   The block index (512-byte unit).
 * @param block_data  Pointer to buffer where the data will be stored (must be 512 bytes or more).
 * @return true if the read succeeds, false otherwise.
 */
bool SD_read_block(uint32_t block_num, byte *block_data);

/**
 * @brief Read multiple 512-byte blocks from the SD card.
 *
 * Issues CMD18 and streams `num_blocks` blocks into `block_data`.
 * Stops transmission with CMD12 when done.
 *
 * @param starting_block_num The starting block index.
 * @param block_data         Pointer to buffer where data will be stored (must be at least 512 * num_blocks bytes long).
 * @param num_blocks         Number of blocks to read.
 * @return true if the read succeeds, false otherwise.
 */
bool SD_read_many_blocks(uint32_t starting_block_num, byte *block_data, size_t num_blocks);

/**
 * @brief Write 512 bytes to a single block on the SD card.
 *
 * Issues CMD24 and writes one block with a 0xFE start token.
 * Waits for a data response and busy period before finishing.
 *
 * @param block_num     The block index.
 * @param data_to_write Pointer to 512-byte buffer with the data.
 * @return true if the write succeeds, false otherwise.
 */
bool SD_write_block(uint32_t block_num, const byte *data_to_write);

/**
 * @brief Write multiple 512-byte blocks to the SD card.
 *
 * Issues CMD25 and streams `num_blocks` blocks, each prefixed with
 * 0xFC start tokens. Ends with 0xFD stop token. Waits for busy
 * periods to finish before returning.
 *
 * @param starting_block_num The starting block index.
 * @param data_to_write      Pointer to buffer with data (512*num_blocks bytes).
 * @param num_blocks         Number of blocks to write (includes starting block).
 * @return true if the write succeeds, false otherwise.
 */
bool SD_write_many_blocks(uint32_t starting_block_num, const byte *data_to_write, size_t num_blocks);

/**
 * @brief Check if a block is empty (all bytes zero).
 *
 * Reads the block and inspects its contents.
 *
 * @param block_num The block index.
 * @return true if the block is empty, false otherwise.
 */
bool SD_is_block_empty(uint32_t block_num);

/**
 * @brief Deinits the SD card and frees internal DMA blocks
 * 
 * 
 * @return true if successful, else false
 */
bool SD_deinit(void);

/**
 * @brief locks the SPI bus. Unlock with SD_unlock()
 * 
 * * must be called after the init
 */
void SD_lock(void);
/**
 * @brief unlocks the SPI bus which was previously locked with SD_lock()
 * 
 * must be called after the init
 */
void SD_unlock(void);

/**
 * @brief Clear a block by writing all zeroes.
 *
 * If already empty, nothing is written.
 *
 * @param block_num The block index.
 * @return true if the block is cleared, false otherwise.
 */
bool SD_clear_block(uint32_t block_num);

/**
 * @brief Clear multiple blocks by writing all zeroes.
 *
 * Allocates a buffer of zeroes and writes it using multi-block write.
 *
 * @param starting_block_num The starting block index.
 * @param num_blocks         Number of blocks to clear.
 * @return true if the operation succeeds, false otherwise.
 */
bool SD_clear_many_blocks(uint32_t starting_block_num, size_t num_blocks);

/**
 * @brief Get the number of 512-byte blocks on the SD card.
 *
 * Reads the CSD register and parses it to compute capacity.
 *
 * @return Total block count, or 0 on failure.
 */
uint32_t SD_get_number_of_512_byte_blocks(void);

/**
 * @brief gets the type of SD card
 *
 * @return the type of SD card (high or standard capacity) as an enum
 */
SD_CARD_TYPE SD_get_type(void);
#endif /* SD_CARD_H */