#ifndef SD_CARD_SPI_H
#define SD_CARD_SPI_H
#include "my_SPI.h" // use the SPI mode of the SD card
/*
Written for XTSD04GLGEAG SD card, but should work for any SPI SD card
SD card protocol is used for SD cards
Uses SPI mode 0
*/

bool SD_card_init(gpio_num_t SD_card_chip_select);
uint32_t SD_get_number_of_512_byte_blocks(void);
bool SD_read_block(uint32_t block_num, byte* block_data);
bool SD_read_many_blocks(uint32_t starting_block_num, byte* block_data, size_t num_blocks);
bool SD_write_block(uint32_t block_num, const byte* data_to_write);
bool SD_write_many_blocks(uint32_t starting_block_num, const byte* data_to_write, size_t num_blocks);

/*
returns true if all 512 bytes at the address block_num are 0, else false
*/
bool SD_is_block_empty(uint32_t block_num);
bool SD_clear_block(uint32_t block_num);
bool SD_clear_many_blocks(uint32_t starting_block_num, size_t num_blocks);
#endif /* SD_CARD_SPI_H */