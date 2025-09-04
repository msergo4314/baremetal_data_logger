#ifndef SD_CARD_SPI_H
#define SD_CARD_SPI_H
#include "my_SPI.h" // use the SPI mode of the SD card
/*
Written for XTSD04GLGEAG SD card, but should work for any SPI SD card

SD card protocol is used for SD cards and is NOT explained in the datasheet

Uses SPI mode 0 or 3 (0 is easier)
*/

bool SD_card_init(gpio_num_t SD_card_chip_select);
bool SD_read_block(uint32_t block_num, byte* block_data);
#endif /* SD_CARD_SPI_H */