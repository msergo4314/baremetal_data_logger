#ifndef ssd1306_I2C_H // header guard
#define SSD1306_I2C_H

#include <string.h>
#include "my_I2C.h" // custom I2C bit-banged protocol implementation

#define SSD1306_ADDRESS     0x3D // can be 0x3C or 0x3D depending on the D/C# pin
#define SSD1306_NUM_PAGES   8 // a page is a horizontal slice of the screen 8 pixels tall
#define SSD1306_OLED_WIDTH  128 // in pixels
#define SSD1306_OLED_HEIGHT 64 // in pixels

// coordinates
#define SSD1306_TOP_LEFT (0, 0)
#define SSD1306_TOP_RIGHT (SSD1306_OLED_WIDTH - 1, 0)
#define SSD1306_BOTTOM_LEFT (0, SSD1306_OLED_HEIGHT - 1)
#define SSD1306_BOTTOM_RIGHT (SSD1306_OLED_WIDTH - 1, SSD1306_OLED_HEIGHT - 1)

// these masks will be used to determine what contorl bytes will be for transmissions
#define SSD1306_CO_BIT (byte)(1 << 7)   // Continuation bit (bit 7)
#define SSD1306_DC_BIT (byte)(1 << 6)   // Data/Command bit (bit 6)

// macro to determine what the control byte looks like for the start of any transmission
#define SSD1306_CONTROL_BYTE(co, dc) ((co ? SSD1306_CO_BIT : 0) | (dc ? SSD1306_DC_BIT : 0))

/*
PAGE MODE
The Page address pointer is selected first, then the start column and end column. The columns increment on each read/write
and if the column address pointer reaches the column end address it resets to column start WITHOUT updating the page pointer
this means it will overwrite data instead of going to the next page

HORIZONTAL MODE
Works identically to PAGE addressing but increments the page pointer when the column pointer reaches the column end
this essentially means you can write subsequent lines 

VERTICAL MODE
the page address pointer is incremented by 1 each read/write instead of the column. 
Wraps to next column when all pages are set
*/

typedef enum {
    PAGE,
    HORIZONTAL, // same as PAGE but wraps around to the next page instead of the same page
    VERTICAL
} ADDRESSING_MODE;

extern ADDRESSING_MODE current_mode;

typedef enum {
    ON = 1,
    OFF = 0
} ON_OFF;

/*
instead of reading the GDDRAM to preserve data on pages, 
we will track it fully in software to save time and reduce complexity
128 * 8 = 1024 bytes total --> 128 columns and 8 pages
*/
extern byte ssd1306GDDRAM_buffer[8][128];

typedef struct pixel_coord {
    // the OLED measures 64x128 pixels so a byte is plenty of space for X and Y
    byte x;
    byte y;
} ssd1306_pixel_coordinate;

// Function prototypes

bool ssd1306_write_bytes(const byte* stream_of_bytes, size_t number_of_bytes, bool start, bool stop);
bool ssd1306_set_addressing_mode(const ADDRESSING_MODE mode);
bool ssd1306_init(void);
bool ssd1306_set_contrast(byte contrast);
bool ssd1306_entire_display_on(void);
bool ssd1306_invert_display(void);
bool ssd1306_normal_display(void);
bool ssd1306_nop(void);
bool ssd1306_display_on(void);
bool ssd1306_display_off(void);
bool ssd1306_refresh_display(void);
bool ssd1306_set_page_address(byte page);
bool ssd1306_set_column_address(byte column);
bool ssd1306_set_column_start_and_end(byte column_start, byte column_end);
bool ssd1306_clear_screen(void);
bool ssd1306_write_string_size8x8p(const char* string_to_print, byte x_offset_pixels_left,
                                   byte x_offset_pixels_right, byte start_page);
bool ssd1306_show_RAM_only(void);
bool ssd1306_refresh_page(byte page_to_refresh);
bool ssd1306_set_pixel(ssd1306_pixel_coordinate pixel_coords, ON_OFF on_or_off, bool flush);
bool ssd1306_set_pixel_xy(byte x, byte y, ON_OFF on_or_off, bool flush);
bool verify_coordinates_are_valid(ssd1306_pixel_coordinate coordinate);
bool ssd1306_draw_line(ssd1306_pixel_coordinate p1, ssd1306_pixel_coordinate p2, bool flush);
bool ssd1306_draw_hline(byte x, byte y1, byte y2, bool flush);
bool ssd1306_draw_vline(byte y, byte x1, byte x2, bool flush);
bool ssd1306_draw_rectangle(ssd1306_pixel_coordinate origin, byte width_px, byte height_px, byte border_thickness_px, bool fill);

#endif // ssd1306_I2C_H