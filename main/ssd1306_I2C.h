#ifndef SSD1306_I2C_H // header guard
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

// determines if we want to set or clear pixlels when drawing lines/rectangles
typedef enum {
    PIXEL_CLEAR = 0,
    PIXEL_SET = 1
} PIXEL_MODE;

/**
 * @brief Initialize the SSD1306 OLED display over I2C.
 *
 * Configures the display with recommended startup settings:
 * oscillator, multiplex ratio, charge pump, remap, COM scan direction,
 * contrast, pre-charge, VCOM deselect, addressing mode, and clears the screen.
 *
 * Leaves the display in PAGE addressing mode.
 * @return True on success, else false
 */
bool ssd1306_init(void);

/**
 * @brief Set the display contrast.
 *
 * @param contrast Value between 0x00 and 0xFF.
 * @return True on success, else false
 */
bool ssd1306_set_contrast(byte contrast);

/**
 * @brief Forces the entire display to be on, ignoring GDDRAM contents.
 * @return True on success, else false
 */
bool ssd1306_entire_display_on(void);


/**
 * @brief Invert all pixels on the display.
 *
 * White becomes black, black becomes white.
 * @return True on success, else false
 */
bool ssd1306_invert_display(void);

/**
 * @brief Restore normal (non-inverted) pixel mapping.
 * @return True on success, else false
 */
bool ssd1306_normal_display(void);

/**
 * @brief Turn the OLED panel on.
 * @return True on success, else false
 */
bool ssd1306_display_on(void);

/**
 * @brief Turn the OLED panel off.
 * @return True on success, else false
 */
bool ssd1306_display_off(void);

/**
 * @brief Refresh the display with the contents of the internal GDDRAM buffer.
 *
 * Sends all 8 pages (8×128 = 1024 bytes) to the SSD1306.
 * @return True on success, else false
 */
bool ssd1306_refresh_display(void);

/**
 * @brief Clear the entire screen.
 *
 * Zeros the internal GDDRAM buffer and updates the display.
 * @return True on success, else false
 */
bool ssd1306_clear_screen(void);

/**
 * @brief Write a string of ASCII characters using the 8×8 font.
 *
 * Draws characters between two X boundaries, wrapping to the next page
 * when needed. Only dirty pages are refreshed for efficiency.
 *
 * @param str Null-terminated string to draw.
 * @param x_left Leftmost column boundary.
 * @param x_right Rightmost column boundary.
 * @param start_page Page index (0–7) to start drawing from.
 * @param flush If true, immediately update the display
 * @return True on success, else false
 */
bool ssd1306_write_string_size8x8p(const char* string_to_print, byte x_offset_pixels_left,
                                   byte x_offset_pixels_right, byte start_page, bool flush);

/**
 * @brief Refresh a single page of the display.
 *
 * @param page_to_refresh Page index (0–7).
 * @return True on success, else false
 */
bool ssd1306_refresh_page(byte page_to_refresh);

/**
 * @brief Set a single pixel by coordinates.
 *
 * @param pixel_coords Pixel coordinate struct (x: 0–127, y: 0–63).
 * @param on_off SET to show, or CLEAR to erase
 * @param flush If true, immediately update the affected page.
 * @return True on success, else false
 */
bool ssd1306_set_pixel(ssd1306_pixel_coordinate pixel_coords, PIXEL_MODE on_or_off, bool flush);

/**
 * @brief Set a single pixel ON by raw x/y coordinates.
 *
 * @param x Column index (0–127).
 * @param y Row index (0–63).
 * @param flush If true, immediately update the affected page.
 * @return True on success, else false
 */
bool ssd1306_set_pixel_xy(byte x, byte y, PIXEL_MODE on_or_off, bool flush);

/**
 * @brief Verify if pixel coordinates are within the display bounds.
 *
 * @param coordinate Pixel coordinate.
 * @return True on success, else false
 */
bool ssd1306_verify_coordinates_are_valid(ssd1306_pixel_coordinate coordinate);

/**
 * @brief Draw a straight line between two points.
 *
 * Uses Bresenham’s line algorithm.
 *
 * @param start Starting coordinate.
 * @param end Ending coordinate.
 * @param flush If true, updates affected pages after drawing.
 * @return True on success, else false
 */
bool ssd1306_draw_line(ssd1306_pixel_coordinate p1, ssd1306_pixel_coordinate p2, bool flush);

/**
 * @brief Draw a straight line of cleared pixlels between two points.
 *
 * Uses Bresenham’s line algorithm.
 *
 * @param start Starting coordinate.
 * @param end Ending coordinate.
 * @param flush If true, updates affected pages after drawing.
 * @return True on success, else false
 */
bool ssd1306_clear_line(ssd1306_pixel_coordinate p1, ssd1306_pixel_coordinate p2, bool flush);

/**
 * @brief Draw a horizontal line.
 *
 * @param y Y-coordinate.
 * @param x1 Starting X-coordinate.
 * @param x2 Ending X-coordinate.
 * @param flush If true, updates affected pages after drawing.
 * @return True on success, else false
 */
bool ssd1306_draw_hline(byte y, byte x1, byte x2, bool flush);

/**
 * @brief Draw a horizontal line of cleared pixels.
 *
 * @param y Y-coordinate.
 * @param x1 Starting X-coordinate.
 * @param x2 Ending X-coordinate.
 * @param flush If true, updates affected pages after drawing.
 * @return True on success, else false
 */
bool ssd1306_clear_hline(byte y, byte x1, byte x2, bool flush);

/**
 * @brief Draw a vertical line.
 *
 * @param x X-coordinate.
 * @param y1 Starting Y-coordinate.
 * @param y2 Ending Y-coordinate.
 * @param flush If true, updates affected pages after drawing.
 * @return True on success, else false
 */
bool ssd1306_draw_vline(byte x, byte y1, byte y2, bool flush);

/**
 * @brief Draw a vertical line of cleared pixels.
 *
 * @param x X-coordinate.
 * @param y1 Starting Y-coordinate.
 * @param y2 Ending Y-coordinate.
 * @param flush If true, updates affected pages after drawing.
 * @return True on success, else false
 */
bool ssd1306_clear_vline(byte x, byte y1, byte y2, bool flush);

/**
 * @brief Draw a rectangle.
 *
 * Can be filled or outlined with specified thickness.
 *
 * @param origin Top-left corner of the rectangle.
 * @param width_px Rectangle width in pixels.
 * @param height_px Rectangle height in pixels.
 * @param border_thickness_px Outline thickness in pixels (ignored if filled).
 * @param fill If true, fill rectangle, else draw outline.
 * @return True on success, else false
 */
bool ssd1306_draw_rectangle(ssd1306_pixel_coordinate origin, byte width_px, byte height_px, byte border_thickness_px, bool fill);

/**
 * @brief Draw a rectangle of cleared pixels.
 *
 * used to "erase" a block on the display.
 *
 * @param origin Top-left corner of the rectangle.
 * @param width_px Rectangle width in pixels.
 * @param height_px Rectangle height in pixels.
 * @return True on success, else false
 */
bool ssd1306_clear_rectangle(ssd1306_pixel_coordinate origin, byte width_px, byte height_px);

#endif // ssd1306_I2C_H