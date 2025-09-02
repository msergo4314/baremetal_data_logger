#include "ssd1306_I2C.h"

// the ssd1306 supports 3 addressing modes: Page, Horizontal, and Vertical
ADDRESSING_MODE current_mode;
byte ssd1306GDDRAM_buffer[8][128] = {};
/*

When transmitting bytes with I2C, we have two options for the OLED -- a command or data
commands tell the display to do something, such as changing contrast or turning all pixels ON/OFF
data tells us which bits to turn on and off using the paging system of the ssd1306


Contorl byte has the format C0, D/C, 0, 0, 0, 0, 0, 0 (6 trailing 0s) -- only the 2 MSBs matter

C0 determines if the following transmission will containg data bytes ONLY. If it is 0, this is the case

D/C determines if the next byte is data or a command 
--> 0 for commands, and 1 for data stored in the GDDRAM (note: the GDDRAM pointer will increase for every data write)

Each control or data byte will have an ACK bit

GDDRAM
a bit mapped static RAM which holds the bit pattern to be displayed. Thd size of the RAM is 128x64 bits (the size of the screen)
the RAM is didvided into 8 pages from PAGE0 to PAGE7

the coordinate (0, 0) is located at the top left of the screen (assuming no column/row remapping). this is page 0, column 0
Each page has a height of 8 pixels from top to bottom (8x8 = 64) and a width of 128 segments (indexed 0-127 from left to right assuming no remaps)

When one byte is written to GDDRAM, all rows of that page are refreshed for a column length of 8 bits.
Bit D0 (the LSB) is written at the top and D7 at the bottom

*/

// helpers not to be used outside of this file
static byte* get_bitmap_from_ascii(byte character);
static inline bool ssd1306_write_command(byte command_code);
static inline bool ssd1306_write_command2(byte command_code, byte command_argument);
static bool ssd1306_set_page_address(byte page);
static bool ssd1306_set_column_address(byte column);
static bool ssd1306_set_column_start_and_end(byte column_start, byte column_end);
static bool ssd1306_nop(void);
static bool ssd1306_write_bytes(const byte* stream_of_bytes, size_t number_of_bytes, bool start, bool stop);
static bool ssd1306_set_addressing_mode(const ADDRESSING_MODE mode);

// shows what is in GDDRAM on the chip and nothing else
static bool ssd1306_show_RAM_only(void) {
    // A4 is the command for entire display ON with RAM contents showing
    return ssd1306_write_command(0xA4);
}

static bool ssd1306_set_page_address(byte page) {
    if (current_mode != PAGE) {
        printf("Must be in PAGE mode\n");
        return false;
    }
    // command for setting the page in page address mode is 0xB0. However, it is OR'd with page (0-7)
    // the actual command thus ranges from B0 to B7
    return ssd1306_write_command(0xB0 | page);
}

static bool ssd1306_set_column_address(byte column) {
    if (current_mode != PAGE) {
        printf("Must be in PAGE mode\n");
        return false;
    }
    // address is sent in two bytes with each byte being one nibble (4 bits)
    return ssd1306_write_command2(column & 0xF, ((column >> 4) | 0x10));
}

// sets column start and end for HORIZONTAL and VERTICAL modes
static bool __attribute__((unused)) ssd1306_set_column_start_and_end(byte column_start, byte column_end) {
    if (current_mode == PAGE) {
        printf("cannot set column start and end in page mode!\n");
        return false;
    }
    if (column_start > 127 || column_end > 127) {
        printf("Desired columns are too large. Use values in the range of (0-127)\n");
        return false;
    }
    byte transmission[4] = {SSD1306_CONTROL_BYTE(0, 0), 0x21, column_start, column_end};
    return ssd1306_write_bytes(transmission, sizeof(transmission), true, true);
}

// wraps the I2C function for the ssd1306 display
static bool ssd1306_write_bytes(const byte* stream_of_bytes, size_t number_of_bytes, bool start, bool stop) {
    return I2C_send_byte_stream(SSD1306_ADDRESS, stream_of_bytes, number_of_bytes, WRITE, start, stop);
}

static bool ssd1306_set_addressing_mode(const ADDRESSING_MODE mode) {
    byte lower_bits = 0x0;
    // the mode we want will determine what the last two bits of the command are
    switch(mode) {
        case PAGE:
        // this is considered a RESET since it's the default
            lower_bits = (byte)0b10;
        break;
        case HORIZONTAL:
            lower_bits = (byte)0b00;
        break;
        case VERTICAL:
            lower_bits = (byte)0b01;
        break;
        default:
            // datasheet says this is INVALID, so should never be used
            lower_bits = (byte)0b11;
        break;
    }
    // command is 0x20 for changing the addressing mode. Follow with the desired mode.
    // The first 6 bits of the third byte are X (don't care) so here they are 0s
    return ssd1306_write_command2(0x20, (byte)lower_bits);
}

// setup to make sure the SSD1306 is ready to use
bool ssd1306_init(void) {
    /*
    Here we set a bunch of parameters to recommended values
    This is necessary to make sure we can boot the display into a known state (all parameters defined) on system reset
    */

    // Always reset the display into a known state
    if (!ssd1306_display_off()) return false; // Display OFF
    // Set display clock divide ratio/oscillator frequency
    // 0x80 = recommended oscillator frequency
    if (!ssd1306_write_command2(0xD5, 0x80)) return false;
    // Set multiplex ratio
    // 0x3F = 1/64 duty (for 128x64 display)
    if (!ssd1306_write_command2(0xA8, 0x3F)) return false;
    // Set display offset
    // 0x00 = no vertical shift
    if (!ssd1306_write_command2(0xD3, 0x00)) return false;

    // Set display start line to 0
    if (!ssd1306_write_command(0x40)) return false;

    // Enable charge pump regulator
    // 0x14 = enable charge pump (required for internal VCC)
    if (!ssd1306_write_command2(0x8D, 0x14)) return false;

    // Use page addressing mode by default
    if (!ssd1306_set_addressing_mode(PAGE)) return false;

    // Segment remap
    // 0xA1 = column address 127 is mapped to SEG0 (mirror horizontally)
    if (!ssd1306_write_command(0xA1)) return false;

    // COM output scan direction
    // 0xC8 = remapped mode (flip vertically)
    if (!ssd1306_write_command(0xC8)) return false;

    // Set COM pins hardware configuration
    // 0x12 = alternative COM pin configuration, disable left/right remap
    if (!ssd1306_write_command2(0xDA, 0x12)) return false;

    // Set contrast to max
    if (!ssd1306_set_contrast(0xFF)) return false;

    // Set pre-charge period
    // 0xF1 = higher precharge for better contrast
    if (!ssd1306_write_command2(0xD9, 0xF1)) return false;

    // Set VCOMH deselect level
    // 0x40 = about 0.77 * Vcc
    if (!ssd1306_write_command2(0xDB, 0x40)) return false;

    // Set normal (non-inverted) display mode
    if (!ssd1306_normal_display()) return false;
    if (!ssd1306_clear_screen()) return false;
    if (!ssd1306_display_on()) return false;

    // Default to PAGE addressing mode
    current_mode = PAGE;
    return true;
}

// sets contrast of display. Higher byte value correlates to higher contrast
bool ssd1306_set_contrast(byte contrast) {
    // to set contrast we will transmit two bytes -- the command byte which indicates we want to change the contrast, and the value itself (data byte)
    return ssd1306_write_command2(0x81, contrast);
}

bool ssd1306_entire_display_on(void) {
    // A5 is the command for entire display ON with no regard for RAM content
    return ssd1306_write_command(0xA5);
}

// changes behaviour to 0 in RAM -> ON in display panel and 1 in RAM -> OFF in display panel
bool ssd1306_invert_display(void) {
    return ssd1306_write_command(0xA7);
}

// changes behaviour to the normal behaviour: 0 in RAM -> OFF in display and 1 in RAM -> ON on display
bool ssd1306_normal_display(void) {
    return ssd1306_write_command(0xA6);
}

// could be useful if you want to waste time
bool __attribute__((unused)) ssd1306_nop(void) {
    return ssd1306_write_command(0xE3);
}

// turns entire display ON
bool ssd1306_display_on(void) {
    return ssd1306_write_command(0xAF);
}

// turns entire display OFF (sleep mode)
bool ssd1306_display_off(void) {
    return ssd1306_write_command(0xAE);
}

// displays the contents of the INTERNAL GDDRAM (the 2D array)
bool ssd1306_refresh_display(void) {
    if (current_mode != PAGE) {
        if (!ssd1306_set_addressing_mode(PAGE)) return false;
    }
    // we will write the internal memory for each page instead of all at once for reliability
    // this is because I2C could fail if we hold the bus too long (not likely to happen though)
    for (byte page = 0; page < 8; page++) {
        ssd1306_set_page_address(page);
        ssd1306_set_column_address(0);
        byte buffer[129];
        buffer[0] =SSD1306_CONTROL_BYTE(0, 1);
        memcpy(&buffer[1], &(ssd1306GDDRAM_buffer[page][0]), SSD1306_OLED_WIDTH);
        if (!ssd1306_write_bytes(buffer, sizeof(buffer), true, true)) return false;
    }
    return ssd1306_show_RAM_only();
}
// clears screen by setting GDDRAM to 0 and calling ssd1306_refresh_display()
bool ssd1306_clear_screen(void) {
    // to clear, write all 0s into RAM buffer, then write the whole buffer
    memset(ssd1306GDDRAM_buffer, 0x0, 1024); // set the internal buffer to be all 0s
    return ssd1306_refresh_display();
}

/*
write a string to the OLED with a specified padding of spaces. Font will be 8x8 pixels.
only supports the visible ASCII characters (32-126).

NOTE: the left pixel offset determines the number of pixels that are NOT set (blank)
*/
bool ssd1306_write_string_size8x8p(const char* string_to_print, byte x_offset_pixels_left,
                                   byte x_offset_pixels_right, byte start_page) {
    // each character we print will be 8x8 pixels, so we can print a max of 16 characters if x_offset_pixels = 0
    if (!string_to_print) {
        printf("Passed NULL pointer\n");
        return false;
    }
    size_t len = strlen(string_to_print);
    // There are only 8 pages so start_page must be < 8
    // we want to be able to print at least one character per page
    if (start_page >= SSD1306_NUM_PAGES ||
    x_offset_pixels_left >= SSD1306_OLED_WIDTH - 8 ||
    x_offset_pixels_left + 8 >= SSD1306_OLED_WIDTH - x_offset_pixels_right) {
        if (start_page >= SSD1306_NUM_PAGES ) {
            printf("Cannot pick a page number over 7! Use 0-7 for pages\n");
        } else {
            printf("Cannot pick an x left offset greater than 119! Use 0-119\n");
        }
        return false;
    }
    // get the array of pixels ready
    // printf("Trying to write string %s at internal address of page %d and column %d\n", string_to_print, page_address, x_offset);
    byte current_page = start_page;
    byte current_column = x_offset_pixels_left;
    bool page_dirty[SSD1306_NUM_PAGES] = {false};

    for (int i = 0; i < len; i++) {
        // grab 8 byte blocks for each character
        const byte* glyph = (byte*)get_bitmap_from_ascii((byte)string_to_print[i]);
        // write the 8 byte block to GDDRAM buffer
        if (current_column + 8 > SSD1306_OLED_WIDTH - x_offset_pixels_right) {
            // if the character would be printed offscreen or past the right offset, wrap to next page and reset column
            current_page++;
            if (current_page >= SSD1306_NUM_PAGES) {
                current_page = 0; // wrap to top if desired, or break to stop
            }
            current_column = x_offset_pixels_left; // move column back to offset
        }
        memcpy(&(ssd1306GDDRAM_buffer[current_page][current_column]), glyph, 8);
        page_dirty[current_page] = true;
        current_column += 8;
        // printf("copied 8 bytes into column %d and row %d of the buffer\n", page_address, x_offset + 8 * i);
    }
    // Refresh only the modified pages
    for (int p = 0; p < SSD1306_NUM_PAGES; p++) {
        if (page_dirty[p]) {
            ssd1306_refresh_page(p);
        }
    }
    return true;
}

/*
the rectangle will be placed with it's top left at the origin coordinate
width determines x distance in pixels including the origin
height determines y distance in pixels including the origin
always draws down and to the right
*/

bool ssd1306_draw_rectangle(ssd1306_pixel_coordinate origin, byte width_px, byte height_px, byte border_thickness_px, bool fill) {
    byte starting_page = origin.y / 8;
    byte vertical_bit = origin.y % 8;
    byte starting_column = origin.x;

    // assuming (0,0) we can have a max width of 128 and height of 64
    if (starting_column + width_px > SSD1306_OLED_WIDTH) {
        printf("width of rectangle too great\n");
        return false;
    }
    if ((8 * starting_page + height_px - vertical_bit > SSD1306_OLED_HEIGHT)) {
        printf("height of rectangle too great\n");
        return false;
    }
    if (border_thickness_px == 0 || width_px == 0 || height_px == 0) {
        printf("rectangle dimensions must be at least 1px\n");
        return false;
    }
    if (fill) {
        byte tracker = 0;
        byte pages_to_refresh = 1;
        for (byte y = origin.y; y < origin.y + height_px; y++) {
            if (!ssd1306_draw_hline(y, origin.x, origin.x + width_px - 1, false)) return false;
            tracker++;
            if (tracker == 8) { // 8 pixels per page
                tracker = 0;
                pages_to_refresh++;
            }
        }

        for (byte i = 0; i < pages_to_refresh; i++) {
            byte page_to_refresh = starting_page + i;
            if (page_to_refresh >= SSD1306_NUM_PAGES) break; // don't exceed 7
            if (!ssd1306_refresh_page(page_to_refresh)) return false;
        }
        return true;
    } else {
        // Draw borders of thickness border_thickness_px
        for (byte t = 0; t < border_thickness_px; t++) {
            // Top border
            ssd1306_draw_hline(origin.y + t, origin.x, origin.x + width_px - 1, false);
            // Bottom border
            ssd1306_draw_hline(origin.y + height_px - t - 1, origin.x, origin.x + width_px - 1, false);
            // Left border
            ssd1306_draw_vline(origin.x + t, origin.y, origin.y + height_px - 1, false);
            // Right border
            ssd1306_draw_vline(origin.x + width_px - t - 1, origin.y, origin.y + height_px - 1, false);
        }
    }
    // Push buffer to display (you could optimize to refresh only affected pages)
    return ssd1306_refresh_display();
}

bool ssd1306_set_pixel_xy(byte x, byte y, ON_OFF on_or_off, bool flush) {
    const ssd1306_pixel_coordinate coords = {x, y};
    return ssd1306_set_pixel(coords, on_or_off, flush);
}

/*
note: 0 indexed. (0, 0) is top left of display
the flush variable determines if the WHOLE screen is printed
*/ 
bool ssd1306_set_pixel(ssd1306_pixel_coordinate pixel_coords, ON_OFF on_or_off, bool flush) {
    
    byte column = pixel_coords.x; // 0-127
    byte page = pixel_coords.y / 8; // 0-7

    if (!ssd1306_verify_coordinates_are_valid(pixel_coords)) {
        printf("Error: cannot draw a pixel that does not fit on the display. Passed in point (%d, %d)\n", (int)column, (int)pixel_coords.y);
        return false;
    }

    // the individual pixel is one bit of the byte at the pair (column, page)
    // this byte is always written to the on chip GDDRAM top down starting at the LSB to the MSB
    byte bit = pixel_coords.y % 8; // this number is from the top down so shift from LSB towards MSB (left shift)

    // if the bit is already set, just exit the function. No work to be done.
    if (((ssd1306GDDRAM_buffer[page][column] >> bit) & 1) == on_or_off) return true;

    else if (on_or_off == OFF) {
        // set the correct bit to be 1 from 0
        // To do this, left shift a 1 into the correct position. Then invert this to get all 1s and one 0 in the correct spot.
        // Lastly, bitwise AND with the original value to set the 1 to a 0.
        ssd1306GDDRAM_buffer[page][column] &= ~(1 << bit);
    } else {
        // set the correct bit to be 0 from 1
        // In this case, just left shift a 1 to the correct spot and bitwise OR it with the original value.
        // This will set the bit of interest to a 0 and keep the others untouched
        ssd1306GDDRAM_buffer[page][column] |= (1 << bit);
        // printf("page value: %d\n", (int)page);
        // printf("column value: %d\n", (int)column);
        // printf("bit value: %d\n", (int)bit);
        // printf("Byte value is now: %d\n", (int)ssd1306GDDRAM_buffer[page][column]);
    }
    // update the screen only if specified
    if (flush) return ssd1306_refresh_page(page);
    return true;
}

// draws a line 1 pixel wide
bool ssd1306_draw_line(ssd1306_pixel_coordinate p1, ssd1306_pixel_coordinate p2, bool flush) {
    if (!ssd1306_verify_coordinates_are_valid(p1) || !ssd1306_verify_coordinates_are_valid(p2)) {
        printf("Invalid coordinates to draw line\n");
        return false;
    }

    int x1 = p1.x, y1 = p1.y;
    int x2 = p2.x, y2 = p2.y;

    if (x1 == x2) {
        return ssd1306_draw_vline(x1, y1, y2, flush);
    } else if (y1 == y2) {
        return ssd1306_draw_hline(y1, x1, x2, flush);
    }

    // Implement Bresenham's line algorithm (from wikipedia...)

    int dx = abs(x2 - x1);
    int dy = -abs(y2 - y1);
    int sx = x1 < x2 ? 1 : -1;
    int sy = y1 < y2 ? 1 : -1;
    int err = dx + dy;

    while (1) {
        if (!ssd1306_set_pixel_xy((byte)x1, (byte)y1, ON, false)) {
            printf("Failed to set pair: %d, %d\n", (int)x1, (int)y1);
            return false;
        }
        if (x1 == x2 && y1 == y2) break;

        int e2 = 2 * err;
        if (e2 >= dy) { err += dy; x1 += sx; }
        if (e2 <= dx) { err += dx; y1 += sy; }
    }

    return ssd1306_refresh_display(); // Use partial page update for better speed
}

bool ssd1306reset_page(byte page) {
    memset(ssd1306GDDRAM_buffer[page], 0x0, 128);
    return ssd1306_refresh_page(page);
}

// Horizontal line of width 1 pixel
bool ssd1306_draw_hline(byte y, byte x1, byte x2, bool flush) {
    // Ensure coordinates are valid
    if (!ssd1306_verify_coordinates_are_valid((ssd1306_pixel_coordinate){.x = x1, .y = y}) ||
        !ssd1306_verify_coordinates_are_valid((ssd1306_pixel_coordinate){.x = x2, .y = y})) {
        printf("invalid coordinates given in ssd1306_draw_hline()\n");
        return false;
    }
    byte start = (x1 < x2) ? x1 : x2;
    byte end   = (x1 > x2) ? x1 : x2;

    ssd1306_pixel_coordinate current = {.y = y}; // x will be set in the loop
    for (byte x = start; x <= end; x++) {
        current.x = x;
        if (!ssd1306_set_pixel(current, ON, false)) return false;
    }
    if (flush) {
        return ssd1306_refresh_page(y / 8);
    }
    return true;
}

// vertical line of width 1 pixel
bool ssd1306_draw_vline(byte x, byte y1, byte y2, bool flush) {
    if (!ssd1306_verify_coordinates_are_valid((ssd1306_pixel_coordinate){.x = x, .y = y1}) || 
        !ssd1306_verify_coordinates_are_valid((ssd1306_pixel_coordinate){.x = x, .y = y2})) {
        printf("invalid coordinates given in ssd1306_draw_vline()\n");
        return false;
    }
    byte start = (y1 < y2) ? y1 : y2;
    byte end   = (y1 > y2) ? y1 : y2;
    bool dirty_pages[SSD1306_NUM_PAGES] = {false};

    ssd1306_pixel_coordinate current = {.x = x, .y = 0}; // y will be set in the loop
    for (byte y = start; y <= end; y++) {
        current.y = y;
        if (!ssd1306_set_pixel(current, ON, false)) return false;
        if (dirty_pages[current.y / 8] == false) {
            dirty_pages[current.y / 8] = true;
        }
    }
    if (flush) {
        for (byte i = 0; i < SSD1306_NUM_PAGES; i++) {
            if (dirty_pages[i]) {
                ssd1306_refresh_page(i);
            }
        }
    }
    return true;
}

// faster than refreshing the whole display
bool ssd1306_refresh_page(byte page_to_refresh) {
    if (page_to_refresh >= SSD1306_NUM_PAGES) {
        printf("Page must be in range 0-%d\n", SSD1306_NUM_PAGES - 1);
        return false; // ssd1306 has 8 pages (0–7)
    }
    if (current_mode != PAGE) {
        if (!ssd1306_set_addressing_mode(PAGE)) return false;
    }
    // set the column to 0 and page to the desired page. Then transmit data from the GDDRAM buffer

    /*
    could optimize this by making this one transmission without two START/STOP conditions
    */
    // ssd1306_set_page_address(page_to_refresh);
    // ssd1306_set_column_address(0);

    byte transmission[] = {SSD1306_CONTROL_BYTE(1, 0), 0xB0 | page_to_refresh,SSD1306_CONTROL_BYTE(0, 0), 0, 0x10};
    if (!ssd1306_write_bytes(transmission, sizeof(transmission), true, true)) return false;

    byte buffer[129];
    buffer[0] =SSD1306_CONTROL_BYTE(0, 1); // data control byte
    memcpy(&buffer[1], &ssd1306GDDRAM_buffer[page_to_refresh][0], SSD1306_OLED_WIDTH);
    if (!ssd1306_write_bytes(buffer, sizeof(buffer), true, true)) return false;
    return true;
}

bool ssd_1306_verify_coordinates_are_valid(ssd1306_pixel_coordinate coordinate) {
    if (coordinate.x >= SSD1306_OLED_WIDTH || coordinate.y >= SSD1306_OLED_HEIGHT) {
        return false;
    }
    return true;
}

/*  FONT DEFINED FROM ASCII 32 -> 127 */
static const byte font8x8[95][8] = {
    /*  ASCII PRINTABLE CHARACTERS */
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, //  032 -> 0x20 ( ) (space)
    {0x00, 0x00, 0x00, 0x5F, 0x5F, 0x00, 0x00, 0x00}, //  033 -> 0x21 (!)
    {0x00, 0x07, 0x07, 0x00, 0x07, 0x07, 0x00, 0x00}, //  034 -> 0x22 (")
    {0x14, 0x7F, 0x7F, 0x14, 0x7F, 0x7F, 0x14, 0x00}, //  035 -> 0x23 (#)
    {0x00, 0x24, 0x2A, 0x7F, 0x7F, 0x2A, 0x12, 0x00}, //  036 -> 0x24 ($)
    {0x46, 0x66, 0x30, 0x18, 0x0C, 0x66, 0x62, 0x00}, //  037 -> 0x25 (%)
    {0x30, 0x7A, 0x4F, 0x5D, 0x37, 0x7A, 0x48, 0x00}, //  038 -> 0x26 (&)
    {0x00, 0x00, 0x00, 0x07, 0x07, 0x00, 0x00, 0x00}, //  039 -> 0x27 (')
    {0x00, 0x00, 0x1C, 0x3E, 0x63, 0x41, 0x00, 0x00}, //  040 -> 0x28 (()
    {0x00, 0x00, 0x41, 0x63, 0x3E, 0x1C, 0x00, 0x00}, //  041 -> 0x29 ())
    {0x08, 0x2A, 0x3E, 0x1C, 0x1C, 0x3E, 0x2A, 0x08}, //  042 -> 0x2a (*)
    {0x00, 0x08, 0x08, 0x3E, 0x3E, 0x08, 0x08, 0x00}, //  043 -> 0x2b (+)
    {0x00, 0x00, 0x80, 0xE0, 0x60, 0x00, 0x00, 0x00}, //  044 -> 0x2c (,)
    {0x00, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x00}, //  045 -> 0x2d (-)
    {0x00, 0x00, 0x00, 0x60, 0x60, 0x00, 0x00, 0x00}, //  046 -> 0x2e (.)
    {0x60, 0x30, 0x18, 0x0C, 0x06, 0x03, 0x01, 0x00}, //  047 -> 0x2f (/)
    {0x3E, 0x7F, 0x51, 0x49, 0x45, 0x7F, 0x3E, 0x00}, //  048 -> 0x30 (0)
    {0x00, 0x40, 0x42, 0x7F, 0x7F, 0x40, 0x40, 0x00}, //  049 -> 0x31 (1)
    {0x00, 0x72, 0x7B, 0x49, 0x49, 0x6F, 0x66, 0x00}, //  050 -> 0x32 (2)
    {0x00, 0x22, 0x63, 0x49, 0x49, 0x7F, 0x36, 0x00}, //  051 -> 0x33 (3)
    {0x18, 0x1C, 0x16, 0x53, 0x7F, 0x7F, 0x50, 0x00}, //  052 -> 0x34 (4)
    {0x00, 0x2F, 0x6F, 0x49, 0x49, 0x79, 0x33, 0x00}, //  053 -> 0x35 (5)
    {0x00, 0x3E, 0x7F, 0x49, 0x49, 0x7B, 0x32, 0x00}, //  054 -> 0x36 (6)
    {0x00, 0x03, 0x03, 0x71, 0x79, 0x0F, 0x07, 0x00}, //  055 -> 0x37 (7)
    {0x00, 0x36, 0x7F, 0x49, 0x49, 0x7F, 0x36, 0x00}, //  056 -> 0x38 (8)
    {0x00, 0x26, 0x6F, 0x49, 0x49, 0x7F, 0x3E, 0x00}, //  057 -> 0x39 (9)
    {0x00, 0x00, 0x00, 0x6C, 0x6C, 0x00, 0x00, 0x00}, //  058 -> 0x3a (:)
    {0x00, 0x00, 0x80, 0xEC, 0x6C, 0x00, 0x00, 0x00}, //  059 -> 0x3b (;)
    {0x00, 0x08, 0x1C, 0x36, 0x63, 0x41, 0x00, 0x00}, //  060 -> 0x3c (<)
    {0x00, 0x24, 0x24, 0x24, 0x24, 0x24, 0x24, 0x00}, //  061 -> 0x3d (=)
    {0x00, 0x41, 0x63, 0x36, 0x1C, 0x08, 0x00, 0x00}, //  062 -> 0x3e (>)
    {0x00, 0x06, 0x07, 0x51, 0x59, 0x0F, 0x06, 0x00}, //  063 -> 0x3f (?)
    {0x3E, 0x7F, 0x41, 0x5D, 0x5D, 0x5F, 0x1E, 0x00}, //  064 -> 0x40 (@)
    {0x00, 0x7C, 0x7E, 0x13, 0x13, 0x7E, 0x7C, 0x00}, //  065 -> 0x41 (A)
    {0x41, 0x7F, 0x7F, 0x49, 0x49, 0x7F, 0x36, 0x00}, //  066 -> 0x42 (B)
    {0x1C, 0x3E, 0x63, 0x41, 0x41, 0x63, 0x22, 0x00}, //  067 -> 0x43 (C)
    {0x41, 0x7F, 0x7F, 0x41, 0x63, 0x3E, 0x1C, 0x00}, //  068 -> 0x44 (D)
    {0x41, 0x7F, 0x7F, 0x49, 0x5D, 0x41, 0x63, 0x00}, //  069 -> 0x45 (E)
    {0x41, 0x7F, 0x7F, 0x49, 0x1D, 0x01, 0x03, 0x00}, //  070 -> 0x46 (F)
    {0x1C, 0x3E, 0x63, 0x41, 0x51, 0x73, 0x72, 0x00}, //  071 -> 0x47 (G)
    {0x00, 0x7F, 0x7F, 0x08, 0x08, 0x7F, 0x7F, 0x00}, //  072 -> 0x48 (H)
    {0x00, 0x41, 0x41, 0x7F, 0x7F, 0x41, 0x41, 0x00}, //  073 -> 0x49 (I)
    {0x30, 0x70, 0x40, 0x41, 0x7F, 0x3F, 0x01, 0x00}, //  074 -> 0x4a (J)
    {0x41, 0x7F, 0x7F, 0x08, 0x1C, 0x77, 0x63, 0x00}, //  075 -> 0x4b (K)
    {0x41, 0x7F, 0x7F, 0x41, 0x40, 0x60, 0x70, 0x00}, //  076 -> 0x4c (L)
    {0x7F, 0x7F, 0x0E, 0x1C, 0x0E, 0x7F, 0x7F, 0x00}, //  077 -> 0x4d (M)
    {0x7F, 0x7F, 0x06, 0x0C, 0x18, 0x7F, 0x7F, 0x00}, //  078 -> 0x4e (N)
    {0x1C, 0x3E, 0x63, 0x41, 0x63, 0x3E, 0x1C, 0x00}, //  079 -> 0x4f (O)
    {0x41, 0x7F, 0x7F, 0x49, 0x09, 0x0F, 0x06, 0x00}, //  080 -> 0x50 (P)
    {0x3C, 0x7E, 0x43, 0x51, 0x33, 0x6E, 0x5C, 0x00}, //  081 -> 0x51 (Q)
    {0x41, 0x7F, 0x7F, 0x09, 0x19, 0x7F, 0x66, 0x00}, //  082 -> 0x52 (R)
    {0x00, 0x26, 0x6F, 0x49, 0x49, 0x7B, 0x32, 0x00}, //  083 -> 0x53 (S)
    {0x00, 0x03, 0x41, 0x7F, 0x7F, 0x41, 0x03, 0x00}, //  084 -> 0x54 (T)
    {0x00, 0x3F, 0x7F, 0x40, 0x40, 0x7F, 0x3F, 0x00}, //  085 -> 0x55 (U)
    {0x00, 0x1F, 0x3F, 0x60, 0x60, 0x3F, 0x1F, 0x00}, //  086 -> 0x56 (V)
    {0x7F, 0x7F, 0x30, 0x18, 0x30, 0x7F, 0x7F, 0x00}, //  087 -> 0x57 (W)
    {0x61, 0x73, 0x1E, 0x0C, 0x1E, 0x73, 0x61, 0x00}, //  088 -> 0x58 (X)
    {0x00, 0x07, 0x4F, 0x78, 0x78, 0x4F, 0x07, 0x00}, //  089 -> 0x59 (Y)
    {0x47, 0x63, 0x71, 0x59, 0x4D, 0x67, 0x73, 0x00}, //  090 -> 0x5a (Z)
    {0x00, 0x00, 0x7F, 0x7F, 0x41, 0x41, 0x00, 0x00}, //  091 -> 0x5b ([)
    {0x01, 0x03, 0x06, 0x0C, 0x18, 0x30, 0x60, 0x00}, //  092 -> 0x5c (\)
    {0x00, 0x00, 0x41, 0x41, 0x7F, 0x7F, 0x00, 0x00}, //  093 -> 0x5d (])
    {0x08, 0x0C, 0x06, 0x03, 0x06, 0x0C, 0x08, 0x00}, //  094 -> 0x5e (^)
    {0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80}, //  095 -> 0x5f (_)
    {0x00, 0x00, 0x01, 0x03, 0x06, 0x04, 0x00, 0x00}, //  096 -> 0x60 (`)
    {0x20, 0x74, 0x54, 0x54, 0x3C, 0x78, 0x40, 0x00}, //  097 -> 0x61 (a)
    {0x41, 0x7F, 0x3F, 0x44, 0x44, 0x7C, 0x38, 0x00}, //  098 -> 0x62 (b)
    {0x00, 0x38, 0x7C, 0x44, 0x44, 0x6C, 0x28, 0x00}, //  099 -> 0x63 (c)
    {0x38, 0x7C, 0x44, 0x45, 0x3F, 0x7F, 0x40, 0x00}, //  100 -> 0x64 (d)
    {0x00, 0x38, 0x7C, 0x54, 0x54, 0x5C, 0x18, 0x00}, //  101 -> 0x65 (e)
    {0x00, 0x48, 0x7E, 0x7F, 0x49, 0x03, 0x02, 0x00}, //  102 -> 0x66 (f)
    {0x00, 0x98, 0xBC, 0xA4, 0xA4, 0xFC, 0x7C, 0x00}, //  103 -> 0x67 (g)
    {0x41, 0x7F, 0x7F, 0x08, 0x04, 0x7C, 0x78, 0x00}, //  104 -> 0x68 (h)
    {0x00, 0x00, 0x44, 0x7D, 0x7D, 0x40, 0x00, 0x00}, //  105 -> 0x69 (i)
    {0x00, 0x60, 0xE0, 0x80, 0x84, 0xFD, 0x7D, 0x00}, //  106 -> 0x6a (j)
    {0x41, 0x7F, 0x7F, 0x10, 0x38, 0x6C, 0x44, 0x00}, //  107 -> 0x6b (k)
    {0x00, 0x00, 0x41, 0x7F, 0x7F, 0x40, 0x00, 0x00}, //  108 -> 0x6c (l)
    {0x78, 0x7C, 0x0C, 0x38, 0x0C, 0x7C, 0x78, 0x00}, //  109 -> 0x6d (m)
    {0x04, 0x7C, 0x78, 0x04, 0x04, 0x7C, 0x78, 0x00}, //  110 -> 0x6e (n)
    {0x00, 0x38, 0x7C, 0x44, 0x44, 0x7C, 0x38, 0x00}, //  111 -> 0x6f (o)
    {0x84, 0xFC, 0xF8, 0xA4, 0x24, 0x3C, 0x18, 0x00}, //  112 -> 0x70 (p)
    {0x18, 0x3C, 0x24, 0xA4, 0xF8, 0xFC, 0x84, 0x00}, //  113 -> 0x71 (q)
    {0x44, 0x7C, 0x78, 0x4C, 0x04, 0x0C, 0x08, 0x00}, //  114 -> 0x72 (r)
    {0x00, 0x48, 0x5C, 0x54, 0x54, 0x74, 0x20, 0x00}, //  115 -> 0x73 (s)
    {0x00, 0x04, 0x3F, 0x7F, 0x44, 0x64, 0x20, 0x00}, //  116 -> 0x74 (t)
    {0x00, 0x3C, 0x7C, 0x40, 0x40, 0x7C, 0x7C, 0x00}, //  117 -> 0x75 (u)
    {0x00, 0x1C, 0x3C, 0x60, 0x60, 0x3C, 0x1C, 0x00}, //  118 -> 0x76 (v)
    {0x3C, 0x7C, 0x60, 0x38, 0x60, 0x7C, 0x3C, 0x00}, //  119 -> 0x77 (w)
    {0x44, 0x6C, 0x38, 0x10, 0x38, 0x6C, 0x44, 0x00}, //  120 -> 0x78 (x)
    {0x00, 0x9C, 0xBC, 0xA0, 0xA0, 0xFC, 0x7C, 0x00}, //  121 -> 0x79 (y)
    {0x00, 0x4C, 0x64, 0x74, 0x5C, 0x4C, 0x64, 0x00}, //  122 -> 0x7a (z)
    {0x00, 0x08, 0x08, 0x3E, 0x77, 0x41, 0x41, 0x00}, //  123 -> 0x7b ({)
    {0x00, 0x00, 0x00, 0x7F, 0x7F, 0x00, 0x00, 0x00}, //  124 -> 0x7c (|)
    {0x00, 0x41, 0x41, 0x77, 0x3E, 0x08, 0x08, 0x00}, //  125 -> 0x7d (})
    {0x10, 0x18, 0x08, 0x18, 0x10, 0x18, 0x08, 0x00}, //  126 -> 0x7e (~)
};

static byte* get_bitmap_from_ascii(byte character) {
    if (character < 32 || character > 126) {
        // if invalid, don't set any pixels (will look like nothing or a space)
        return (byte*)font8x8[0]; // return all 0s
    }
    return (byte*)font8x8[character - 32];
}

// Send a single command byte
static inline bool ssd1306_write_command(byte cmd) {
    byte tx[2] = {0x00, cmd};  // 0x00 = control byte for commands
    return ssd1306_write_bytes(tx, 2, true, true);
}

/*
Send two bytes as a single command sequence
many commands for the ssd1306 use one for the command itself, and one for actual command data
*/
static inline bool ssd1306_write_command2(byte command_code, byte command_argument) {
    byte tx[3] = {0x00, command_code, command_argument};
    return ssd1306_write_bytes(tx, 3, true, true);
}
