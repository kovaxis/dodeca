
#include "screen.h"
#include "bitmaps.h"

#include "T4K/T4K_common.h"
extern SSD1306Device oled;

#define CHAR_COUNT 11

typedef struct
{
    // Dimensions of each font character
    byte width;
    byte height;
    // 5 different positions, each an X/Y pair of pixel positions for the top-left corner of a
    // character
    byte positions[10];
    // Pointer to character data in PROGMEM
    // Each character is the same length and size, and consists of a series of bits, in column
    // first order, from top to bottom.
    // There are 2*CHAR_COUNT characters in here (twice the amount, because each character has a
    // 90 degree rotated form).
    const byte *data;
} Font;

typedef struct
{
    // Dimensions of the sprite.
    byte width;
    byte height;
    // Default position of the sprite.
    byte default_x;
    byte default_y;
    // Pointer to sprite data in PROGMEM
    const byte *data;
} Sprite;

const PROGMEM Font FONT_00 = {
    // Width, height
    8,
    12,
    {
        // Position 0
        3,
        18,
        // Position 1
        13,
        18,
        // Position 2
        20,
        18,
        // Position 3
        27,
        18,
        // Position 4
        37,
        18,
    },
    // Data
    // Characters: "0123456789:" (11*2 characters)
    // Character dimensions: 8x12 (12 bytes each)
    FONT_00_DATA,
};

const PROGMEM Font FONT_45 = {
    // Width, height
    14,
    14,
    {
        // Position 0
        5,
        29,
        // Position 1
        12,
        22,
        // Position 2
        17,
        17,
        // Position 3
        23,
        11,
        // Position 4
        30,
        4,
    },
    // Data
    // Characters: "0123456789:" (11*2 characters)
    // Character dimensions: 14x14 (25 bytes each)
    FONT_45_DATA,
};

const PROGMEM Sprite LOWBAT_SPRITE = {
    29,
    16,
    10,
    16,
    LOWBAT_DATA,
};

const PROGMEM Sprite FULLBAT_SPRITE = {
    29,
    16,
    10,
    16,
    FULLBAT_DATA,
};

const PROGMEM Sprite CHARGING_SPRITE = {
    29,
    16,
    10,
    16,
    CHARGING_DATA,
};

static byte buffer_mem[2 * SCRBUF_WIDTH * SCRBUF_PAGES];
static byte *front_buffer = buffer_mem;

void scr_clear()
{
    for (int i = 0; i < SCRBUF_WIDTH * SCRBUF_PAGES; i += 1)
    {
        front_buffer[i] = 0;
    }
}

void draw_upright(int width, int height, int base_x, int base_y, const byte *data)
{
    //Write data
    int min_page = base_y / 8;
    int page_offset = base_y % 8;
    int max_page = (base_y + height + 7) / 8 - 1;
    for (int page = min_page; page <= max_page; page += 1)
    {
        byte mask = 0xff;
        if (page == min_page)
        {
            mask <<= page_offset;
        }
        else if (page == max_page)
        {
            mask >>= (8 - (base_y + height) % 8) % 8;
        }
        for (int dx = 0; dx < width; dx += 1)
        {
            int offset = (page - min_page) * 8 - page_offset + dx * height;
            int byte_offset = (offset + 8) / 8 - 1;
            int bit_offset = (offset + 8) % 8;
            byte page_col = data[byte_offset] >> bit_offset;
            if (bit_offset > 0)
            {
                page_col |= data[byte_offset + 1] << (8 - bit_offset);
            }
            front_buffer[page * SCRBUF_WIDTH + base_x + dx] |= page_col & mask;
        }
    }
}

void draw_upsidedown(int width, int height, int base_x, int base_y, const byte *data)
{
    //Write data
    int min_page = base_y / 8;
    int page_offset = base_y % 8;
    int max_page = (base_y + height + 7) / 8 - 1;
    for (int page = min_page; page <= max_page; page += 1)
    {
        byte mask = 0xff;
        if (page == min_page)
        {
            mask <<= page_offset;
        }
        else if (page == max_page)
        {
            mask >>= (8 - (base_y + height) % 8) % 8;
        }
        for (int dx = 0; dx < width; dx += 1)
        {
            int offset = width * height + page_offset - (page - min_page) * 8 - dx * height;
            int byte_offset = (offset + 8) / 8 - 1;
            int bit_offset = (offset + 8) % 8;
            byte page_col = data[byte_offset - 1] >> bit_offset;
            if (bit_offset > 0)
            {
                page_col |= data[byte_offset] << (8 - bit_offset);
            }
            page_col = reverse_byte(page_col);
            front_buffer[page * SCRBUF_WIDTH + base_x + dx] |= page_col & mask;
        }
    }
}

void scr_draw_bat_sprite(BatStatus bat_status)
{
    const Sprite* src_sprite;
    switch (bat_status) {
        case BAT_LOW:
            src_sprite = &LOWBAT_SPRITE;
            break;
        case BAT_CHARGING:
            src_sprite = &CHARGING_SPRITE;
            break;
        case BAT_CHARGED:
            src_sprite = &FULLBAT_SPRITE;
            break;
#ifdef DEBUG_SERIAL
        default:
            Serial.print(F("ERROR: attempting to draw bat_status="));
            Serial.println(bat_status, HEX);
#endif
    }

    Sprite sprite;
    byte data[MAX_SPRITE_SIZE];
    memcpy_P(&sprite, src_sprite, sizeof(Sprite));
    memcpy_P(data, sprite.data, (((int)sprite.width) * sprite.height + 7) / 8);
    draw_upright(sprite.width, sprite.height, sprite.default_x, sprite.default_y, data);
}

void scr_draw(int orient, int pos, int ch)
{
    //Pick rotation based on `orient`
    const Font *font_ptr;
    int rotate;
    switch (orient)
    {
    case ORIENT_000:
        font_ptr = &FONT_00;
        rotate = 0;
        break;
    case ORIENT_072:
        font_ptr = &FONT_00;
        rotate = 1;
        break;
    case ORIENT_144:
        font_ptr = &FONT_45;
        rotate = 2;
        break;
    case ORIENT_216:
        font_ptr = &FONT_45;
        rotate = 3;
        break;
    case ORIENT_288:
        font_ptr = &FONT_00;
        rotate = 3;
        break;
    default:
        return;
    }
    Font font;
    memcpy_P(&font, font_ptr, sizeof(Font));

    //Get position
    int base_x = font.positions[pos * 2];
    int base_y = font.positions[pos * 2 + 1];

    //Get character data
#define MAX_CAP 64
    byte data[MAX_CAP];
    int stride = (((int)font.width) * font.height + 7) / 8;
    if (stride > MAX_CAP)
    {
#ifdef DEBUG_SERIAL
        Serial.println(F("Font size overflows char data capacity!"));
#endif
        for (;;)
        {
        }
    }
#undef MAX_CAP
    int char_idx;
    int width;
    int height;
    int x;
    int y;
    if ((rotate & 0x1) == 0)
    {
        //Do not rotate 90 degrees
        width = font.width;
        height = font.height;
        x = base_x;
        y = base_y;
        char_idx = ch;
    }
    else
    {
        //Rotate 90 degrees
        width = font.height;
        height = font.width;
        x = SCRBUF_WIDTH / 2 - (base_y - SCRBUF_PAGES * 4) - width;
        y = SCRBUF_PAGES * 4 + (base_x - SCRBUF_WIDTH / 2);
        char_idx = ch + CHAR_COUNT;
    }
    memcpy_P(&data, font.data + char_idx * stride, stride);

    //Draw differently depending on orientation
    if ((rotate & 0x2) == 0)
    {
        draw_upright(width, height, x, y, data);
    }
    else
    {
        draw_upsidedown(width, height, SCRBUF_WIDTH - x - width, SCRBUF_PAGES * 8 - y - height, data);
    }
}

void scr_show()
{
    //Find backbuffer
    byte *backbuf;
    if (front_buffer == buffer_mem)
    {
        backbuf = buffer_mem + SCRBUF_WIDTH * SCRBUF_PAGES;
    }
    else
    {
        backbuf = buffer_mem;
    }
    //Update OLED
    for (int page = 0; page < SCRBUF_PAGES; page += 1)
    {
        //Find the last modified column
        int modified_up_to = 0;
        for (int x = SCRBUF_WIDTH - 1; x >= 0; x -= 1)
        {
            int idx = page * SCRBUF_WIDTH + x;
            if (front_buffer[idx] != backbuf[idx])
            {
                modified_up_to = x + 1;
                break;
            }
        }
        bool started = false;
        for (int x = 0; x < modified_up_to; x += 1)
        {
            int idx = page * SCRBUF_WIDTH + x;
            byte page_col = front_buffer[idx];
            if (!started)
            {
                if (page_col == backbuf[idx])
                {
                    continue;
                }
                oled.setCursor(SCRBUF_X + x, SCRBUF_PAGEY + page);
                oled.startData();
                started = true;
            }
            oled.sendData(page_col);
        }
        if (started)
        {
            oled.endData();
        }
    }

    //Swap buffer
    front_buffer = backbuf;
}

void scr_force_swap()
{
    //Find backbuffer
    byte *backbuf;
    if (front_buffer == buffer_mem)
    {
        backbuf = buffer_mem + SCRBUF_WIDTH * SCRBUF_PAGES;
    }
    else
    {
        backbuf = buffer_mem;
    }

    //Swap buffer
    front_buffer = backbuf;
}