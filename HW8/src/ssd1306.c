// based on adafruit and sparkfun libraries

#include <string.h> // for memset
#include <stdio.h>
#include <xc.h> // for the core timer delay

#include "ssd1306.h"
#include "font.h"

unsigned char ssd1306_write = 0b01111000; // i2c address
unsigned char ssd1306_read = 0b01111001;  // i2c address
unsigned char ssd1306_buffer[512];  // 128x32/8. Every bit is a pixel

void ssd1306_setup() {
    // give a little delay for the ssd1306 to power up
    _CP0_SET_COUNT(0);
    while (_CP0_GET_COUNT() < 48000000 / 2 / 50) {
    }
    ssd1306_command(SSD1306_DISPLAYOFF);
    ssd1306_command(SSD1306_SETDISPLAYCLOCKDIV);
    ssd1306_command(0x80);
    ssd1306_command(SSD1306_SETMULTIPLEX);
    ssd1306_command(0x1F); // height-1 = 31
    ssd1306_command(SSD1306_SETDISPLAYOFFSET);
    ssd1306_command(0x0);
    ssd1306_command(SSD1306_SETSTARTLINE);
    ssd1306_command(SSD1306_CHARGEPUMP);
    ssd1306_command(0x14);
    ssd1306_command(SSD1306_MEMORYMODE);
    ssd1306_command(0x00);
    ssd1306_command(SSD1306_SEGREMAP | 0x1);
    ssd1306_command(SSD1306_COMSCANDEC);
    ssd1306_command(SSD1306_SETCOMPINS);
    ssd1306_command(0x02);
    ssd1306_command(SSD1306_SETCONTRAST);
    ssd1306_command(0x8F);
    ssd1306_command(SSD1306_SETPRECHARGE);
    ssd1306_command(0xF1);
    ssd1306_command(SSD1306_SETVCOMDETECT);
    ssd1306_command(0x40);
    ssd1306_command(SSD1306_DISPLAYON);
    ssd1306_clear();
    ssd1306_update();
}

// send a command instruction (not pixel data)
void ssd1306_command(unsigned char c) {
    i2c_master_start();
    i2c_master_send(ssd1306_write);
    i2c_master_send(0x00); // bit 7 is 0 for Co bit (data bytes only), bit 6 is 0 for DC (data is a command))
    i2c_master_send(c);
    i2c_master_stop();
}

// update every pixel on the screen
void ssd1306_update() {
    ssd1306_command(SSD1306_PAGEADDR);
    ssd1306_command(0);
    ssd1306_command(0xFF);
    ssd1306_command(SSD1306_COLUMNADDR);
    ssd1306_command(0);
    ssd1306_command(128 - 1); // Width

    unsigned short count = 512; // WIDTH * ((HEIGHT + 7) / 8)
    unsigned char * ptr = ssd1306_buffer; // first address of the pixel buffer
    i2c_master_start();
    i2c_master_send(ssd1306_write);
    i2c_master_send(0x40); // send pixel data
    // send every pixel
    while (count--) {
        i2c_master_send(*ptr++);
    }
    i2c_master_stop();
}

// set a pixel value. Call update() to push to the display)
void ssd1306_drawPixel(unsigned char x, unsigned char y, unsigned char color) {
    if ((x < 0) || (x >= 128) || (y < 0) || (y >= 32)) {
        return;
    }

    if (color == 1) {
        ssd1306_buffer[x + (y / 8)*128] |= (1 << (y & 7));
    } else {
        ssd1306_buffer[x + (y / 8)*128] &= ~(1 << (y & 7));
    }
}

// zero every pixel value
void ssd1306_clear() {
    memset(ssd1306_buffer, 0, 512); // make every bit a 0, memset in string.h
}


// Draw a character
void ssd1306_drawChar(unsigned char x, unsigned char y, unsigned char c) {
    unsigned short i, j;
    for (i=0; i<5; i++) {
        for (j=0; j<8; j++) {
            ssd1306_drawPixel(x+i, y+j, (ASCII[c - 0x20][i] >> j) & 1);
        }
    }
}

const unsigned short CHAR_SPACING = 5;

void ssd1306_drawMessage(unsigned char x, unsigned char y, char* msg) {
    unsigned short i, j;
    for (i=0; i<strlen(msg); i++) {
        ssd1306_drawChar(x + i*CHAR_SPACING, y, msg[i]);
    }
}

void ssd1306_drawAccMap(short x, short y) {
    short i, j;
    short x_low, x_high, y_low, y_high;
    // Calculate x and y bar range
    x += SSD1306_MID_X;
    y += SSD1306_MID_Y;
    if (x >= SSD1306_MID_X) {
        x_low = SSD1306_MID_X;
        x_high = x;
    } else {
        x_low = x;
        x_high = SSD1306_MID_X;
    }
    if (y >= SSD1306_MID_Y) {
        y_low = SSD1306_MID_Y;
        y_high = y;
    } else {
        y_low = y;
        y_high = SSD1306_MID_Y;
    }

    // Draw x bar
    for (i=x_low; i<=x_high; ++i) {
        ssd1306_drawPixel(i, SSD1306_MID_Y, 1);
    }

    // Draw y bar
    for (i=y_low; i<=y_high; ++i) {
        ssd1306_drawPixel(SSD1306_MID_X, i, 1);
    }
}
