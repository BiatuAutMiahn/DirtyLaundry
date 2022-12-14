#include <pgmspace.h>
#include <stdint.h>

#define I_BATTERY_WIDTH 16
#define I_BATTERY_HEIGHT 10
const unsigned char i_battery[20] PROGMEM = {
    0xFF, 0xFC, 0x80, 0x02, 0x80, 0x02, 0x80, 0x03, 0x80, 0x03, 0x80, 0x03, 0x80, 0x03, 0x80, 0x02, 0x80, 0x02, 0xFF, 0xFC};

const unsigned char i_charge[4] PROGMEM = {
    0x30, 0x36, 0xC0, 0xD8};

#define I_LIGHTNING_WIDTH 8
#define I_LIGHTNING_HEIGHT 10
const unsigned char i_lightning[10] PROGMEM = {
    0x04, 0x08, 0x18, 0x30, 0x1C, 0x0C, 0x08, 0x18, 0x30, 0x20};

#define I_EXCLAIM_WIDTH 8
#define I_EXCLAIM_HEIGHT 10
const unsigned char i_exclaim[10] PROGMEM = {
    0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x00, 0x18, 0x18};

#define I_SMGLYPH_WIDTH 16
#define I_SMGLYPH_HEIGHT 16
const unsigned char i_smglyph[32] PROGMEM = {0x00, 0x80, 0x01, 0x00, 0x02, 0x00, 0x04, 0x00, 0x0f, 0xe0, 0x18, 0x10, 0x28, 0x10, 0x48, 0x11, 0x88, 0x12, 0x08, 0x14, 0x08, 0x18, 0x07, 0xf0, 0x00, 0x20, 0x00, 0x40, 0x00, 0x80, 0x01, 0x00};

#define I_SIGNAL_WIDTH 8
#define I_SIGNAL_HEIGHT 8
const unsigned char i_signal[4][8] PROGMEM = {{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80}, {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0xa0}, {0x00, 0x00, 0x00, 0x00, 0x08, 0x08, 0x28, 0xa8}, {0x00, 0x00, 0x02, 0x02, 0x0a, 0x0a, 0x2a, 0xaa}};

//0,8
#define I_TEXT1_WIDTH 56
#define I_TEXT1_HEIGHT 10
const unsigned char i_text1[70] PROGMEM = {0x00, 0x00, 0x71, 0x80, 0x18, 0x40, 0x00, 0x1f, 0x00, 0x80, 0x00, 0x00, 0x40, 0x00, 0x04, 0x58, 0x87, 0x16, 0x71, 0xf8, 0x80, 0x04, 0x6b, 0xe1, 0x1a, 0x10, 0x88, 0x80, 0x08, 0x48, 0x81, 0x12, 0x10, 0x85, 0x00, 0x08, 0x49, 0x02, 0x12, 0x20, 0x85, 0x00, 0x08, 0x91, 0x02, 0x24, 0x20, 0x86, 0x0c, 0x3e, 0x91, 0x03, 0xa4, 0x38, 0xe4, 0x0c, 0x00, 0x01, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x06, 0x00, 0x00, 0x00, 0x18, 0x00};

//2,20
#define I_TEXT2_WIDTH 80
#define I_TEXT2_HEIGHT 39
const unsigned char i_text2[390] PROGMEM = {0x00, 0x00, 0x00, 0x00, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfe, 0x07, 0x00, 0x00, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0x07, 0x00, 0x00, 0x38, 0x00, 0x00, 0x00, 0x00, 0x01, 0xc7, 0x80, 0x00, 0x00, 0x38, 0x00, 0x00, 0x00, 0x00, 0x01, 0xc3, 0x9e, 0x02, 0x79, 0xff, 0x38, 0x70, 0x00, 0x00, 0x01, 0x83, 0xbe, 0x06, 0xfd, 0xff, 0x38, 0x70, 0x00, 0x00, 0x01, 0x83, 0xa6, 0x07, 0x9c, 0x30, 0x38, 0x60, 0x00, 0x00, 0x03, 0x83, 0x86, 0x07, 0x1c, 0x30, 0x18, 0xe0, 0x00, 0x00, 0x03, 0x83, 0x86, 0x07, 0x00, 0x70, 0x18, 0xe0, 0x00, 0x00, 0x03, 0x07, 0x0e, 0x06, 0x00, 0x70, 0x19, 0xc0, 0x00, 0x00, 0x03, 0x0f, 0x0e, 0x06, 0x00, 0x70, 0x19, 0x80, 0x00, 0x00, 0x03, 0x1e, 0x0e, 0x4e, 0x00, 0x70, 0x1b, 0x80, 0x00, 0x00, 0x07, 0xfc, 0x0f, 0xce, 0x00, 0x7c, 0x1f, 0x00, 0x00, 0x00, 0x07, 0xf0, 0x07, 0x8e, 0x00, 0x3c, 0x0e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x0c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x0c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x0c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x00, 0x00, 0x00, 0x1c, 0x00, 0xfe, 0x61, 0x8d, 0xe0, 0x7f, 0x13, 0xce, 0x1c, 0x1c, 0x01, 0xfc, 0x61, 0x9b, 0xe0, 0xfe, 0x37, 0xee, 0x1c, 0x1c, 0x03, 0x8c, 0x63, 0x9e, 0x61, 0xc6, 0x3c, 0xee, 0x18, 0x1c, 0x03, 0x8c, 0x63, 0x9c, 0x61, 0xc6, 0x38, 0xe6, 0x38, 0x18, 0x07, 0x0c, 0xe3, 0x1c, 0x63, 0x8e, 0x38, 0x06, 0x38, 0x18, 0x07, 0x1c, 0xe3, 0x18, 0x63, 0x8e, 0x30, 0x06, 0x70, 0x18, 0x07, 0x1c, 0xe7, 0x18, 0xe3, 0x8c, 0x30, 0x06, 0x60, 0x38, 0x07, 0x38, 0xef, 0x38, 0xe3, 0x9c, 0x70, 0x06, 0xe0, 0x3f, 0xc7, 0xf8, 0xff, 0x38, 0xe3, 0xfc, 0x70, 0x07, 0xc0, 0x3f, 0xc3, 0xd8, 0x73, 0x30, 0xc1, 0xec, 0x70, 0x03, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x38, 0x00};

//82,9
#define I_WASHER_WIDTH 40
#define I_WASHER_HEIGHT 48
const unsigned char i_washer[240] PROGMEM = {0x1f, 0xff, 0xff, 0xff, 0xf8, 0x20, 0x02, 0x00, 0x00, 0x04, 0x40, 0x02, 0x1c, 0x11, 0x12, 0x40, 0x02, 0x2a, 0x2a, 0xaa, 0x4f, 0xf2, 0x22, 0x11, 0x12, 0x50, 0x0a, 0x22, 0x00, 0x02, 0x40, 0x02, 0x1c, 0x00, 0x02, 0x40, 0x02, 0x00, 0x00, 0x02, 0x7f, 0xff, 0xff, 0xff, 0xfe, 0x40, 0x00, 0x00, 0x00, 0x02, 0x40, 0x00, 0x00, 0x00, 0x02, 0x40, 0x00, 0x00, 0x00, 0x02, 0x40, 0x00, 0x00, 0x00, 0x02, 0x40, 0x00, 0xff, 0x00, 0x02, 0x40, 0x07, 0x00, 0xe0, 0x02, 0x40, 0x08, 0xff, 0x10, 0x02, 0x40, 0x33, 0x00, 0xcc, 0x02, 0x40, 0x4c, 0x00, 0x32, 0x02, 0x40, 0x90, 0x00, 0x09, 0x02, 0x40, 0xa0, 0x00, 0x05, 0x02, 0x41, 0x20, 0x00, 0x02, 0x82, 0x42, 0x20, 0x00, 0x02, 0x42, 0x42, 0x10, 0x00, 0x01, 0x42, 0x42, 0x08, 0x00, 0x01, 0x42, 0x44, 0x08, 0x00, 0x00, 0xa2, 0x44, 0x04, 0x00, 0x00, 0xa2, 0x44, 0x04, 0x00, 0x00, 0xa2, 0x44, 0x04, 0x00, 0x00, 0xa2, 0x44, 0x04, 0x00, 0x00, 0xa2, 0x44, 0x04, 0x00, 0x00, 0xa2, 0x44, 0x04, 0x00, 0x00, 0xa2, 0x44, 0x08, 0x00, 0x00, 0xa2, 0x42, 0x08, 0x00, 0x01, 0x42, 0x42, 0x10, 0x00, 0x01, 0x42, 0x42, 0x20, 0x00, 0x02, 0x42, 0x41, 0x20, 0x00, 0x02, 0x82, 0x40, 0xa0, 0x00, 0x05, 0x02, 0x40, 0x90, 0x00, 0x09, 0x02, 0x40, 0x4c, 0x00, 0x32, 0x02, 0x40, 0x33, 0x00, 0xcc, 0x02, 0x40, 0x08, 0xff, 0x10, 0x02, 0x40, 0x07, 0x00, 0xe0, 0x02, 0x40, 0x00, 0xff, 0x00, 0x02, 0x40, 0x00, 0x00, 0x00, 0x02, 0x40, 0x00, 0x00, 0x00, 0x02, 0x40, 0x00, 0x00, 0x00, 0x02, 0x20, 0x00, 0x00, 0x00, 0x04, 0x1f, 0xff, 0xff, 0xff, 0xf8};

//90,25
#define I_WASHER_MASK_WIDTH 24
#define I_WASHER_MASK_HEIGHT 24
const unsigned char i_washer_mask[72] PROGMEM = {0x33, 0x00, 0xcc, 0x4c, 0x00, 0x32, 0x90, 0x00, 0x09, 0xa0, 0x00, 0x05, 0x20, 0x00, 0x02, 0x20, 0x00, 0x02, 0x10, 0x00, 0x01, 0x08, 0x00, 0x01, 0x08, 0x00, 0x00, 0x04, 0x00, 0x00, 0x04, 0x00, 0x00, 0x04, 0x00, 0x00, 0x04, 0x00, 0x00, 0x04, 0x00, 0x00, 0x04, 0x00, 0x00, 0x08, 0x00, 0x00, 0x08, 0x00, 0x01, 0x10, 0x00, 0x01, 0x20, 0x00, 0x02, 0x20, 0x00, 0x02, 0xa0, 0x00, 0x05, 0x90, 0x00, 0x09, 0x4c, 0x00, 0x32, 0x33, 0x00, 0x66};  //{0x07, 0xf8, 0x00, 0x1f, 0xfe, 0x00, 0x7f, 0xff, 0x80, 0xff, 0xff, 0xc0, 0xff, 0xff, 0xe0, 0xff, 0xff, 0xe0, 0x7f, 0xff, 0xf0, 0x3f, 0xff, 0xf0, 0x3f, 0xff, 0xf8, 0x1f, 0xff, 0xf8, 0x1f, 0xff, 0xf8, 0x1f, 0xff, 0xf8, 0x1f, 0xff, 0xf8, 0x1f, 0xff, 0xf8, 0x1f, 0xff, 0xf8, 0x3f, 0xff, 0xf8, 0x3f, 0xff, 0xf0, 0x7f, 0xff, 0xf0, 0xff, 0xff, 0xe0, 0xff, 0xff, 0xe0, 0xff, 0xff, 0xc0, 0x7f, 0xff, 0x80, 0x1f, 0xfe, 0x00, 0x07, 0xf8, 0x00};

//90,25
#define I_GLYPH_WIDTH 24
#define I_GLYPH_HEIGHT 24
const unsigned char i_glyph[8][72] PROGMEM = {{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x38, 0x00, 0x00, 0x70, 0x00, 0x00, 0xe0, 0x00, 0x01, 0xc0, 0x00, 0x03, 0xff, 0x00, 0x01, 0xff, 0x80, 0x01, 0x81, 0x80, 0x01, 0x81, 0x80, 0x01, 0x81, 0x88, 0x01, 0x81, 0x9c, 0x01, 0x81, 0xb8, 0x01, 0x81, 0xf0, 0x01, 0xff, 0xe0, 0x00, 0xff, 0xc0, 0x00, 0x03, 0x80, 0x00, 0x07, 0x00, 0x00, 0x0e, 0x00, 0x00, 0x1c, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
                                              {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0x1f, 0x00, 0x00, 0xfc, 0x00, 0x03, 0xf0, 0x00, 0x03, 0xfc, 0x00, 0x00, 0xdf, 0x80, 0x00, 0xc3, 0x80, 0x01, 0x80, 0xc0, 0x01, 0x81, 0xc0, 0x01, 0x81, 0x80, 0x01, 0x01, 0x80, 0x01, 0xc3, 0x00, 0x00, 0xfb, 0x70, 0x00, 0x3f, 0xf8, 0x00, 0x0f, 0xc0, 0x00, 0x3f, 0x00, 0x00, 0xf8, 0x00, 0x00, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
                                              {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0f, 0xff, 0xe0, 0x07, 0xff, 0xf0, 0x00, 0x7e, 0x00, 0x00, 0xe7, 0x00, 0x01, 0xc3, 0x80, 0x01, 0x81, 0xc0, 0x01, 0x00, 0xc0, 0x01, 0x00, 0xc0, 0x01, 0x81, 0xc0, 0x01, 0xc3, 0x80, 0x00, 0xe7, 0x00, 0x00, 0x7e, 0x00, 0x07, 0xff, 0xf0, 0x0f, 0xff, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
                                              {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xe0, 0x00, 0x00, 0x78, 0x00, 0x00, 0x3f, 0x00, 0x00, 0x0f, 0xc0, 0x00, 0x3f, 0xf0, 0x01, 0xfb, 0x78, 0x01, 0xc3, 0x10, 0x01, 0x01, 0x80, 0x01, 0x81, 0x80, 0x01, 0x81, 0xc0, 0x01, 0x80, 0xc0, 0x00, 0xc3, 0x80, 0x02, 0xdf, 0x80, 0x03, 0xfc, 0x00, 0x03, 0xf0, 0x00, 0x00, 0xfc, 0x00, 0x00, 0x1f, 0x00, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
                                              {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x00, 0x00, 0x1c, 0x00, 0x00, 0x0e, 0x00, 0x00, 0x07, 0x00, 0x00, 0x03, 0x80, 0x00, 0xff, 0xc0, 0x01, 0xff, 0xe0, 0x01, 0x81, 0xf0, 0x01, 0x81, 0xb8, 0x01, 0x81, 0x9c, 0x01, 0x81, 0x88, 0x01, 0x81, 0x80, 0x01, 0x81, 0x80, 0x03, 0xff, 0x80, 0x03, 0xff, 0x00, 0x01, 0xc0, 0x00, 0x00, 0xe0, 0x00, 0x00, 0x70, 0x00, 0x00, 0x38, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
                                              {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x00, 0x01, 0x80, 0x00, 0x01, 0x80, 0x00, 0x31, 0xc0, 0x00, 0xfc, 0xc0, 0x00, 0xdf, 0xe0, 0x00, 0xc3, 0xe0, 0x01, 0x80, 0xe0, 0x01, 0x81, 0xf0, 0x01, 0x81, 0xb0, 0x01, 0x01, 0xb8, 0x01, 0xc3, 0x18, 0x03, 0xfb, 0x10, 0x03, 0x3f, 0x00, 0x03, 0x8c, 0x00, 0x01, 0x80, 0x00, 0x01, 0x80, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
                                              {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x40, 0x06, 0x00, 0x60, 0x06, 0x3c, 0x60, 0x02, 0x7e, 0x60, 0x00, 0xe7, 0x60, 0x01, 0xc3, 0xe0, 0x01, 0x81, 0xe0, 0x01, 0x00, 0xe0, 0x01, 0x00, 0xe0, 0x01, 0x81, 0xe0, 0x01, 0xc3, 0xe0, 0x02, 0xe7, 0x60, 0x02, 0x7e, 0x60, 0x06, 0x3c, 0x60, 0x02, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
                                              {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x01, 0x80, 0x00, 0x01, 0x80, 0x00, 0x03, 0x8c, 0x00, 0x03, 0x3f, 0x08, 0x01, 0xfb, 0x18, 0x01, 0xc3, 0x18, 0x01, 0x01, 0xb8, 0x01, 0x81, 0xb0, 0x01, 0x81, 0xf0, 0x01, 0x80, 0xe0, 0x00, 0xc3, 0xe0, 0x00, 0xdf, 0xe0, 0x00, 0xfc, 0xc0, 0x00, 0x31, 0xc0, 0x00, 0x01, 0x80, 0x00, 0x01, 0x80, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}};

// #define I_SPIN_WIDTH 16
// #define I_SPIN_HEIGHT 16
// const unsigned char i_spin[8][32] PROGMEM = {{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x10, 0x04, 0x38, 0x00, 0x10, 0x04, 0x00, 0x0c, 0x10, 0x1c, 0x1f, 0xf8, 0x0f, 0xf0, 0x00, 0x00, 0x00, 0x00},
//                                              {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x00, 0x38, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x04, 0x20, 0x00, 0x30, 0x04, 0x30, 0x00, 0x38, 0x08, 0x1e, 0x00, 0x0f, 0xe0, 0x00, 0x00, 0x00, 0x00},
//                                              {0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x1b, 0x80, 0x39, 0x00, 0x30, 0x00, 0x30, 0x00, 0x30, 0x00, 0x30, 0x00, 0x30, 0x00, 0x30, 0x00, 0x10, 0x08, 0x18, 0x00, 0x0e, 0xa0, 0x00, 0x00, 0x00, 0x00},
//                                              {0x00, 0x00, 0x00, 0x00, 0x0f, 0x90, 0x1f, 0x38, 0x38, 0x10, 0x30, 0x00, 0x30, 0x00, 0x30, 0x00, 0x20, 0x00, 0x20, 0x00, 0x20, 0x00, 0x00, 0x00, 0x08, 0x00, 0x02, 0x80, 0x00, 0x00, 0x00, 0x00},
//                                              {0x00, 0x00, 0x00, 0x00, 0x0f, 0xf0, 0x1f, 0xf8, 0x30, 0x18, 0x20, 0x00, 0x20, 0x08, 0x00, 0x1c, 0x20, 0x08, 0x00, 0x00, 0x20, 0x00, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
//                                              {0x00, 0x00, 0x00, 0x00, 0x07, 0xf0, 0x00, 0x18, 0x10, 0x1c, 0x00, 0x0c, 0x20, 0x0c, 0x00, 0x0c, 0x20, 0x04, 0x00, 0x00, 0x00, 0x08, 0x00, 0x1c, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
//                                              {0x00, 0x00, 0x00, 0x00, 0x05, 0x70, 0x00, 0x08, 0x10, 0x04, 0x00, 0x04, 0x00, 0x04, 0x00, 0x04, 0x00, 0x04, 0x00, 0x04, 0x00, 0x0c, 0x00, 0x9c, 0x01, 0xd8, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00},
//                                              {0x00, 0x00, 0x00, 0x00, 0x01, 0x40, 0x00, 0x10, 0x00, 0x00, 0x00, 0x04, 0x00, 0x04, 0x00, 0x04, 0x00, 0x04, 0x00, 0x04, 0x00, 0x0c, 0x08, 0x18, 0x1c, 0xf0, 0x09, 0xe0, 0x00, 0x00, 0x00, 0x00}};

// #define I_SPIN_WIDTH 8
// #define I_SPIN_HEIGHT 8
// const unsigned char i_spin[8][8] PROGMEM = {
//     {0x30, 0x40, 0x81, 0x80, 0x81, 0x80, 0x42, 0x38},
//     {0x04, 0x00, 0x01, 0x80, 0x81, 0x80, 0x42, 0x3c},
//     {0x14, 0x00, 0x01, 0x00, 0x01, 0x81, 0x42, 0x3c},
//     {0x28, 0x02, 0x81, 0x01, 0x01, 0x01, 0x02, 0x1c},
//     {0x1c, 0x42, 0x01, 0x81, 0x01, 0x81, 0x02, 0x00},
//     {0x3c, 0x42, 0x81, 0x01, 0x80, 0x00, 0x40, 0x10},
//     {0x3c, 0x40, 0x80, 0x80, 0x80, 0x80, 0x02, 0x28},
//     {0x20, 0x40, 0x80, 0x81, 0x80, 0x81, 0x40, 0x34}};

#define I_SPIN_WIDTH 16
#define I_SPIN_HEIGHT 16
const unsigned char i_spin[14][32] PROGMEM = {{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x80, 0x0d, 0x80, 0x08, 0x00, 0x10, 0x00, 0x10, 0x00, 0x10, 0x00, 0x10, 0x00, 0x00, 0x00, 0x08, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
                                              {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xe0, 0x0c, 0x60, 0x08, 0x00, 0x10, 0x00, 0x10, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
                                              {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xc0, 0x0c, 0x30, 0x08, 0x30, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
                                              {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xc0, 0x04, 0x30, 0x08, 0x10, 0x00, 0x18, 0x10, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
                                              {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xc0, 0x04, 0x30, 0x08, 0x10, 0x00, 0x08, 0x00, 0x08, 0x00, 0x18, 0x00, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
                                              {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x40, 0x04, 0x30, 0x00, 0x10, 0x00, 0x08, 0x00, 0x08, 0x00, 0x08, 0x00, 0x08, 0x00, 0x30, 0x00, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
                                              {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x40, 0x00, 0x10, 0x00, 0x10, 0x00, 0x08, 0x00, 0x08, 0x00, 0x08, 0x00, 0x08, 0x00, 0x10, 0x00, 0x70, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
                                              {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x00, 0x10, 0x00, 0x00, 0x00, 0x08, 0x00, 0x08, 0x00, 0x08, 0x00, 0x08, 0x00, 0x10, 0x01, 0xb0, 0x01, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
                                              {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00, 0x08, 0x00, 0x08, 0x00, 0x10, 0x06, 0x30, 0x07, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
                                              {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00, 0x08, 0x00, 0x00, 0x0c, 0x10, 0x0c, 0x30, 0x03, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
                                              {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x08, 0x18, 0x00, 0x08, 0x10, 0x0c, 0x20, 0x03, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
                                              {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x00, 0x18, 0x00, 0x10, 0x00, 0x10, 0x00, 0x08, 0x10, 0x0c, 0x20, 0x03, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
                                              {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0c, 0x00, 0x0c, 0x00, 0x10, 0x00, 0x10, 0x00, 0x10, 0x00, 0x10, 0x00, 0x08, 0x00, 0x0c, 0x20, 0x02, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
                                              {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0x0e, 0x00, 0x08, 0x00, 0x10, 0x00, 0x10, 0x00, 0x10, 0x00, 0x10, 0x00, 0x08, 0x00, 0x08, 0x00, 0x02, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}};

#define I_TIMEOUT_Zz_WIDTH 8
// #define I_TIMEOUT_Zz_HEIGHT 8
//const unsigned char i_timeout_Zz[8] PROGMEM = {0x07, 0x32, 0x14, 0x27, 0x4c, 0x64, 0x08, 0x0c};
//const unsigned char i_timeout_Zz[8] PROGMEM = {0x1c, 0x04, 0x08, 0x10, 0x1b, 0x01, 0x02, 0x03};
//#define I_TIMEOUT_Zz_HEIGHT 9
//const unsigned char i_timeout_Zz[9] PROGMEM = {0x0e, 0x02, 0x04, 0x08, 0x0c, 0x03, 0x01, 0x02, 0x03};
#define I_TIMEOUT_Zz_HEIGHT 10
const unsigned char i_timeout_Zz[10] PROGMEM = {0x07, 0x01, 0x02, 0x04, 0x06, 0x00, 0x03, 0x01, 0x02, 0x03};

// #define I_TIMEOUT_WIDTH 8
// #define I_TIMEOUT_HEIGHT 8
// const unsigned char i_timeout[8][8] PROGMEM = {
//     {0x3c, 0x7e, 0xff, 0xff, 0xff, 0xff, 0x7e, 0x3c},
//     {0x3c, 0x4e, 0xef, 0xff, 0xff, 0xff, 0x7e, 0x3c},
//     {0x3c, 0x4e, 0x8f, 0x8f, 0xff, 0xff, 0x7e, 0x3c},
//     {0x3c, 0x4e, 0x8f, 0x8f, 0x9f, 0xff, 0x7e, 0x3c},
//     {0x3c, 0x4e, 0x8f, 0x8f, 0x8f, 0x8f, 0x4e, 0x3c},
//     {0x3c, 0x4e, 0x8f, 0x8f, 0x8f, 0x87, 0x42, 0x3c},
//     {0x3c, 0x4e, 0x8f, 0x8f, 0x81, 0x81, 0x42, 0x3c},
//     {0x3c, 0x4e, 0x8d, 0x89, 0x81, 0x81, 0x42, 0x3c}};
//const unsigned char i_timeout[30][8] PROGMEM = {{0x3c, 0x7e, 0xff, 0xff, 0xff, 0xff, 0x7e, 0x3c}, {0x3c, 0x6e, 0xff, 0xff, 0xff, 0xff, 0x7e, 0x3c}, {0x3c, 0x6e, 0xef, 0xff, 0xff, 0xff, 0x7e, 0x3c}, {0x3c, 0x6e, 0xef, 0xef, 0xff, 0xff, 0x7e, 0x3c}, {0x3c, 0x4e, 0xef, 0xef, 0xff, 0xff, 0x7e, 0x3c}, {0x3c, 0x4e, 0xcf, 0xef, 0xff, 0xff, 0x7e, 0x3c}, {0x3c, 0x4e, 0x8f, 0xef, 0xff, 0xff, 0x7e, 0x3c}, {0x3c, 0x4e, 0x8f, 0x8f, 0xff, 0xff, 0x7e, 0x3c}, {0x3c, 0x4e, 0x8f, 0x8f, 0xbf, 0xff, 0x7e, 0x3c}, {0x3c, 0x4e, 0x8f, 0x8f, 0x9f, 0xff, 0x7e, 0x3c}, {0x3c, 0x4e, 0x8f, 0x8f, 0x8f, 0xff, 0x7e, 0x3c}, {0x3c, 0x4e, 0x8f, 0x8f, 0x8f, 0xbf, 0x7e, 0x3c}, {0x3c, 0x4e, 0x8f, 0x8f, 0x8f, 0x9f, 0x7e, 0x3c}, {0x3c, 0x4e, 0x8f, 0x8f, 0x8f, 0x9f, 0x5e, 0x3c}, {0x3c, 0x4e, 0x8f, 0x8f, 0x8f, 0x8f, 0x4e, 0x3c}, {0x3c, 0x4e, 0x8f, 0x8f, 0x8f, 0x8f, 0x46, 0x3c}, {0x3c, 0x4e, 0x8f, 0x8f, 0x8f, 0x87, 0x46, 0x3c}, {0x3c, 0x4e, 0x8f, 0x8f, 0x87, 0x87, 0x46, 0x3c}, {0x3c, 0x4e, 0x8f, 0x8f, 0x87, 0x87, 0x42, 0x3c}, {0x3c, 0x4e, 0x8f, 0x8f, 0x87, 0x83, 0x42, 0x3c}, {0x3c, 0x4e, 0x8f, 0x8f, 0x87, 0x81, 0x42, 0x3c}, {0x3c, 0x4e, 0x8f, 0x8f, 0x83, 0x81, 0x42, 0x3c}, {0x3c, 0x4e, 0x8f, 0x8f, 0x81, 0x81, 0x42, 0x3c}, {0x3c, 0x4e, 0x8f, 0x8d, 0x81, 0x81, 0x42, 0x3c}, {0x3c, 0x4e, 0x8f, 0x89, 0x81, 0x81, 0x42, 0x3c}, {0x3c, 0x4e, 0x8f, 0x81, 0x81, 0x81, 0x42, 0x3c}, {0x3c, 0x4e, 0x8d, 0x81, 0x81, 0x81, 0x42, 0x3c}, {0x3c, 0x4e, 0x89, 0x81, 0x81, 0x81, 0x42, 0x3c}, {0x3c, 0x4a, 0x89, 0x81, 0x81, 0x81, 0x42, 0x3c}, {0x3c, 0x42, 0x81, 0x81, 0x81, 0x81, 0x42, 0x3c}};
#define I_TIMEOUT_WIDTH 16
#define I_TIMEOUT_HEIGHT 16
const unsigned char i_timeout[25][32] PROGMEM = {{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xc0, 0x0f, 0xf0, 0x0f, 0xf0, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x0f, 0xf0, 0x0f, 0xf0, 0x03, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
                                                 {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xc0, 0x0e, 0xf0, 0x0e, 0xf0, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x0f, 0xf0, 0x0f, 0xf0, 0x03, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
                                                 {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xc0, 0x0c, 0xf0, 0x0e, 0xf0, 0x1e, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x0f, 0xf0, 0x0f, 0xf0, 0x03, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
                                                 {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xc0, 0x0c, 0xf0, 0x0c, 0xf0, 0x1e, 0xf8, 0x1e, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x0f, 0xf0, 0x0f, 0xf0, 0x03, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
                                                 {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xc0, 0x0c, 0xf0, 0x08, 0xf0, 0x18, 0xf8, 0x1c, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x0f, 0xf0, 0x0f, 0xf0, 0x03, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
                                                 {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xc0, 0x0c, 0xf0, 0x08, 0xf0, 0x10, 0xf8, 0x18, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x0f, 0xf0, 0x0f, 0xf0, 0x03, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
                                                 {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xc0, 0x0c, 0xf0, 0x08, 0xf0, 0x10, 0xf8, 0x10, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x0f, 0xf0, 0x0f, 0xf0, 0x03, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
                                                 {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xc0, 0x0c, 0xf0, 0x08, 0xf0, 0x10, 0xf8, 0x10, 0xf8, 0x13, 0xf8, 0x1f, 0xf8, 0x0f, 0xf0, 0x0f, 0xf0, 0x03, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
                                                 {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xc0, 0x0c, 0xf0, 0x08, 0xf0, 0x10, 0xf8, 0x10, 0xf8, 0x11, 0xf8, 0x17, 0xf8, 0x0f, 0xf0, 0x0f, 0xf0, 0x03, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
                                                 {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xc0, 0x0c, 0xf0, 0x08, 0xf0, 0x10, 0xf8, 0x10, 0xf8, 0x10, 0xf8, 0x13, 0xf8, 0x0f, 0xf0, 0x0f, 0xf0, 0x03, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
                                                 {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xc0, 0x0c, 0xf0, 0x08, 0xf0, 0x10, 0xf8, 0x10, 0xf8, 0x10, 0xf8, 0x10, 0xf8, 0x09, 0xf0, 0x0f, 0xf0, 0x03, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
                                                 {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xc0, 0x0c, 0xf0, 0x08, 0xf0, 0x10, 0xf8, 0x10, 0xf8, 0x10, 0xf8, 0x10, 0xf8, 0x08, 0xf0, 0x0d, 0xf0, 0x03, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
                                                 {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xc0, 0x0c, 0xf0, 0x08, 0xf0, 0x10, 0xf8, 0x10, 0xf8, 0x10, 0xf8, 0x10, 0xf8, 0x08, 0xf0, 0x0c, 0xf0, 0x03, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
                                                 {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xc0, 0x0c, 0xf0, 0x08, 0xf0, 0x10, 0xf8, 0x10, 0xf8, 0x10, 0xf8, 0x10, 0xf8, 0x08, 0x70, 0x0c, 0x70, 0x03, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
                                                 {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xc0, 0x0c, 0xf0, 0x08, 0xf0, 0x10, 0xf8, 0x10, 0xf8, 0x10, 0xf8, 0x10, 0x78, 0x08, 0x70, 0x0c, 0x30, 0x03, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
                                                 {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xc0, 0x0c, 0xf0, 0x08, 0xf0, 0x10, 0xf8, 0x10, 0xf8, 0x10, 0x78, 0x10, 0x38, 0x08, 0x30, 0x0c, 0x30, 0x03, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
                                                 {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xc0, 0x0c, 0xf0, 0x08, 0xf0, 0x10, 0xf8, 0x10, 0xf8, 0x10, 0x78, 0x10, 0x18, 0x08, 0x10, 0x0c, 0x30, 0x03, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
                                                 {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xc0, 0x0c, 0xf0, 0x08, 0xf0, 0x10, 0xf8, 0x10, 0xf8, 0x10, 0x18, 0x10, 0x08, 0x08, 0x10, 0x0c, 0x30, 0x03, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
                                                 {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xc0, 0x0c, 0xf0, 0x08, 0xf0, 0x10, 0xf8, 0x10, 0xf8, 0x10, 0x08, 0x10, 0x08, 0x08, 0x10, 0x0c, 0x30, 0x03, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
                                                 {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xc0, 0x0c, 0xf0, 0x08, 0xf0, 0x10, 0xf8, 0x10, 0xc8, 0x10, 0x08, 0x10, 0x08, 0x08, 0x10, 0x0c, 0x30, 0x03, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
                                                 {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xc0, 0x0c, 0xf0, 0x08, 0xf0, 0x10, 0xe8, 0x10, 0x88, 0x10, 0x08, 0x10, 0x08, 0x08, 0x10, 0x0c, 0x30, 0x03, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
                                                 {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xc0, 0x0c, 0xf0, 0x08, 0xf0, 0x10, 0x88, 0x10, 0x08, 0x10, 0x08, 0x10, 0x08, 0x08, 0x10, 0x0c, 0x30, 0x03, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
                                                 {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xc0, 0x0c, 0xf0, 0x08, 0x90, 0x10, 0x88, 0x10, 0x08, 0x10, 0x08, 0x10, 0x08, 0x08, 0x10, 0x0c, 0x30, 0x03, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
                                                 {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xc0, 0x0c, 0xb0, 0x08, 0x10, 0x10, 0x08, 0x10, 0x08, 0x10, 0x08, 0x10, 0x08, 0x08, 0x10, 0x0c, 0x30, 0x03, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
                                                 {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xc0, 0x0c, 0x30, 0x08, 0x10, 0x10, 0x08, 0x10, 0x08, 0x10, 0x08, 0x10, 0x08, 0x08, 0x10, 0x0c, 0x30, 0x03, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}};

#define I_BADBATT_WIDTH 48
#define I_BADBATT_HEIGHT 48
const unsigned char i_badbatt[288] PROGMEM = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x00, 0x00,
    0x00, 0x00, 0xc0, 0x3c, 0x00, 0x00, 0x00, 0x01, 0xe0, 0x3e, 0x00, 0x00, 0x00, 0x03, 0xe0, 0x1f,
    0x00, 0x00, 0x00, 0x07, 0xc0, 0x0f, 0x80, 0x00, 0x00, 0x0f, 0x80, 0x07, 0xc0, 0x00, 0x00, 0x1f,
    0x00, 0x7c, 0x0f, 0xff, 0xff, 0x81, 0xf0, 0xfe, 0x07, 0xff, 0xff, 0x03, 0xf8, 0xc0, 0x7c, 0x00,
    0x01, 0xf0, 0x18, 0xc0, 0x3e, 0x00, 0x03, 0xe0, 0x18, 0xc0, 0x1f, 0x00, 0x07, 0xc0, 0x18, 0xc0,
    0x0f, 0x80, 0x0f, 0x80, 0x18, 0xc0, 0x07, 0xc0, 0x1f, 0x00, 0x1e, 0xc0, 0x03, 0xe0, 0x3e, 0x00,
    0x1f, 0xc0, 0x01, 0xf0, 0x7c, 0x00, 0x1b, 0xc0, 0x00, 0xf8, 0xf8, 0x00, 0x1b, 0xc0, 0x00, 0x7d,
    0xf0, 0x00, 0x1b, 0xc0, 0x00, 0x3f, 0xe0, 0x00, 0x1b, 0xc0, 0x00, 0x1f, 0xc0, 0x00, 0x1b, 0xc0,
    0x00, 0x1f, 0xc0, 0x00, 0x1b, 0xc0, 0x00, 0x3f, 0xe0, 0x00, 0x1b, 0xc0, 0x00, 0x7d, 0xf0, 0x00,
    0x1b, 0xc0, 0x00, 0xf8, 0xf8, 0x00, 0x1b, 0xc0, 0x01, 0xf0, 0x7c, 0x00, 0x1b, 0xc0, 0x03, 0xe0,
    0x3e, 0x00, 0x1f, 0xc0, 0x07, 0xc0, 0x1f, 0x00, 0x1e, 0xc0, 0x0f, 0x80, 0x0f, 0x80, 0x18, 0xc0,
    0x1f, 0x00, 0x07, 0xc0, 0x18, 0xc0, 0x3e, 0x00, 0x03, 0xe0, 0x18, 0xc0, 0x7c, 0x00, 0x01, 0xf0,
    0x18, 0xfe, 0x07, 0xff, 0xff, 0x03, 0xf8, 0x7c, 0x0f, 0xff, 0xff, 0x81, 0xf0, 0x07, 0xc0, 0x00,
    0x00, 0x1f, 0x00, 0x0f, 0x80, 0x00, 0x00, 0x0f, 0x80, 0x1f, 0x00, 0x00, 0x00, 0x07, 0xc0, 0x3e,
    0x00, 0x00, 0x00, 0x03, 0xe0, 0x3c, 0x00, 0x00, 0x00, 0x01, 0xe0, 0x18, 0x00, 0x00, 0x00, 0x00,
    0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};