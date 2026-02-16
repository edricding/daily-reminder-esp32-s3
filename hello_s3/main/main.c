#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

#include "driver/i2c.h"
#include "driver/i2s_std.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_event.h"
#include "esp_heap_caps.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_st77916.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_sntp.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "nvs_flash.h"

// Fill your Wi-Fi here to enable NTP time sync.
#ifndef WIFI_SSID
#define WIFI_SSID "DZH_2.4G"
#endif

#ifndef WIFI_PASS
#define WIFI_PASS "18917127511"
#endif

// Asia/Shanghai: "CST-8". Change if needed.
#ifndef CLOCK_TIMEZONE
#define CLOCK_TIMEZONE "CST-8"
#endif

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1
#define WIFI_MAX_RETRY     10

#define LCD_H_RES 360
#define LCD_V_RES 360

#define LCD_SPI_HOST         SPI2_HOST
#define LCD_PIN_CS           21
#define LCD_PIN_SCK          40
#define LCD_PIN_DATA0        46
#define LCD_PIN_DATA1        45
#define LCD_PIN_DATA2        42
#define LCD_PIN_DATA3        41
#define LCD_PIN_TE           18
#define LCD_PIN_BACKLIGHT    5

#define EXIO_I2C_PORT        I2C_NUM_0
#define EXIO_I2C_SCL         10
#define EXIO_I2C_SDA         11
#define EXIO_ADDR            0x20
#define EXIO_REG_OUTPUT      0x01
#define EXIO_REG_CONFIG      0x03
#define EXIO_OUTPUT_DEFAULT  0x00
#define EXIO_TOUCH_RST_PIN   1
#define EXIO_LCD_RST_PIN     2
#define EXIO_AUDIO_SD_PIN    5

// Official schematic uses NS8002 SD pin on Extend_IO5 (MIC_SD net).
// In official demo EXIO defaults to 0x00, so keep this default low.
#ifndef EXIO_AUDIO_SD_ACTIVE_HIGH
#define EXIO_AUDIO_SD_ACTIVE_HIGH 0
#endif

#define TOUCH_I2C_ADDR       0x15
#define TOUCH_INT_PIN        4
#define TOUCH_REG_DATA       0x01
#define TOUCH_REG_CHIP_ID    0xA7
#define TOUCH_REG_SLEEP_CTRL 0xFE

#define SPEAKER_I2S_DOUT     47
#define SPEAKER_I2S_BCLK     48
#define SPEAKER_I2S_WS       38

#define BEEP_SAMPLE_RATE_HZ     44100
#define BEEP_FREQ_HZ            1600
#define BEEP_DURATION_MS        140
#define BEEP_AMPLITUDE          24000
#define BEEP_WRITE_TIMEOUT_MS   600
#define BEEP_WRITE_CHUNK_BYTES  2048

#define DRAW_CHUNK_ROWS      8

// Some ESP32-S3-Touch-LCD-1.85C batches require this init table.
static const st77916_lcd_init_cmd_t st77916_init_waveshare_185c[] = {
    {0xF0, (uint8_t[]){0x28}, 1, 0},
    {0xF2, (uint8_t[]){0x28}, 1, 0},
    {0x73, (uint8_t[]){0xF0}, 1, 0},
    {0x7C, (uint8_t[]){0xD1}, 1, 0},
    {0x83, (uint8_t[]){0xE0}, 1, 0},
    {0x84, (uint8_t[]){0x61}, 1, 0},
    {0xF2, (uint8_t[]){0x82}, 1, 0},
    {0xF0, (uint8_t[]){0x00}, 1, 0},
    {0xF0, (uint8_t[]){0x01}, 1, 0},
    {0xF1, (uint8_t[]){0x01}, 1, 0},
    {0xB0, (uint8_t[]){0x56}, 1, 0},
    {0xB1, (uint8_t[]){0x4D}, 1, 0},
    {0xB2, (uint8_t[]){0x24}, 1, 0},
    {0xB4, (uint8_t[]){0x87}, 1, 0},
    {0xB5, (uint8_t[]){0x44}, 1, 0},
    {0xB6, (uint8_t[]){0x8B}, 1, 0},
    {0xB7, (uint8_t[]){0x40}, 1, 0},
    {0xB8, (uint8_t[]){0x86}, 1, 0},
    {0xBA, (uint8_t[]){0x00}, 1, 0},
    {0xBB, (uint8_t[]){0x08}, 1, 0},
    {0xBC, (uint8_t[]){0x08}, 1, 0},
    {0xBD, (uint8_t[]){0x00}, 1, 0},
    {0xC0, (uint8_t[]){0x80}, 1, 0},
    {0xC1, (uint8_t[]){0x10}, 1, 0},
    {0xC2, (uint8_t[]){0x37}, 1, 0},
    {0xC3, (uint8_t[]){0x80}, 1, 0},
    {0xC4, (uint8_t[]){0x10}, 1, 0},
    {0xC5, (uint8_t[]){0x37}, 1, 0},
    {0xC6, (uint8_t[]){0xA9}, 1, 0},
    {0xC7, (uint8_t[]){0x41}, 1, 0},
    {0xC8, (uint8_t[]){0x01}, 1, 0},
    {0xC9, (uint8_t[]){0xA9}, 1, 0},
    {0xCA, (uint8_t[]){0x41}, 1, 0},
    {0xCB, (uint8_t[]){0x01}, 1, 0},
    {0xD0, (uint8_t[]){0x91}, 1, 0},
    {0xD1, (uint8_t[]){0x68}, 1, 0},
    {0xD2, (uint8_t[]){0x68}, 1, 0},
    {0xF5, (uint8_t[]){0x00, 0xA5}, 2, 0},
    {0xDD, (uint8_t[]){0x4F}, 1, 0},
    {0xDE, (uint8_t[]){0x4F}, 1, 0},
    {0xF1, (uint8_t[]){0x10}, 1, 0},
    {0xF0, (uint8_t[]){0x00}, 1, 0},
    {0xF0, (uint8_t[]){0x02}, 1, 0},
    {0xE0, (uint8_t[]){0xF0, 0x0A, 0x10, 0x09, 0x09, 0x36, 0x35, 0x33, 0x4A, 0x29, 0x15, 0x15, 0x2E, 0x34}, 14, 0},
    {0xE1, (uint8_t[]){0xF0, 0x0A, 0x0F, 0x08, 0x08, 0x05, 0x34, 0x33, 0x4A, 0x39, 0x15, 0x15, 0x2D, 0x33}, 14, 0},
    {0xF0, (uint8_t[]){0x10}, 1, 0},
    {0xF3, (uint8_t[]){0x10}, 1, 0},
    {0xE0, (uint8_t[]){0x07}, 1, 0},
    {0xE1, (uint8_t[]){0x00}, 1, 0},
    {0xE2, (uint8_t[]){0x00}, 1, 0},
    {0xE3, (uint8_t[]){0x00}, 1, 0},
    {0xE4, (uint8_t[]){0xE0}, 1, 0},
    {0xE5, (uint8_t[]){0x06}, 1, 0},
    {0xE6, (uint8_t[]){0x21}, 1, 0},
    {0xE7, (uint8_t[]){0x01}, 1, 0},
    {0xE8, (uint8_t[]){0x05}, 1, 0},
    {0xE9, (uint8_t[]){0x02}, 1, 0},
    {0xEA, (uint8_t[]){0xDA}, 1, 0},
    {0xEB, (uint8_t[]){0x00}, 1, 0},
    {0xEC, (uint8_t[]){0x00}, 1, 0},
    {0xED, (uint8_t[]){0x0F}, 1, 0},
    {0xEE, (uint8_t[]){0x00}, 1, 0},
    {0xEF, (uint8_t[]){0x00}, 1, 0},
    {0xF8, (uint8_t[]){0x00}, 1, 0},
    {0xF9, (uint8_t[]){0x00}, 1, 0},
    {0xFA, (uint8_t[]){0x00}, 1, 0},
    {0xFB, (uint8_t[]){0x00}, 1, 0},
    {0xFC, (uint8_t[]){0x00}, 1, 0},
    {0xFD, (uint8_t[]){0x00}, 1, 0},
    {0xFE, (uint8_t[]){0x00}, 1, 0},
    {0xFF, (uint8_t[]){0x00}, 1, 0},
    {0x60, (uint8_t[]){0x40}, 1, 0},
    {0x61, (uint8_t[]){0x04}, 1, 0},
    {0x62, (uint8_t[]){0x00}, 1, 0},
    {0x63, (uint8_t[]){0x42}, 1, 0},
    {0x64, (uint8_t[]){0xD9}, 1, 0},
    {0x65, (uint8_t[]){0x00}, 1, 0},
    {0x66, (uint8_t[]){0x00}, 1, 0},
    {0x67, (uint8_t[]){0x00}, 1, 0},
    {0x68, (uint8_t[]){0x00}, 1, 0},
    {0x69, (uint8_t[]){0x00}, 1, 0},
    {0x6A, (uint8_t[]){0x00}, 1, 0},
    {0x6B, (uint8_t[]){0x00}, 1, 0},
    {0x70, (uint8_t[]){0x40}, 1, 0},
    {0x71, (uint8_t[]){0x03}, 1, 0},
    {0x72, (uint8_t[]){0x00}, 1, 0},
    {0x73, (uint8_t[]){0x42}, 1, 0},
    {0x74, (uint8_t[]){0xD8}, 1, 0},
    {0x75, (uint8_t[]){0x00}, 1, 0},
    {0x76, (uint8_t[]){0x00}, 1, 0},
    {0x77, (uint8_t[]){0x00}, 1, 0},
    {0x78, (uint8_t[]){0x00}, 1, 0},
    {0x79, (uint8_t[]){0x00}, 1, 0},
    {0x7A, (uint8_t[]){0x00}, 1, 0},
    {0x7B, (uint8_t[]){0x00}, 1, 0},
    {0x80, (uint8_t[]){0x48}, 1, 0},
    {0x81, (uint8_t[]){0x00}, 1, 0},
    {0x82, (uint8_t[]){0x06}, 1, 0},
    {0x83, (uint8_t[]){0x02}, 1, 0},
    {0x84, (uint8_t[]){0xD6}, 1, 0},
    {0x85, (uint8_t[]){0x04}, 1, 0},
    {0x86, (uint8_t[]){0x00}, 1, 0},
    {0x87, (uint8_t[]){0x00}, 1, 0},
    {0x88, (uint8_t[]){0x48}, 1, 0},
    {0x89, (uint8_t[]){0x00}, 1, 0},
    {0x8A, (uint8_t[]){0x08}, 1, 0},
    {0x8B, (uint8_t[]){0x02}, 1, 0},
    {0x8C, (uint8_t[]){0xD8}, 1, 0},
    {0x8D, (uint8_t[]){0x04}, 1, 0},
    {0x8E, (uint8_t[]){0x00}, 1, 0},
    {0x8F, (uint8_t[]){0x00}, 1, 0},
    {0x90, (uint8_t[]){0x48}, 1, 0},
    {0x91, (uint8_t[]){0x00}, 1, 0},
    {0x92, (uint8_t[]){0x0A}, 1, 0},
    {0x93, (uint8_t[]){0x02}, 1, 0},
    {0x94, (uint8_t[]){0xDA}, 1, 0},
    {0x95, (uint8_t[]){0x04}, 1, 0},
    {0x96, (uint8_t[]){0x00}, 1, 0},
    {0x97, (uint8_t[]){0x00}, 1, 0},
    {0x98, (uint8_t[]){0x48}, 1, 0},
    {0x99, (uint8_t[]){0x00}, 1, 0},
    {0x9A, (uint8_t[]){0x0C}, 1, 0},
    {0x9B, (uint8_t[]){0x02}, 1, 0},
    {0x9C, (uint8_t[]){0xDC}, 1, 0},
    {0x9D, (uint8_t[]){0x04}, 1, 0},
    {0x9E, (uint8_t[]){0x00}, 1, 0},
    {0x9F, (uint8_t[]){0x00}, 1, 0},
    {0xA0, (uint8_t[]){0x48}, 1, 0},
    {0xA1, (uint8_t[]){0x00}, 1, 0},
    {0xA2, (uint8_t[]){0x05}, 1, 0},
    {0xA3, (uint8_t[]){0x02}, 1, 0},
    {0xA4, (uint8_t[]){0xD5}, 1, 0},
    {0xA5, (uint8_t[]){0x04}, 1, 0},
    {0xA6, (uint8_t[]){0x00}, 1, 0},
    {0xA7, (uint8_t[]){0x00}, 1, 0},
    {0xA8, (uint8_t[]){0x48}, 1, 0},
    {0xA9, (uint8_t[]){0x00}, 1, 0},
    {0xAA, (uint8_t[]){0x07}, 1, 0},
    {0xAB, (uint8_t[]){0x02}, 1, 0},
    {0xAC, (uint8_t[]){0xD7}, 1, 0},
    {0xAD, (uint8_t[]){0x04}, 1, 0},
    {0xAE, (uint8_t[]){0x00}, 1, 0},
    {0xAF, (uint8_t[]){0x00}, 1, 0},
    {0xB0, (uint8_t[]){0x48}, 1, 0},
    {0xB1, (uint8_t[]){0x00}, 1, 0},
    {0xB2, (uint8_t[]){0x09}, 1, 0},
    {0xB3, (uint8_t[]){0x02}, 1, 0},
    {0xB4, (uint8_t[]){0xD9}, 1, 0},
    {0xB5, (uint8_t[]){0x04}, 1, 0},
    {0xB6, (uint8_t[]){0x00}, 1, 0},
    {0xB7, (uint8_t[]){0x00}, 1, 0},
    {0xB8, (uint8_t[]){0x48}, 1, 0},
    {0xB9, (uint8_t[]){0x00}, 1, 0},
    {0xBA, (uint8_t[]){0x0B}, 1, 0},
    {0xBB, (uint8_t[]){0x02}, 1, 0},
    {0xBC, (uint8_t[]){0xDB}, 1, 0},
    {0xBD, (uint8_t[]){0x04}, 1, 0},
    {0xBE, (uint8_t[]){0x00}, 1, 0},
    {0xBF, (uint8_t[]){0x00}, 1, 0},
    {0xC0, (uint8_t[]){0x10}, 1, 0},
    {0xC1, (uint8_t[]){0x47}, 1, 0},
    {0xC2, (uint8_t[]){0x56}, 1, 0},
    {0xC3, (uint8_t[]){0x65}, 1, 0},
    {0xC4, (uint8_t[]){0x74}, 1, 0},
    {0xC5, (uint8_t[]){0x88}, 1, 0},
    {0xC6, (uint8_t[]){0x99}, 1, 0},
    {0xC7, (uint8_t[]){0x01}, 1, 0},
    {0xC8, (uint8_t[]){0xBB}, 1, 0},
    {0xC9, (uint8_t[]){0xAA}, 1, 0},
    {0xD0, (uint8_t[]){0x10}, 1, 0},
    {0xD1, (uint8_t[]){0x47}, 1, 0},
    {0xD2, (uint8_t[]){0x56}, 1, 0},
    {0xD3, (uint8_t[]){0x65}, 1, 0},
    {0xD4, (uint8_t[]){0x74}, 1, 0},
    {0xD5, (uint8_t[]){0x88}, 1, 0},
    {0xD6, (uint8_t[]){0x99}, 1, 0},
    {0xD7, (uint8_t[]){0x01}, 1, 0},
    {0xD8, (uint8_t[]){0xBB}, 1, 0},
    {0xD9, (uint8_t[]){0xAA}, 1, 0},
    {0xF3, (uint8_t[]){0x01}, 1, 0},
    {0xF0, (uint8_t[]){0x00}, 1, 0},
    {0x21, (uint8_t[]){0x00}, 1, 0},
    {0x11, (uint8_t[]){0x00}, 1, 120},
    {0x29, (uint8_t[]){0x00}, 1, 0},
};

static const char *TAG = "clock_lcd";

static EventGroupHandle_t s_wifi_event_group;
static int s_wifi_retry_count = 0;
static bool s_time_synced = false;

static esp_lcd_panel_handle_t s_panel = NULL;
static uint16_t *s_draw_buf = NULL;
static size_t s_draw_buf_pixels = 0;
static uint64_t s_boot_us = 0;
static uint8_t s_exio_output_state = 0;

static i2s_chan_handle_t s_i2s_tx_chan = NULL;
static i2s_chan_handle_t s_i2s_rx_chan = NULL;
static int16_t *s_beep_pcm = NULL;
static size_t s_beep_pcm_bytes = 0;

static bool s_touch_pressed_last = false;
static int64_t s_last_beep_us = 0;

static void beep_once(void);

enum {
    SEG_A = 1 << 0,
    SEG_B = 1 << 1,
    SEG_C = 1 << 2,
    SEG_D = 1 << 3,
    SEG_E = 1 << 4,
    SEG_F = 1 << 5,
    SEG_G = 1 << 6,
};

static const uint8_t s_digit_mask[10] = {
    SEG_A | SEG_B | SEG_C | SEG_D | SEG_E | SEG_F,
    SEG_B | SEG_C,
    SEG_A | SEG_B | SEG_D | SEG_E | SEG_G,
    SEG_A | SEG_B | SEG_C | SEG_D | SEG_G,
    SEG_B | SEG_C | SEG_F | SEG_G,
    SEG_A | SEG_C | SEG_D | SEG_F | SEG_G,
    SEG_A | SEG_C | SEG_D | SEG_E | SEG_F | SEG_G,
    SEG_A | SEG_B | SEG_C,
    SEG_A | SEG_B | SEG_C | SEG_D | SEG_E | SEG_F | SEG_G,
    SEG_A | SEG_B | SEG_C | SEG_D | SEG_F | SEG_G,
};

static uint16_t rgb565_be(uint8_t r, uint8_t g, uint8_t b)
{
    uint16_t c = ((uint16_t)(r & 0xF8) << 8) | ((uint16_t)(g & 0xFC) << 3) | ((uint16_t)b >> 3);
    return (uint16_t)((c << 8) | (c >> 8));
}

static esp_err_t ensure_draw_buf(size_t pixels)
{
    if (s_draw_buf_pixels >= pixels) {
        return ESP_OK;
    }

    if (s_draw_buf) {
        free(s_draw_buf);
        s_draw_buf = NULL;
        s_draw_buf_pixels = 0;
    }

    s_draw_buf = heap_caps_malloc(pixels * sizeof(uint16_t), MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    if (!s_draw_buf) {
        s_draw_buf = heap_caps_malloc(pixels * sizeof(uint16_t), MALLOC_CAP_DMA);
    }
    if (!s_draw_buf) {
        return ESP_ERR_NO_MEM;
    }

    s_draw_buf_pixels = pixels;
    return ESP_OK;
}

static void lcd_fill_rect(int x, int y, int w, int h, uint16_t color)
{
    if (!s_panel || w <= 0 || h <= 0) {
        return;
    }

    if (x < 0) {
        w += x;
        x = 0;
    }
    if (y < 0) {
        h += y;
        y = 0;
    }
    if (x + w > LCD_H_RES) {
        w = LCD_H_RES - x;
    }
    if (y + h > LCD_V_RES) {
        h = LCD_V_RES - y;
    }
    if (w <= 0 || h <= 0) {
        return;
    }

    size_t chunk_pixels = (size_t)w * DRAW_CHUNK_ROWS;
    if (ensure_draw_buf(chunk_pixels) != ESP_OK) {
        return;
    }

    for (size_t i = 0; i < chunk_pixels; i++) {
        s_draw_buf[i] = color;
    }

    int remain = h;
    int y_pos = y;
    while (remain > 0) {
        int rows = (remain > DRAW_CHUNK_ROWS) ? DRAW_CHUNK_ROWS : remain;
        esp_lcd_panel_draw_bitmap(s_panel, x, y_pos, x + w, y_pos + rows, s_draw_buf);
        y_pos += rows;
        remain -= rows;
    }
}

static void draw_colon(int x, int y, int digit_h, int dot_size, uint16_t color)
{
    int top_y = y + digit_h / 3;
    int bottom_y = y + (digit_h * 2) / 3;
    lcd_fill_rect(x, top_y, dot_size, dot_size, color);
    lcd_fill_rect(x, bottom_y, dot_size, dot_size, color);
}

static void draw_digit_mask(int x, int y, int digit_w, int digit_h, int seg_w, uint8_t mask, uint16_t color)
{
    int mid_y = y + digit_h / 2 - seg_w / 2;
    int upper_h = mid_y - (y + seg_w);
    int lower_y = mid_y + seg_w;
    int lower_h = (y + digit_h - seg_w) - lower_y;

    if (upper_h < 1) {
        upper_h = 1;
    }
    if (lower_h < 1) {
        lower_h = 1;
    }

    if (mask & SEG_A) {
        lcd_fill_rect(x + seg_w, y, digit_w - 2 * seg_w, seg_w, color);
    }
    if (mask & SEG_B) {
        lcd_fill_rect(x + digit_w - seg_w, y + seg_w, seg_w, upper_h, color);
    }
    if (mask & SEG_C) {
        lcd_fill_rect(x + digit_w - seg_w, lower_y, seg_w, lower_h, color);
    }
    if (mask & SEG_D) {
        lcd_fill_rect(x + seg_w, y + digit_h - seg_w, digit_w - 2 * seg_w, seg_w, color);
    }
    if (mask & SEG_E) {
        lcd_fill_rect(x, lower_y, seg_w, lower_h, color);
    }
    if (mask & SEG_F) {
        lcd_fill_rect(x, y + seg_w, seg_w, upper_h, color);
    }
    if (mask & SEG_G) {
        lcd_fill_rect(x + seg_w, mid_y, digit_w - 2 * seg_w, seg_w, color);
    }
}

static void draw_digit_value(int x, int y, int digit_w, int digit_h, int seg_w, int value, uint16_t color)
{
    if (value < 0 || value > 9) {
        return;
    }
    draw_digit_mask(x, y, digit_w, digit_h, seg_w, s_digit_mask[value], color);
}

static void draw_time(const struct tm *ti, bool synced)
{
    const uint16_t bg = rgb565_be(0, 0, 0);
    const uint16_t fg = synced ? rgb565_be(0, 220, 90) : rgb565_be(230, 180, 0);

    const int digit_w = 42;
    const int digit_h = 96;
    const int seg_w = 8;
    const int colon_w = 8;
    const int gap = 8;

    int total_w = (6 * digit_w) + (2 * colon_w) + (7 * gap);
    int x0 = (LCD_H_RES - total_w) / 2;
    int y = (LCD_V_RES - digit_h) / 2;

    int digits[6] = {
        ti->tm_hour / 10,
        ti->tm_hour % 10,
        ti->tm_min / 10,
        ti->tm_min % 10,
        ti->tm_sec / 10,
        ti->tm_sec % 10
    };

    static bool initialized = false;
    static bool last_synced = false;
    static int last_digits[6] = {-1, -1, -1, -1, -1, -1};

    int digit_x[6] = {0};
    int colon_x[2] = {0};
    int x = x0;
    for (int i = 0; i < 6; i++) {
        digit_x[i] = x;
        x += digit_w;

        if (i == 1) {
            x += gap;
            colon_x[0] = x;
            x += colon_w;
        } else if (i == 3) {
            x += gap;
            colon_x[1] = x;
            x += colon_w;
        }
        x += gap;
    }

    if (!initialized || synced != last_synced) {
        lcd_fill_rect(0, y - 20, LCD_H_RES, digit_h + 40, bg);
        for (int i = 0; i < 6; i++) {
            draw_digit_value(digit_x[i], y, digit_w, digit_h, seg_w, digits[i], fg);
            last_digits[i] = digits[i];
        }
        draw_colon(colon_x[0], y, digit_h, colon_w, fg);
        draw_colon(colon_x[1], y, digit_h, colon_w, fg);
        initialized = true;
        last_synced = synced;
        return;
    }

    for (int i = 0; i < 6; i++) {
        if (digits[i] == last_digits[i]) {
            continue;
        }
        uint8_t old_mask = (last_digits[i] >= 0 && last_digits[i] <= 9) ? s_digit_mask[last_digits[i]] : 0;
        uint8_t new_mask = s_digit_mask[digits[i]];
        uint8_t turn_off = old_mask & (uint8_t)(~new_mask);
        uint8_t turn_on = new_mask & (uint8_t)(~old_mask);
        if (turn_off) {
            draw_digit_mask(digit_x[i], y, digit_w, digit_h, seg_w, turn_off, bg);
        }
        if (turn_on) {
            draw_digit_mask(digit_x[i], y, digit_w, digit_h, seg_w, turn_on, fg);
        }
        last_digits[i] = digits[i];
    }
}

static esp_err_t i2c_write_u8(uint8_t dev_addr, uint8_t reg, uint8_t val)
{
    uint8_t payload[2] = {reg, val};
    return i2c_master_write_to_device(EXIO_I2C_PORT, dev_addr, payload, sizeof(payload), pdMS_TO_TICKS(100));
}

static esp_err_t i2c_read_bytes(uint8_t dev_addr, uint8_t reg, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(EXIO_I2C_PORT, dev_addr, &reg, 1, data, len, pdMS_TO_TICKS(100));
}

static esp_err_t exio_write_reg(uint8_t reg, uint8_t val)
{
    return i2c_write_u8(EXIO_ADDR, reg, val);
}

static esp_err_t exio_write_output(uint8_t val)
{
    s_exio_output_state = val;
    return exio_write_reg(EXIO_REG_OUTPUT, val);
}

static esp_err_t exio_set_pin_level(uint8_t exio_pin, bool high)
{
    if (exio_pin < 1 || exio_pin > 8) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t mask = (uint8_t)(1U << (exio_pin - 1));
    uint8_t next = s_exio_output_state;
    if (high) {
        next |= mask;
    } else {
        next &= (uint8_t)(~mask);
    }
    return exio_write_output(next);
}

static void exio_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = EXIO_I2C_SDA,
        .scl_io_num = EXIO_I2C_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 400000,
        .clk_flags = 0,
    };
    ESP_ERROR_CHECK(i2c_param_config(EXIO_I2C_PORT, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(EXIO_I2C_PORT, conf.mode, 0, 0, 0));

    // TCA9554: all pins output.
    ESP_ERROR_CHECK(exio_write_reg(EXIO_REG_CONFIG, 0x00));
    // Match official demo: default output state is 0x00.
    ESP_ERROR_CHECK(exio_write_output(EXIO_OUTPUT_DEFAULT));
    ESP_LOGI(TAG, "EXIO output default: 0x%02x", s_exio_output_state);
}

static void audio_amp_init_via_exio(void)
{
    bool sd_level = EXIO_AUDIO_SD_ACTIVE_HIGH ? true : false;
    ESP_ERROR_CHECK(exio_set_pin_level(EXIO_AUDIO_SD_PIN, sd_level));
    ESP_LOGI(TAG, "audio amp SD pin EXIO%d set to %d", EXIO_AUDIO_SD_PIN, sd_level ? 1 : 0);
}

static void audio_amp_sd_selftest(void)
{
    ESP_LOGI(TAG, "audio SD self-test: EXIO%d=0", EXIO_AUDIO_SD_PIN);
    ESP_ERROR_CHECK(exio_set_pin_level(EXIO_AUDIO_SD_PIN, false));
    vTaskDelay(pdMS_TO_TICKS(80));
    beep_once();
    vTaskDelay(pdMS_TO_TICKS(120));

    ESP_LOGI(TAG, "audio SD self-test: EXIO%d=1", EXIO_AUDIO_SD_PIN);
    ESP_ERROR_CHECK(exio_set_pin_level(EXIO_AUDIO_SD_PIN, true));
    vTaskDelay(pdMS_TO_TICKS(80));
    beep_once();
    vTaskDelay(pdMS_TO_TICKS(120));

    // Restore configured level after probing both polarities.
    audio_amp_init_via_exio();
}

static void lcd_hw_reset_via_exio(void)
{
    ESP_ERROR_CHECK(exio_set_pin_level(EXIO_LCD_RST_PIN, false));
    vTaskDelay(pdMS_TO_TICKS(20));
    ESP_ERROR_CHECK(exio_set_pin_level(EXIO_LCD_RST_PIN, true));
    vTaskDelay(pdMS_TO_TICKS(120));
}

static void touch_hw_reset_via_exio(void)
{
    ESP_ERROR_CHECK(exio_set_pin_level(EXIO_TOUCH_RST_PIN, false));
    vTaskDelay(pdMS_TO_TICKS(10));
    ESP_ERROR_CHECK(exio_set_pin_level(EXIO_TOUCH_RST_PIN, true));
    vTaskDelay(pdMS_TO_TICKS(50));
}

static void touch_init(void)
{
    touch_hw_reset_via_exio();

    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << TOUCH_INT_PIN,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    uint8_t chip_id = 0;
    if (i2c_read_bytes(TOUCH_I2C_ADDR, TOUCH_REG_CHIP_ID, &chip_id, 1) == ESP_OK) {
        ESP_LOGI(TAG, "CST816 chip id: 0x%02x", chip_id);
    } else {
        ESP_LOGW(TAG, "CST816 chip id read failed");
    }

    // Same setting used in known working example to keep touch awake.
    if (i2c_write_u8(TOUCH_I2C_ADDR, TOUCH_REG_SLEEP_CTRL, 10) != ESP_OK) {
        ESP_LOGW(TAG, "CST816 sleep config write failed");
    }
}

static bool touch_read_pressed(uint16_t *x, uint16_t *y)
{
    uint8_t buf[6] = {0};
    if (i2c_read_bytes(TOUCH_I2C_ADDR, TOUCH_REG_DATA, buf, sizeof(buf)) != ESP_OK) {
        return false;
    }

    uint8_t points = (uint8_t)(buf[1] & 0x0F);
    if (points == 0) {
        return false;
    }

    if (x) {
        *x = (uint16_t)(((buf[2] & 0x0F) << 8) | buf[3]);
    }
    if (y) {
        *y = (uint16_t)(((buf[4] & 0x0F) << 8) | buf[5]);
    }
    return true;
}

static void beep_init(void)
{
    size_t beep_samples = (BEEP_SAMPLE_RATE_HZ * BEEP_DURATION_MS) / 1000;
    if (beep_samples == 0) {
        beep_samples = 1;
    }

    s_beep_pcm_bytes = beep_samples * 2 * sizeof(int16_t);  // stereo 16-bit
    s_beep_pcm = heap_caps_malloc(s_beep_pcm_bytes, MALLOC_CAP_INTERNAL);
    if (!s_beep_pcm) {
        s_beep_pcm = heap_caps_malloc(s_beep_pcm_bytes, MALLOC_CAP_DEFAULT);
    }
    if (!s_beep_pcm) {
        ESP_LOGW(TAG, "beep pcm alloc failed");
        return;
    }

    int period = BEEP_SAMPLE_RATE_HZ / BEEP_FREQ_HZ;
    if (period < 2) {
        period = 2;
    }

    for (size_t i = 0; i < beep_samples; i++) {
        int16_t sample = ((i % (size_t)period) < (size_t)(period / 2)) ? BEEP_AMPLITUDE : (int16_t)-BEEP_AMPLITUDE;
        s_beep_pcm[i * 2] = sample;
        s_beep_pcm[i * 2 + 1] = sample;
    }

    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    chan_cfg.auto_clear = true;

    esp_err_t err = i2s_new_channel(&chan_cfg, &s_i2s_tx_chan, &s_i2s_rx_chan);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "i2s_new_channel failed: %s", esp_err_to_name(err));
        return;
    }

    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(BEEP_SAMPLE_RATE_HZ),
        .slot_cfg = I2S_STD_PHILIP_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = SPEAKER_I2S_BCLK,
            .ws = SPEAKER_I2S_WS,
            .dout = SPEAKER_I2S_DOUT,
            .din = I2S_GPIO_UNUSED,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false,
            },
        },
    };

    err = i2s_channel_init_std_mode(s_i2s_tx_chan, &std_cfg);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "i2s_channel_init_std_mode failed: %s", esp_err_to_name(err));
        return;
    }

    err = i2s_channel_enable(s_i2s_tx_chan);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "i2s_channel_enable failed: %s", esp_err_to_name(err));
        return;
    }

    ESP_LOGI(TAG, "beep i2s ready: %d Hz, PHILIPS, bclk=%d ws=%d dout=%d",
             BEEP_SAMPLE_RATE_HZ, SPEAKER_I2S_BCLK, SPEAKER_I2S_WS, SPEAKER_I2S_DOUT);
}

static void beep_once(void)
{
    if (!s_i2s_tx_chan || !s_beep_pcm || s_beep_pcm_bytes == 0) {
        return;
    }

    size_t offset = 0;
    esp_err_t last_err = ESP_OK;
    size_t last_written = 0;
    while (offset < s_beep_pcm_bytes) {
        size_t remain = s_beep_pcm_bytes - offset;
        size_t chunk = (remain > BEEP_WRITE_CHUNK_BYTES) ? BEEP_WRITE_CHUNK_BYTES : remain;
        size_t written = 0;
        esp_err_t err = i2s_channel_write(
            s_i2s_tx_chan,
            ((const uint8_t *)s_beep_pcm) + offset,
            chunk,
            &written,
            pdMS_TO_TICKS(BEEP_WRITE_TIMEOUT_MS));

        // If timeout happens after partial DMA copy, keep moving forward.
        if (written > 0) {
            offset += written;
        }

        if (err != ESP_OK && written == 0) {
            last_err = err;
            last_written = written;
            break;
        }

        if (err != ESP_OK) {
            last_err = err;
            last_written = written;
        }
    }

    if (offset != s_beep_pcm_bytes) {
        ESP_LOGW(TAG, "beep write partial: %u/%u bytes, err=%s, last_written=%u",
                 (unsigned)offset, (unsigned)s_beep_pcm_bytes,
                 esp_err_to_name(last_err), (unsigned)last_written);
    }
}

static void touch_check_and_beep(void)
{
    uint16_t x = 0;
    uint16_t y = 0;
    bool pressed = touch_read_pressed(&x, &y);
    int64_t now_us = esp_timer_get_time();

    if (pressed && !s_touch_pressed_last && (now_us - s_last_beep_us) > 120000) {
        s_last_beep_us = now_us;
        ESP_LOGI(TAG, "touch press: x=%u y=%u", x, y);
        beep_once();
    }
    s_touch_pressed_last = pressed;
}

static void backlight_init(void)
{
    ledc_timer_config_t timer_cfg = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = 20000,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ledc_channel_config_t channel_cfg = {
        .gpio_num = LCD_PIN_BACKLIGHT,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0,
        .intr_type = LEDC_INTR_DISABLE,
        .flags.output_invert = 0,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_cfg));
    ESP_ERROR_CHECK(ledc_channel_config(&channel_cfg));
}

static void backlight_set_percent(uint8_t percent)
{
    if (percent > 100) {
        percent = 100;
    }
    uint32_t duty = (percent * 1023U) / 100U;
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0));
}

static void lcd_init(void)
{
    gpio_set_direction(LCD_PIN_TE, GPIO_MODE_OUTPUT);
    gpio_set_level(LCD_PIN_TE, 0);

    spi_bus_config_t bus_cfg = {
        .sclk_io_num = LCD_PIN_SCK,
        .data0_io_num = LCD_PIN_DATA0,
        .data1_io_num = LCD_PIN_DATA1,
        .data2_io_num = LCD_PIN_DATA2,
        .data3_io_num = LCD_PIN_DATA3,
        .max_transfer_sz = LCD_H_RES * DRAW_CHUNK_ROWS * sizeof(uint16_t),
        .flags = SPICOMMON_BUSFLAG_MASTER,
        .intr_flags = 0,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(LCD_SPI_HOST, &bus_cfg, SPI_DMA_CH_AUTO));

    esp_lcd_panel_io_spi_config_t io_cfg = ST77916_PANEL_IO_QSPI_CONFIG(LCD_PIN_CS, NULL, NULL);
    io_cfg.pclk_hz = 20 * 1000 * 1000;
    io_cfg.trans_queue_depth = 10;

    esp_lcd_panel_io_handle_t io_handle = NULL;
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_SPI_HOST, &io_cfg, &io_handle));

    st77916_vendor_config_t vendor_cfg = {
        .init_cmds = st77916_init_waveshare_185c,
        .init_cmds_size = sizeof(st77916_init_waveshare_185c) / sizeof(st77916_init_waveshare_185c[0]),
        .flags.use_qspi_interface = 1,
    };

    esp_lcd_panel_dev_config_t panel_cfg = {
        .reset_gpio_num = -1,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
        .data_endian = LCD_RGB_DATA_ENDIAN_BIG,
        .bits_per_pixel = 16,
        .vendor_config = &vendor_cfg,
        .flags.reset_active_high = 0,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_st77916(io_handle, &panel_cfg, &s_panel));
    ESP_ERROR_CHECK(esp_lcd_panel_reset(s_panel));
    ESP_ERROR_CHECK(esp_lcd_panel_init(s_panel));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(s_panel, true));
}

static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    (void)arg;
    (void)event_data;

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_wifi_retry_count < WIFI_MAX_RETRY) {
            esp_wifi_connect();
            s_wifi_retry_count++;
            ESP_LOGI(TAG, "retry Wi-Fi connection (%d/%d)", s_wifi_retry_count, WIFI_MAX_RETRY);
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        s_wifi_retry_count = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

static void nvs_init(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
}

static bool wifi_connect_blocking(void)
{
    if (strlen(WIFI_SSID) == 0) {
        ESP_LOGW(TAG, "WIFI_SSID is empty, skip Wi-Fi/NTP and show uptime clock");
        return false;
    }

    s_wifi_event_group = xEventGroupCreate();
    if (!s_wifi_event_group) {
        return false;
    }

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t any_id;
    esp_event_handler_instance_t got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &got_ip));

    wifi_config_t wifi_cfg = {0};
    strlcpy((char *)wifi_cfg.sta.ssid, WIFI_SSID, sizeof(wifi_cfg.sta.ssid));
    strlcpy((char *)wifi_cfg.sta.password, WIFI_PASS, sizeof(wifi_cfg.sta.password));
    wifi_cfg.sta.pmf_cfg.capable = true;
    wifi_cfg.sta.pmf_cfg.required = false;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "connecting to Wi-Fi: %s", WIFI_SSID);
    EventBits_t bits = xEventGroupWaitBits(
        s_wifi_event_group,
        WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
        pdFALSE,
        pdFALSE,
        pdMS_TO_TICKS(20000));

    bool connected = (bits & WIFI_CONNECTED_BIT) != 0;
    if (!connected) {
        ESP_LOGW(TAG, "Wi-Fi connect timeout or failure");
    }

    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, got_ip));
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, any_id));
    vEventGroupDelete(s_wifi_event_group);
    s_wifi_event_group = NULL;

    return connected;
}

static bool sync_time_from_ntp(void)
{
    setenv("TZ", CLOCK_TIMEZONE, 1);
    tzset();

    static const char *ntp_servers[] = {
        "ntp.aliyun.com",
        "ntp.ntsc.ac.cn",
        "pool.ntp.org",
        "time.cloudflare.com",
    };

    time_t now = 0;
    struct tm ti = {0};

    for (size_t s = 0; s < sizeof(ntp_servers) / sizeof(ntp_servers[0]); s++) {
        esp_sntp_stop();
        esp_sntp_setoperatingmode(ESP_SNTP_OPMODE_POLL);
        esp_sntp_set_sync_mode(SNTP_SYNC_MODE_IMMED);
        esp_sntp_setservername(0, ntp_servers[s]);
        esp_sntp_init();

        ESP_LOGI(TAG, "NTP trying server: %s", ntp_servers[s]);

        for (int i = 0; i < 50; i++) {  // 25s per server
            if (esp_sntp_get_sync_status() == SNTP_SYNC_STATUS_COMPLETED) {
                time(&now);
                localtime_r(&now, &ti);
                ESP_LOGI(TAG, "NTP synced via %s: %04d-%02d-%02d %02d:%02d:%02d",
                         ntp_servers[s],
                         ti.tm_year + 1900, ti.tm_mon + 1, ti.tm_mday,
                         ti.tm_hour, ti.tm_min, ti.tm_sec);
                return true;
            }

            time(&now);
            localtime_r(&now, &ti);
            if (ti.tm_year >= (2024 - 1900)) {
                ESP_LOGI(TAG, "NTP synced via %s: %04d-%02d-%02d %02d:%02d:%02d",
                         ntp_servers[s],
                         ti.tm_year + 1900, ti.tm_mon + 1, ti.tm_mday,
                         ti.tm_hour, ti.tm_min, ti.tm_sec);
                return true;
            }
            vTaskDelay(pdMS_TO_TICKS(500));
        }

        ESP_LOGW(TAG, "NTP timeout on server: %s", ntp_servers[s]);
    }

    ESP_LOGW(TAG, "all NTP servers failed, fallback to uptime clock");
    return false;
}

void app_main(void)
{
    s_boot_us = esp_timer_get_time();

    nvs_init();
    exio_init();
    lcd_hw_reset_via_exio();
    touch_init();
    audio_amp_init_via_exio();
    beep_init();
    vTaskDelay(pdMS_TO_TICKS(120));
    audio_amp_sd_selftest();
    ESP_LOGI(TAG, "audio self-test beep");
    beep_once();
    vTaskDelay(pdMS_TO_TICKS(120));
    beep_once();
    backlight_init();
    backlight_set_percent(60);
    lcd_init();

    s_time_synced = false;
    if (wifi_connect_blocking()) {
        s_time_synced = sync_time_from_ntp();
    }

    int64_t last_clock_update_us = 0;
    while (1) {
        int64_t now_us = esp_timer_get_time();

        if (now_us - last_clock_update_us >= 1000000) {
            last_clock_update_us = now_us;
            struct tm ti = {0};
            if (s_time_synced) {
                time_t now = 0;
                time(&now);
                localtime_r(&now, &ti);
                if (ti.tm_year < (2024 - 1900)) {
                    s_time_synced = false;
                }
            }

            if (!s_time_synced) {
                uint64_t uptime_s = (esp_timer_get_time() - s_boot_us) / 1000000ULL;
                ti.tm_hour = (int)((uptime_s / 3600ULL) % 24ULL);
                ti.tm_min = (int)((uptime_s / 60ULL) % 60ULL);
                ti.tm_sec = (int)(uptime_s % 60ULL);
            }

            draw_time(&ti, s_time_synced);
            ESP_LOGI(TAG, "displayed: %02d:%02d:%02d (%s)",
                     ti.tm_hour, ti.tm_min, ti.tm_sec,
                     s_time_synced ? "ntp" : "uptime");
        }

        touch_check_and_beep();
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
