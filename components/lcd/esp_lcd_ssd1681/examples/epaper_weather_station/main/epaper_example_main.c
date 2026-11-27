/*
 * E-paper weather station, LVGL 9.6 + SSD1681.
 *
 * What this example demonstrates:
 * - One driver for both a monochrome and a black/red/white panel. Select the
 *   panel with PANEL_TYPE below. MONO uses an LVGL I1 (1bpp) buffer; TRICOLOR
 *   uses I2 (2bpp, palette [white, black, red, white]) and the driver splits
 *   each pixel into the BW (0x24) and RED (0x26) planes in one pass. No
 *   RGB565 conversion happens in the driver.
 * - The LVGL refresh timer is paused (not deleted: lv_refr_now() needs it).
 *   The panel is only redrawn when the user presses "Update" or the weather
 *   timer fires (every 15 minutes). LVGL renders on demand via lv_refr_now(),
 *   flush_cb uploads the frame and starts the waveform, and BUSY releases it.
 * - MONO uses FAST (partial) refresh for small updates with a FULL (flashy)
 *   refresh every N updates to restore DC balance. TRICOLOR only uses FULL
 *   (the RED plane occupies the reference-RAM address).
 *
 * SPDX-License-Identifier: CC0-1.0
 */
#include <assert.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "lvgl.h"

#include "esp_lcd_panel_ssd1681.h"
#include "lut_waveshare_154.h"

#define TAG "weather"

#define EPD_HOST       SPI2_HOST
#define PIN_SCLK       6
#define PIN_MOSI       7
#define PIN_DC         9
#define PIN_RST        4
#define PIN_CS         10
#define PIN_BUSY       18

#define EPD_WIDTH      200
#define EPD_HEIGHT     200
#define EPD_PIXEL_CLOCK (20 * 1000 * 1000)

/* Set to SSD1681_PANEL_TRICOLOR for a black/red/white panel. */
#define PANEL_TYPE     SSD1681_PANEL_MONO

/* Refresh budget: one full flashy refresh after every 3 fast updates.
 * Long-term partial-only refreshes cause ghosting and drifting grey levels. */
#define FAST_REFRESH_LIMIT 3
#define WEATHER_PERIOD_MS  (15 * 60 * 1000)

static lv_display_t *g_disp;
static esp_lcd_panel_handle_t g_panel;
static SemaphoreHandle_t g_refresh_done;
static lv_timer_t *g_weather_timer;
static lv_obj_t *g_temp_label;
static lv_obj_t *g_cond_label;
static lv_obj_t *g_time_label;
static int g_update_count;
static lv_timer_t *g_refresh_watch;
static esp_lcd_ssd1681_refresh_mode_t g_next_mode;

/* ---------------------------------------------------------------------------
 * LVGL port: I1 framebuffer + manual refresh
 * ------------------------------------------------------------------------- */

static bool IRAM_ATTR refresh_done_cb(esp_lcd_panel_handle_t panel, const void *edata, void *ctx)
{
    BaseType_t wake = pdFALSE;
    (void)panel;
    (void)edata;
    (void)ctx;
    xSemaphoreGiveFromISR(g_refresh_done, &wake);
    return wake == pdTRUE;
}

static void flush_cb(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map)
{
    /* LVGL stores the palette before the pixel bits: I1 = 2 entries (8 bytes),
     * I2 = 4 entries (16 bytes). Rows are MSB-first. */
    const uint8_t *bits = px_map + (PANEL_TYPE == SSD1681_PANEL_MONO ? 8 : 16);
    (void)disp;
    (void)area;  /* FULL render mode always flushes the whole screen */

    esp_err_t err = esp_lcd_panel_draw_bitmap(g_panel, 0, 0, EPD_WIDTH, EPD_HEIGHT, bits);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "draw_bitmap failed: %s", esp_err_to_name(err));
        lv_display_flush_ready(g_disp);
        return;
    }
    /* Start the waveform here. lv_refr_now() has already copied the frame;
     * delaying the 0x20 activate to another timer left the panel on a blank
     * RAM image if that timer never ran. */
    err = epaper_panel_refresh_screen(g_panel, g_next_mode);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "refresh failed: %s", esp_err_to_name(err));
        lv_display_flush_ready(g_disp);
    }
}

/* Runs from lv_timer_handler() after the BUSY ISR has signaled completion. */
static void refresh_done_task(lv_timer_t *timer)
{
    (void)timer;
    if (xSemaphoreTake(g_refresh_done, 0) != pdTRUE) {
        return;  /* panel is still refreshing */
    }
    lv_display_flush_ready(g_disp);
    ESP_LOGI(TAG, "refresh complete (image retained with no power)");
}

/* ---------------------------------------------------------------------------
 * "Weather station" UI: readable at arm's length, no animation, no gradients
 * ------------------------------------------------------------------------- */

static void update_readings(lv_timer_t *timer)
{
    static const char *const conditions[] = { "CLEAR", "CLOUDY", "RAIN" };
    static const int temps[] = { 23, 19, 14 };
    int i = g_update_count % 3;

    lv_label_set_text_fmt(g_temp_label, "%d C", temps[i]);
    lv_label_set_text(g_cond_label, conditions[i]);

    time_t now = time(NULL);
    struct tm *t = localtime(&now);
    lv_label_set_text_fmt(g_time_label, "%02d:%02d  %02d/%02d",
                          t->tm_hour, t->tm_min, t->tm_mon + 1, t->tm_mday);

    g_update_count++;
    ESP_LOGI(TAG, "update %d", g_update_count);
    /* The driver also enforces its own refresh budget; choosing FAST here
     * is safe because it silently falls back to FULL when needed. */
    g_next_mode = SSD1681_REFRESH_FAST;
    lv_refr_now(g_disp);
    (void)timer;
}

static void update_button_cb(lv_event_t *e)
{
    (void)e;
    update_readings(NULL);
}

static void build_ui(void)
{
    lv_obj_t *scr = lv_screen_active();
    lv_obj_set_style_bg_color(scr, lv_color_white(), 0);
    lv_obj_set_style_pad_all(scr, 8, 0);

    g_temp_label = lv_label_create(scr);
    lv_label_set_text(g_temp_label, "23 C");
    lv_obj_set_style_text_font(g_temp_label, &lv_font_montserrat_40, 0);
    lv_obj_set_style_text_color(g_temp_label, lv_color_black(), 0);
    lv_obj_align(g_temp_label, LV_ALIGN_TOP_LEFT, 8, 12);

    g_cond_label = lv_label_create(scr);
    lv_label_set_text(g_cond_label, "CLEAR");
    lv_obj_set_style_text_font(g_cond_label, &lv_font_montserrat_28, 0);
    if (PANEL_TYPE == SSD1681_PANEL_TRICOLOR) {
        /* Palette index 2 = red; shows the RED plane working end to end. */
        lv_obj_set_style_text_color(g_cond_label, lv_color_make(0xff, 0x00, 0x00), 0);
    } else {
        lv_obj_set_style_text_color(g_cond_label, lv_color_black(), 0);
    }
    lv_obj_align(g_cond_label, LV_ALIGN_TOP_RIGHT, -8, 20);

    g_time_label = lv_label_create(scr);
    lv_label_set_text(g_time_label, "--:--");
    lv_obj_set_style_text_font(g_time_label, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(g_time_label, lv_color_black(), 0);
    lv_obj_align(g_time_label, LV_ALIGN_BOTTOM_MID, 0, -8);

    lv_obj_t *btn = lv_button_create(scr);
    lv_obj_set_size(btn, 80, 32);
    lv_obj_align(btn, LV_ALIGN_BOTTOM_MID, 0, -32);
    lv_obj_add_event_cb(btn, update_button_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *label = lv_label_create(btn);
    lv_label_set_text(label, "UPDATE");
    lv_obj_center(label);

    lv_obj_t *note = lv_label_create(scr);
    lv_label_set_text(note, "Powered by EPD");
    lv_obj_set_style_text_font(note, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(note, lv_color_black(), 0);
    lv_obj_align(note, LV_ALIGN_TOP_MID, 0, 0);
}

/* ---------------------------------------------------------------------------
 * Hardware init
 * ------------------------------------------------------------------------- */

static void init_panel(void)
{
    ESP_ERROR_CHECK(spi_bus_initialize(EPD_HOST, &(spi_bus_config_t) {
        .sclk_io_num = PIN_SCLK,
        .mosi_io_num = PIN_MOSI,
        .miso_io_num = -1,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = EPD_WIDTH * EPD_HEIGHT / 4,
    }, SPI_DMA_CH_AUTO));

    esp_lcd_panel_io_handle_t io = NULL;
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)EPD_HOST, &(esp_lcd_panel_io_spi_config_t) {
        .dc_gpio_num = PIN_DC,
        .cs_gpio_num = PIN_CS,
        .pclk_hz = EPD_PIXEL_CLOCK,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
        .trans_queue_depth = 8,
    }, &io));

    esp_lcd_ssd1681_config_t panel_cfg = {
        .busy_gpio_num = PIN_BUSY,
        .type = PANEL_TYPE,
        .fast_lut = { .bytes = lut_waveshare_154_fast },
        .fast_refresh_limit = FAST_REFRESH_LIMIT,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_ssd1681(io, &(esp_lcd_panel_dev_config_t) {
        .reset_gpio_num = PIN_RST,
        .vendor_config = &panel_cfg,
    }, &g_panel));

    epaper_panel_callbacks_t cbs = { .on_epaper_refresh_done = refresh_done_cb };
    ESP_ERROR_CHECK(epaper_panel_register_event_callbacks(g_panel, &cbs, NULL));
    ESP_ERROR_CHECK(esp_lcd_panel_reset(g_panel));
    ESP_ERROR_CHECK(esp_lcd_panel_init(g_panel));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(g_panel, true));
}

static void init_lvgl(void)
{
    lv_init();
    /* One screen-sized indexed buffer; palette entries come first.
     * MONO (I1): 2 entries (8 bytes), 1 bit/pixel -> palette [black, white].
     * TRICOLOR (I2): 4 entries (16 bytes), 2 bits/pixel ->
     *   palette [white, black, red, white]; the driver maps index 2 to the
     *   RED plane. */
    const bool tri = (PANEL_TYPE == SSD1681_PANEL_TRICOLOR);
    const uint32_t buf_size = (tri ? 16 : 8) + EPD_WIDTH * EPD_HEIGHT / (tri ? 4 : 8);
    uint8_t *buf = heap_caps_calloc(1, buf_size, MALLOC_CAP_DMA);
    assert(buf);
    if (tri) {
        ((uint32_t *)buf)[0] = 0xffffffff;  /* white */
        ((uint32_t *)buf)[1] = 0xff000000;  /* black */
        ((uint32_t *)buf)[2] = 0xffff0000;  /* red (ARGB8888) */
        ((uint32_t *)buf)[3] = 0xffffffff;  /* white */
    } else {
        ((uint32_t *)buf)[0] = 0xff000000;  /* black */
        ((uint32_t *)buf)[1] = 0xffffffff;  /* white */
    }

    g_disp = lv_display_create(EPD_WIDTH, EPD_HEIGHT);
    lv_display_set_color_format(g_disp, tri ? LV_COLOR_FORMAT_I2 : LV_COLOR_FORMAT_I1);
    lv_display_set_buffers(g_disp, buf, NULL, buf_size, LV_DISPLAY_RENDER_MODE_FULL);
    lv_display_set_flush_cb(g_disp, flush_cb);

    /* Pause the periodic refresh timer so LVGL does not redraw at 30 Hz.
     * Do not delete it: lv_refr_now() is a no-op without a refresh timer. */
    lv_timer_pause(lv_display_get_refr_timer(g_disp));
    g_refresh_watch = lv_timer_create(refresh_done_task, 50, NULL);
}

void app_main(void)
{
    g_refresh_done = xSemaphoreCreateBinary();
    gpio_install_isr_service(0);
    init_panel();
    init_lvgl();
    build_ui();

    /* First frame: full refresh establishes a clean baseline. TRICOLOR always
     * uses full refresh (the driver also enforces this internally). */
    g_next_mode = SSD1681_REFRESH_FULL;
    lv_obj_invalidate(lv_screen_active());
    lv_refr_now(g_disp);

    /* Periodic weather updates. The display repaints only when this fires or
     * the user presses the button, never at LVGL's default 30/60 Hz. */
    g_weather_timer = lv_timer_create(update_readings, WEATHER_PERIOD_MS, NULL);

    while (1) {
        lv_timer_handler();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
