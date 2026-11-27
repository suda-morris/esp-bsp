/*
 * SPDX-FileCopyrightText: 2023-2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_check.h"
#include "esp_heap_caps.h"

#ifndef CONFIG_LCD_ENABLE_DEBUG_LOG
#define CONFIG_LCD_ENABLE_DEBUG_LOG 0
#endif
#if CONFIG_LCD_ENABLE_DEBUG_LOG
#define LOG_LOCAL_LEVEL ESP_LOG_DEBUG
#endif
#include "esp_log.h"

#include "esp_lcd_panel_interface.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ssd1681.h"
#include "esp_lcd_ssd1681_commands.h"

#define SSD1681_WIDTH      200
#define SSD1681_HEIGHT     200
#define SSD1681_FRAME_BYTES  (SSD1681_WIDTH * SSD1681_HEIGHT / 8)
#define SSD1681_BUSY_TIMEOUT_MS 15000

/* A safe, previous-table-friendly partial waveform for the Waveshare 1.54" V2
 * panel (GDEH0154D67). Same structure as the vendor's FAST_REFRESH_KEEP table:
 * it only pulses LUT0/LUT1 for one frame using the "keep" end option, so it
 * moves particles only where the new BW image differs from the old one. */
static const uint8_t s_default_fast_lut[SSD1681_LUT_SIZE] = {
    0x80, 0x48, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x40, 0x48, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x80, 0x48, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x40, 0x48, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x0a, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x00, 0x00, 0x00,
    0x07, 0x17, 0x41, 0x00, 0x32, 0x20
};

static const char *TAG = "ssd1681";

typedef struct {
    esp_lcd_panel_t base;
    esp_lcd_panel_io_handle_t io;
    int reset_gpio_num;
    bool reset_active_high;
    int busy_gpio_num;
    esp_lcd_ssd1681_panel_type_t type;
    uint8_t mux_height;
    uint8_t fast_refresh_limit;
    uint8_t fast_refresh_count;

    const uint8_t *fast_lut;
    /* Native (vertical bit order) planes. BW always maps to RAM 0x24.
     * MONO: second plane is the previous BW frame, written to RAM 0x26 as the
     *       FAST reference; TRICOLOR: second plane is RED, written to 0x26. */
    uint8_t *bw_ram;
    uint8_t *second_ram;

    esp_lcd_epaper_panel_cb_t refresh_done_cb;
    void *refresh_done_ctx;
    volatile bool busy;
    bool sleeping;
    bool suppress_callback;  /* internal (LUT load) sequence, not a user refresh */
} ssd1681_panel_t;

static esp_err_t panel_ssd1681_del(esp_lcd_panel_t *panel);
static esp_err_t panel_ssd1681_reset(esp_lcd_panel_t *panel);
static esp_err_t panel_ssd1681_init(esp_lcd_panel_t *panel);
static esp_err_t panel_ssd1681_draw_bitmap(esp_lcd_panel_t *panel, int x_start, int y_start,
        int x_end, int y_end, const void *color_data);
static esp_err_t ssd1681_write_second_plane(ssd1681_panel_t *panel);
static esp_err_t panel_ssd1681_invert_color(esp_lcd_panel_t *panel, bool invert);
static esp_err_t panel_ssd1681_mirror(esp_lcd_panel_t *panel, bool mirror_x, bool mirror_y);
static esp_err_t panel_ssd1681_swap_xy(esp_lcd_panel_t *panel, bool swap_axes);
static esp_err_t panel_ssd1681_set_gap(esp_lcd_panel_t *panel, int x_gap, int y_gap);
static esp_err_t panel_ssd1681_disp_on_off(esp_lcd_panel_t *panel, bool on_off);

static void busy_isr(void *arg);
static esp_err_t ssd1681_write_lut(ssd1681_panel_t *panel, const uint8_t *lut);
static esp_err_t ssd1681_wait_busy_locked(ssd1681_panel_t *panel, uint32_t timeout_ms);
static esp_err_t ssd1681_start_sequence(ssd1681_panel_t *panel, uint8_t seq);
static esp_err_t ssd1681_set_window(ssd1681_panel_t *panel, int x_start, int y_start, int x_end, int y_end);
static esp_err_t ssd1681_write_plane(ssd1681_panel_t *panel, uint8_t cmd, const uint8_t *data, size_t len);

/* SSD1681 data-entry mode 0x03 (AM=0): X updates first. Each byte is 8
 * horizontal pixels of one row, MSB = leftmost pixel. This matches LVGL I1. */
static inline uint8_t *native_addr(uint8_t *buf, int x, int y)
{
    return &buf[y * (SSD1681_WIDTH / 8) + (x >> 3)];
}

static inline void native_set(uint8_t *buf, int x, int y, bool value)
{
    uint8_t *p = native_addr(buf, x, y);
    const uint8_t mask = (uint8_t)(1u << (7 - (x & 7)));
    if (value) {
        *p |= mask;
    } else {
        *p &= (uint8_t)~mask;
    }
}

static void IRAM_ATTR busy_isr(void *arg)
{
    ssd1681_panel_t *panel = (ssd1681_panel_t *)arg;
    gpio_intr_disable(panel->busy_gpio_num);
    panel->busy = false;
    if (!panel->suppress_callback && panel->refresh_done_cb) {
        if (panel->refresh_done_cb(&panel->base, NULL, panel->refresh_done_ctx)) {
            portYIELD_FROM_ISR();
        }
    }
}

esp_err_t epaper_panel_register_event_callbacks(esp_lcd_panel_handle_t panel, epaper_panel_callbacks_t *cbs, void *user_ctx)
{
    ESP_RETURN_ON_FALSE(panel && cbs, ESP_ERR_INVALID_ARG, TAG, "invalid argument");
    ssd1681_panel_t *p = __containerof(panel, ssd1681_panel_t, base);
    p->refresh_done_cb = cbs->on_epaper_refresh_done;
    p->refresh_done_ctx = user_ctx;
    return ESP_OK;
}

/* Submit a display-update sequence. Enables the BUSY interrupt only around
 * the activation so the ISR cannot fire for unrelated SPI traffic. */
static esp_err_t ssd1681_start_sequence(ssd1681_panel_t *panel, uint8_t seq)
{
    if (panel->busy_gpio_num >= 0) {
        panel->busy = true;
        gpio_intr_enable(panel->busy_gpio_num);
    }
    esp_err_t err = esp_lcd_panel_io_tx_param(panel->io, SSD1681_CMD_DISP_UPDATE_CTRL2, &seq, 1);
    if (err == ESP_OK) {
        err = esp_lcd_panel_io_tx_param(panel->io, SSD1681_CMD_MASTER_ACTIVATE, NULL, 0);
    }
    if (err != ESP_OK) {
        panel->busy = false;
        if (panel->busy_gpio_num >= 0) {
            gpio_intr_disable(panel->busy_gpio_num);
        }
    }
    return err;
}

static esp_err_t ssd1681_write_lut(ssd1681_panel_t *panel, const uint8_t *lut)
{
    esp_lcd_panel_io_handle_t io = panel->io;
    /* Bytes 0..152 belong to command 0x32 (VS, TP, SR, RP, FR, XON); bytes
     * 153..158 belong to four separate commands. Sending all 159 bytes to
     * 0x32 would silently truncate the analog settings. */
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, SSD1681_CMD_SET_LUT_REG, lut, 153), TAG, "write waveform");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, SSD1681_CMD_SET_END_OPTION, &lut[153], 1), TAG, "write EOPT");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, SSD1681_CMD_SET_GATE_VOLTAGE, &lut[154], 1), TAG, "write VGH");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, SSD1681_CMD_SET_SRC_VOLTAGE, &lut[155], 3), TAG, "write VSH");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, SSD1681_CMD_SET_VCOM, &lut[158], 1), TAG, "write VCOM");
    return ESP_OK;
}

esp_err_t epaper_panel_set_fast_lut(esp_lcd_panel_handle_t panel, const uint8_t *lut, size_t size)
{
    ESP_RETURN_ON_FALSE(panel && lut && size == SSD1681_LUT_SIZE, ESP_ERR_INVALID_ARG, TAG, "invalid LUT");
    ssd1681_panel_t *p = __containerof(panel, ssd1681_panel_t, base);
    p->fast_lut = lut;
    p->fast_refresh_count = 0;
    return ESP_OK;
}

esp_err_t epaper_panel_load_otp_lut(esp_lcd_panel_handle_t panel)
{
    ESP_RETURN_ON_FALSE(panel, ESP_ERR_INVALID_ARG, TAG, "invalid panel");
    ssd1681_panel_t *p = __containerof(panel, ssd1681_panel_t, base);
    /* 0xb1 = enable clock + load temperature + load OTP LUT mode 1 + disable clock.
     * This is itself a BUSY sequence; suppress the user callback so it is not
     * mistaken for the end of a display refresh. */
    p->suppress_callback = true;
    esp_err_t err = ssd1681_start_sequence(p, SSD1681_PARAM_LOAD_OTP_LUT);
    if (err == ESP_OK) {
        err = ssd1681_wait_busy_locked(p, SSD1681_BUSY_TIMEOUT_MS);
    }
    p->suppress_callback = false;
    return err;
}

static esp_err_t ssd1681_wait_busy_locked(ssd1681_panel_t *panel, uint32_t timeout_ms)
{
    /* SSD1681 drives BUSY high while working. Poll the pin like the vendor
     * code: relying only on the ISR misses the edge if BUSY never went high,
     * and a pull-up on an open-drain line can keep the flag stuck. */
    TickType_t start = xTaskGetTickCount();
    vTaskDelay(pdMS_TO_TICKS(1));
    while (panel->busy_gpio_num >= 0 && gpio_get_level(panel->busy_gpio_num)) {
        if ((xTaskGetTickCount() - start) > pdMS_TO_TICKS(timeout_ms)) {
            ESP_LOGE(TAG, "refresh timed out");
            panel->busy = false;
            if (panel->busy_gpio_num >= 0) {
                gpio_intr_disable(panel->busy_gpio_num);
            }
            return ESP_ERR_TIMEOUT;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    panel->busy = false;
    return ESP_OK;
}

esp_err_t esp_lcd_ssd1681_wait_busy(esp_lcd_panel_handle_t panel, uint32_t timeout_ms)
{
    ESP_RETURN_ON_FALSE(panel, ESP_ERR_INVALID_ARG, TAG, "invalid panel");
    ssd1681_panel_t *p = __containerof(panel, ssd1681_panel_t, base);
    return ssd1681_wait_busy_locked(p, timeout_ms);
}

static esp_err_t ssd1681_set_window(ssd1681_panel_t *panel, int x_start, int y_start, int x_end, int y_end)
{
    esp_lcd_panel_io_handle_t io = panel->io;
    /* X coordinates are byte addresses; require byte-aligned requested areas. */
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, SSD1681_CMD_SET_RAMX_START_END,
                        (uint8_t[]){ (uint8_t)(x_start >> 3), (uint8_t)(x_end >> 3) }, 2), TAG, "RAM X window");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, SSD1681_CMD_SET_RAMY_START_END,
                        (uint8_t[]){ (uint8_t)(y_start & 0xff), (uint8_t)(y_start >> 8),
                                     (uint8_t)(y_end & 0xff), (uint8_t)(y_end >> 8) }, 4), TAG, "RAM Y window");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, SSD1681_CMD_SET_INIT_X_ADDR,
                        (uint8_t[]){ (uint8_t)(x_start >> 3) }, 1), TAG, "RAM X cursor");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, SSD1681_CMD_SET_INIT_Y_ADDR,
                        (uint8_t[]){ (uint8_t)(y_start & 0xff), (uint8_t)(y_start >> 8) }, 2), TAG, "RAM Y cursor");
    return ESP_OK;
}

static esp_err_t ssd1681_write_plane(ssd1681_panel_t *panel, uint8_t cmd, const uint8_t *data, size_t len)
{
    /* esp_lcd_panel_io_tx_color() may use DMA; the native planes are allocated
     * from DMA-capable memory so no bounce buffer is needed. */
    return esp_lcd_panel_io_tx_color(panel->io, cmd, data, len);
}

/* MONO: RAM 0x26 holds the previous BW frame (FAST display-mode-2 reference).
 * TRICOLOR: RAM 0x26 is the RED plane; there is no previous frame, so only
 * FULL refresh is used. Either way callers never manage 0x26 by hand. */
static esp_err_t ssd1681_write_second_plane(ssd1681_panel_t *panel)
{
    return ssd1681_write_plane(panel, SSD1681_CMD_WRITE_RED_RAM, panel->second_ram, SSD1681_FRAME_BYTES);
}

esp_err_t epaper_panel_refresh_screen(esp_lcd_panel_handle_t panel, esp_lcd_ssd1681_refresh_mode_t mode)
{
    ESP_RETURN_ON_FALSE(panel, ESP_ERR_INVALID_ARG, TAG, "invalid panel");
    ssd1681_panel_t *p = __containerof(panel, ssd1681_panel_t, base);
    ESP_RETURN_ON_FALSE(!p->sleeping, ESP_ERR_INVALID_STATE, TAG, "panel is asleep");
    ESP_RETURN_ON_FALSE(!p->busy, ESP_ERR_NOT_FINISHED, TAG, "previous refresh not finished");

    /* TRICOLOR has no spare plane for a reference frame: 0x26 is the RED
     * plane. Partial refresh is impossible, so silently use the OTP FULL
     * waveform (which drives LUT2 for red pixels). */
    bool use_fast = (mode == SSD1681_REFRESH_FAST) && (p->type == SSD1681_PANEL_MONO);
    if (use_fast && p->fast_refresh_limit && p->fast_refresh_count >= p->fast_refresh_limit) {
        /* Too many partial refreshes in a row would grey out the film. */
        ESP_LOGI(TAG, "partial refresh budget exhausted, doing full refresh");
        use_fast = false;
    }

    if (use_fast) {
        ESP_RETURN_ON_ERROR(ssd1681_write_lut(p, p->fast_lut), TAG, "write fast LUT");
        p->fast_refresh_count++;
    } else {
        /* 0xF7 reloads the temperature-selected OTP waveform and displays it.
         * Do not insert a separate 0xB1 wait here: if that wait failed, the
         * actual display sequence never ran and the panel stayed blank. */
        p->fast_refresh_count = 0;
    }

    /* 0xCF = clock + analog + display mode 2 (RAM 0x26 is the previous image).
     * 0xF7 = clock + analog + load OTP LUT + display mode 1 (full flash). */
    const uint8_t update_seq = use_fast ? SSD1681_PARAM_DISPLAY_MODE_2_PARTIAL : SSD1681_PARAM_DISPLAY_MODE_1;
    return ssd1681_start_sequence(p, update_seq);
}

esp_err_t esp_lcd_new_panel_ssd1681(const esp_lcd_panel_io_handle_t io,
                                    const esp_lcd_panel_dev_config_t *panel_dev_config,
                                    esp_lcd_panel_handle_t *ret_panel)
{
    ESP_RETURN_ON_FALSE(io && panel_dev_config && ret_panel && panel_dev_config->vendor_config,
                        ESP_ERR_INVALID_ARG, TAG, "invalid argument");
    const esp_lcd_ssd1681_config_t *cfg = panel_dev_config->vendor_config;
    ESP_RETURN_ON_FALSE(cfg->busy_gpio_num >= 0, ESP_ERR_NOT_SUPPORTED, TAG,
                        "BUSY pin is required for safe waveform completion");
    ESP_RETURN_ON_FALSE(cfg->type == SSD1681_PANEL_MONO || cfg->type == SSD1681_PANEL_TRICOLOR,
                        ESP_ERR_INVALID_ARG, TAG, "invalid panel type");

    ssd1681_panel_t *p = heap_caps_calloc(1, sizeof(ssd1681_panel_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    ESP_RETURN_ON_FALSE(p, ESP_ERR_NO_MEM, TAG, "no memory for panel");
    p->io = io;
    p->reset_gpio_num = panel_dev_config->reset_gpio_num;
    p->reset_active_high = panel_dev_config->flags.reset_active_high;
    p->busy_gpio_num = cfg->busy_gpio_num;
    p->type = cfg->type;
    p->mux_height = cfg->mux_height ? cfg->mux_height : SSD1681_HEIGHT;
    p->fast_refresh_limit = cfg->fast_refresh_limit;
    p->fast_lut = cfg->fast_lut.bytes ? cfg->fast_lut.bytes : s_default_fast_lut;
    p->busy = false;
    p->sleeping = false;

    p->bw_ram = heap_caps_calloc(1, SSD1681_FRAME_BYTES, MALLOC_CAP_DMA);
    p->second_ram = heap_caps_calloc(1, SSD1681_FRAME_BYTES, MALLOC_CAP_DMA);
    if (!p->bw_ram || !p->second_ram) {
        free(p->bw_ram);
        free(p->second_ram);
        free(p);
        ESP_LOGE(TAG, "no memory for frame planes");
        return ESP_ERR_NO_MEM;
    }

    if (p->reset_gpio_num >= 0) {
        gpio_config_t rst = {
            .pin_bit_mask = 1ULL << p->reset_gpio_num,
            .mode = GPIO_MODE_OUTPUT,
        };
        esp_err_t err = gpio_config(&rst);
        if (err != ESP_OK) {
            free(p->bw_ram); free(p->second_ram); free(p);
            return err;
        }
    }
    gpio_config_t busy = {
        .pin_bit_mask = 1ULL << p->busy_gpio_num,
        .mode = GPIO_MODE_INPUT,
        /* Idle is low. A pull-up would pin an open-drain BUSY high and the
         * driver would wait forever / never see the falling edge. */
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_NEGEDGE,
    };
    esp_err_t err = gpio_config(&busy);
    if (err != ESP_OK) {
        if (p->reset_gpio_num >= 0) {
            gpio_reset_pin(p->reset_gpio_num);
        }
        free(p->bw_ram); free(p->second_ram); free(p);
        return err;
    }
    err = gpio_isr_handler_add(p->busy_gpio_num, busy_isr, p);
    if (err != ESP_OK) {
        gpio_reset_pin(p->busy_gpio_num);
        if (p->reset_gpio_num >= 0) {
            gpio_reset_pin(p->reset_gpio_num);
        }
        free(p->bw_ram); free(p->second_ram); free(p);
        return err;
    }
    gpio_intr_disable(p->busy_gpio_num);

    p->base.del = panel_ssd1681_del;
    p->base.reset = panel_ssd1681_reset;
    p->base.init = panel_ssd1681_init;
    p->base.draw_bitmap = panel_ssd1681_draw_bitmap;
    p->base.invert_color = panel_ssd1681_invert_color;
    p->base.mirror = panel_ssd1681_mirror;
    p->base.swap_xy = panel_ssd1681_swap_xy;
    p->base.set_gap = panel_ssd1681_set_gap;
    p->base.disp_on_off = panel_ssd1681_disp_on_off;
    *ret_panel = &p->base;
    return ESP_OK;
}

static esp_err_t panel_ssd1681_del(esp_lcd_panel_t *panel)
{
    ssd1681_panel_t *p = __containerof(panel, ssd1681_panel_t, base);
    if (p->busy_gpio_num >= 0) {
        gpio_isr_handler_remove(p->busy_gpio_num);
        gpio_reset_pin(p->busy_gpio_num);
    }
    if (p->reset_gpio_num >= 0) {
        gpio_reset_pin(p->reset_gpio_num);
    }
    free(p->bw_ram);
    free(p->second_ram);
    free(p);
    return ESP_OK;
}

static esp_err_t panel_ssd1681_reset(esp_lcd_panel_t *panel)
{
    ssd1681_panel_t *p = __containerof(panel, ssd1681_panel_t, base);
    if (p->reset_gpio_num >= 0) {
        gpio_set_level(p->reset_gpio_num, p->reset_active_high);
        vTaskDelay(pdMS_TO_TICKS(10));
        gpio_set_level(p->reset_gpio_num, !p->reset_active_high);
    } else {
        esp_lcd_panel_io_tx_param(p->io, SSD1681_CMD_SWRST, NULL, 0);
    }
    vTaskDelay(pdMS_TO_TICKS(10));
    return ssd1681_wait_busy_locked(p, SSD1681_BUSY_TIMEOUT_MS);
}

static esp_err_t panel_ssd1681_init(esp_lcd_panel_t *panel)
{
    ssd1681_panel_t *p = __containerof(panel, ssd1681_panel_t, base);
    esp_lcd_panel_io_handle_t io = p->io;
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, SSD1681_CMD_SWRST, NULL, 0), TAG, "soft reset");
    ESP_RETURN_ON_ERROR(ssd1681_wait_busy_locked(p, SSD1681_BUSY_TIMEOUT_MS), TAG, "wait after soft reset");
    p->busy = false;
    p->sleeping = false;

    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, SSD1681_CMD_OUTPUT_CTRL,
                        (uint8_t[]){ (uint8_t)(p->mux_height - 1), 0x00, 0x00 }, 3), TAG, "driver output");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, SSD1681_CMD_DATA_ENTRY_MODE,
                        (uint8_t[]){ SSD1681_PARAM_DATA_ENTRY_MODE_Y_X }, 1), TAG, "data entry");
    ESP_RETURN_ON_ERROR(ssd1681_set_window(p, 0, 0, SSD1681_WIDTH - 1, SSD1681_HEIGHT - 1), TAG, "full RAM window");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, SSD1681_CMD_SET_BORDER_WAVEFORM,
                        (uint8_t[]){ SSD1681_PARAM_BORDER_WAVEFORM }, 1), TAG, "border waveform");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, SSD1681_CMD_SET_TEMP_SENSOR,
                        (uint8_t[]){ 0x80 }, 1), TAG, "internal temperature sensor");

    /* Boot to a clean white image: BW = 1 (LUT1 white), second plane = 0
     * (RED off; for MONO this is the previous frame, 1 = white). */
    memset(p->bw_ram, 0xff, SSD1681_FRAME_BYTES);
    memset(p->second_ram, p->type == SSD1681_PANEL_MONO ? 0xff : 0x00, SSD1681_FRAME_BYTES);
    ESP_RETURN_ON_ERROR(ssd1681_write_plane(p, SSD1681_CMD_WRITE_BW_RAM, p->bw_ram, SSD1681_FRAME_BYTES),
                        TAG, "clear BW RAM");
    ESP_RETURN_ON_ERROR(ssd1681_write_second_plane(p), TAG, "clear second RAM");
    p->fast_refresh_count = 0;
    return ESP_OK;
}

static esp_err_t panel_ssd1681_draw_bitmap(esp_lcd_panel_t *panel, int x_start, int y_start,
        int x_end, int y_end, const void *color_data)
{
    ssd1681_panel_t *p = __containerof(panel, ssd1681_panel_t, base);
    ESP_RETURN_ON_FALSE(color_data, ESP_ERR_INVALID_ARG, TAG, "null bitmap");
    ESP_RETURN_ON_FALSE(!p->busy && !p->sleeping, ESP_ERR_INVALID_STATE, TAG, "panel busy or asleep");
    ESP_RETURN_ON_FALSE(x_start >= 0 && y_start >= 0 && x_end <= SSD1681_WIDTH && y_end <= SSD1681_HEIGHT
                        && x_start < x_end && y_start < y_end, ESP_ERR_INVALID_ARG, TAG, "invalid area");
    const uint8_t *src = color_data;
    int w = x_end - x_start;
    if (p->type == SSD1681_PANEL_MONO) {
        /* LVGL I1: MSB first, 8 pixels per byte. The example sets palette
         * [black, white], so index 1 = white; BW RAM 1 = white (LUT1). */
        for (int y = y_start; y < y_end; y++) {
            for (int x = x_start; x < x_end; x++) {
                int sx = x - x_start;
                int sy = y - y_start;
                uint8_t bit = (src[sy * ((w + 7) / 8) + (sx >> 3)] >> (7 - (sx & 7))) & 1;
                native_set(p->bw_ram, x, y, bit != 0);
            }
        }
    } else {
        /* LVGL I2: MSB first, four 2-bit indices per byte. Palette must be
         * [white, black, red, white]. Map index -> (BW, RED) RAM bits:
         *   black(1) -> (0,0)=LUT0, red(2) -> (0,1)=LUT2,
         *   white(0/3) -> (1,0)=LUT1. Both planes are written in this pass,
         *   so a red pixel is never left as a stale ghost in the BW plane. */
        size_t stride = ((size_t)w * 2 + 3) / 4;  /* bytes per row of 2bpp indices */
        for (int y = y_start; y < y_end; y++) {
            for (int x = x_start; x < x_end; x++) {
                int sx = x - x_start;
                int sy = y - y_start;
                uint8_t idx = (src[sy * stride + (sx >> 2)] >> (6 - ((sx & 3) << 1))) & 0x3;
                native_set(p->bw_ram, x, y, idx != 1 && idx != 2);
                native_set(p->second_ram, x, y, idx == 2);
            }
        }
    }

    /* Upload both full planes with a full RAM window. MONO puts the last
     * displayed BW frame in 0x26 as the FAST reference; TRICOLOR puts RED in
     * 0x26 and relies on the OTP waveform's LUT2 for red. */
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(p->io, SSD1681_CMD_DATA_ENTRY_MODE,
                        (uint8_t[]){ SSD1681_PARAM_DATA_ENTRY_MODE_Y_X }, 1), TAG, "data entry");
    ESP_RETURN_ON_ERROR(ssd1681_set_window(p, 0, 0, SSD1681_WIDTH - 1, SSD1681_HEIGHT - 1), TAG, "set RAM window");
    ESP_RETURN_ON_ERROR(ssd1681_write_plane(p, SSD1681_CMD_WRITE_BW_RAM, p->bw_ram, SSD1681_FRAME_BYTES),
                        TAG, "write BW plane");
    ESP_RETURN_ON_ERROR(ssd1681_write_second_plane(p), TAG, "write second plane");
    if (p->type == SSD1681_PANEL_MONO) {
        memcpy(p->second_ram, p->bw_ram, SSD1681_FRAME_BYTES);  /* current frame becomes next reference */
    }
    return ESP_OK;
}

static esp_err_t panel_ssd1681_invert_color(esp_lcd_panel_t *panel, bool invert)
{
    ssd1681_panel_t *p = __containerof(panel, ssd1681_panel_t, base);
    uint8_t cfg = invert ? (SSD1681_PARAM_INVERSE_BW | SSD1681_PARAM_INVERSE_RED) : 0;
    return esp_lcd_panel_io_tx_param(p->io, SSD1681_CMD_DISP_UPDATE_CTRL1, &cfg, 1);
}

static esp_err_t panel_ssd1681_mirror(esp_lcd_panel_t *panel, bool mirror_x, bool mirror_y)
{
    ESP_RETURN_ON_FALSE(!mirror_x && !mirror_y, ESP_ERR_NOT_SUPPORTED, TAG,
                        "hardware mirroring is not supported; rotate in LVGL instead");
    return ESP_OK;
}

static esp_err_t panel_ssd1681_swap_xy(esp_lcd_panel_t *panel, bool swap_axes)
{
    ESP_RETURN_ON_FALSE(!swap_axes, ESP_ERR_NOT_SUPPORTED, TAG,
                        "hardware axis swap is not supported; rotate in LVGL instead");
    return ESP_OK;
}

static esp_err_t panel_ssd1681_set_gap(esp_lcd_panel_t *panel, int x_gap, int y_gap)
{
    ESP_RETURN_ON_FALSE(x_gap == 0 && y_gap == 0, ESP_ERR_NOT_SUPPORTED, TAG, "gaps are not supported");
    return ESP_OK;
}

static esp_err_t panel_ssd1681_disp_on_off(esp_lcd_panel_t *panel, bool on_off)
{
    ssd1681_panel_t *p = __containerof(panel, ssd1681_panel_t, base);
    if (on_off) {
        /* Datasheet: deep sleep can only be exited with a hardware reset. */
        if (p->sleeping) {
            ESP_RETURN_ON_ERROR(panel_ssd1681_reset(panel), TAG, "wake reset");
            return panel_ssd1681_init(panel);
        }
        /* Already awake. OTP waveform is loaded as part of a FULL refresh (0xF7). */
        return ESP_OK;
    }
    /* Deep Sleep Mode 1 keeps RAM. Waking requires a hardware reset followed
     * by init(); callers must re-send their frame afterwards. Prefer leaving
     * the panel idle between short-interval updates. */
    p->sleeping = true;
    return esp_lcd_panel_io_tx_param(p->io, SSD1681_CMD_SLEEP,
                                     (uint8_t[]){ SSD1681_PARAM_SLEEP_MODE_1 }, 1);
}
