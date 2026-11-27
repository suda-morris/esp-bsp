/*
 * SPDX-FileCopyrightText: 2023-2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "esp_lcd_panel_dev.h"
#include "esp_lcd_panel_io.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SSD1681_LUT_SIZE 159  /**< Bytes of one SSD1681 waveform setting */

/**
 * @brief E-paper refresh mode
 *
 * Full and fast modes use different waveforms. FAST reuses the previous BW RAM
 * as the "old image" so the controller only drives pixels that change; it is
 * quicker but leaves a faint ghost and must be followed by a FULL refresh
 * every few updates (see @ref esp_lcd_ssd1681_config_t::fast_refresh_limit).
 */
typedef enum {
    SSD1681_REFRESH_FULL = 0,  /**< OTP/flash full waveform, every pixel is driven */
    SSD1681_REFRESH_FAST,      /**< MCU-supplied partial waveform, only changed pixels */
} esp_lcd_ssd1681_refresh_mode_t;

/**
 * @brief Panel color type; also selects the framebuffer format expected by draw_bitmap()
 *
 * MONO takes a 1bpp (LVGL I1) buffer: one bit per pixel, MSB first, 8 pixels
 * per byte. FULL and FAST refresh are both supported; RAM 0x26 is reserved as
 * the previous BW frame used by FAST (display mode 2).
 *
 * TRICOLOR takes a 2bpp (LVGL I2) palette-indexed buffer. The four palette
 * entries must be [white, black, red, white]; the driver splits each pixel
 * into a BW plane (0x24) and a RED plane (0x26) in one pass. SSD1681 has no
 * third plane to hold a reference frame, so FAST is not meaningful: a FAST
 * request is silently downgraded to FULL.
 */
typedef enum {
    SSD1681_PANEL_MONO = 0,  /**< Black/white panel, I1 1bpp buffer, FULL + FAST */
    SSD1681_PANEL_TRICOLOR,  /**< Black/red/white panel, I2 2bpp buffer, FULL only */
} esp_lcd_ssd1681_panel_type_t;

/**
 * @brief SSD1681 waveform setting, loaded as-is into the controller
 *
 * The 159 bytes are laid out exactly as the SSD1681 "Waveform Setting" table:
 * - [0..59]:   Voltage select codes (VS) for LUT0..LUT4, two 2-bit codes per byte.
 *              Code 00=VSS/DCVCOM, 01=VSH1, 10=VSL, 11=VSH2. The BW RAM bit
 *              selects LUT0 (black) or LUT1 (white), the RED RAM bit selects
 *              LUT2 (red) or LUT3 (same as LUT2).
 * - [60..143]: Phase lengths TP[0A..11D], state repeats SR[0AB..11CD] and
 *              group repeats RP[0..11]. All counts are N+1 frames/times.
 * - [144..149]: Frame rates FR[0..11], two 4-bit values per byte.
 * - [150..152]: XON bits, all-gate-on for each AB/CD sub-group.
 * - [153]: EOPT, waveform end option (0x22 normal, 0x07 keep source level).
 * - [154..158]: VGH, VSH1, VSH2, VSL and VCOM analog voltages.
 *
 * The waveform is temperature compensated by the panel OTP. By copying a known
 * good waveform (the built-in table of the Waveshare 1.54" V2 panel is used in
 * the example) instead of inventing one, the values stay physically valid. The
 * driver never fabricates voltages or timing.
 */
typedef struct {
    const uint8_t *bytes;  /**< Pointer to exactly 159 bytes, must remain valid while in use */
} esp_lcd_ssd1681_lut_t;

/**
 * @brief Additional configuration for the SSD1681 panel
 *
 * Pass as `esp_lcd_panel_dev_config_t::vendor_config`.
 */
typedef struct {
    int busy_gpio_num;                    /**< BUSY input pin (required for safe waveform completion) */
    esp_lcd_ssd1681_panel_type_t type;    /**< Panel color type; selects the framebuffer format and plane layout */
    esp_lcd_ssd1681_lut_t fast_lut;        /**< Partial/fast waveform; NULL uses the previous-table-friendly default; MONO only */
    uint8_t fast_refresh_limit;              /**< Force a full refresh after N fast refreshes (0 disables the counter) */
    uint8_t mux_height;                      /**< Gate output lines; leave 0 for the 200x200 panel */
} esp_lcd_ssd1681_config_t;

/**
 * @brief Callback invoked when the BUSY pin signals the end of a refresh
 *
 * Runs in ISR context on the GPIO ISR. Keep it short, no blocking calls.
 */
typedef bool (*esp_lcd_epaper_panel_cb_t)(esp_lcd_panel_handle_t panel, const void *edata, void *user_data);

typedef struct {
    esp_lcd_epaper_panel_cb_t on_epaper_refresh_done;  /**< Refresh-done callback, NULL to unregister */
} epaper_panel_callbacks_t;

/**
 * @brief Create an SSD1681 e-paper panel
 *
 * @note gpio_install_isr_service() must be called first when busy_gpio_num >= 0.
 */
esp_err_t esp_lcd_new_panel_ssd1681(const esp_lcd_panel_io_handle_t io,
                                    const esp_lcd_panel_dev_config_t *panel_dev_config,
                                    esp_lcd_panel_handle_t *ret_panel);

/**
 * @brief Send a cached frame to the panel and start a waveform
 *
 * Unlike a TFT, this only submits the command; the display keeps refreshing
 * for seconds after the call returns. Use `esp_lcd_ssd1681_wait_busy()` or
 * the refresh-done callback before touching the panel again.
 */
esp_err_t epaper_panel_refresh_screen(esp_lcd_panel_handle_t panel, esp_lcd_ssd1681_refresh_mode_t mode);

/**
 * @brief Block until the panel finishes the current refresh
 *
 * Forced to poll with a timeout even with no BUSY pin configured.
 */
esp_err_t esp_lcd_ssd1681_wait_busy(esp_lcd_panel_handle_t panel, uint32_t timeout_ms);

/**
 * @brief Replace the fast (partial) waveform
 */
esp_err_t epaper_panel_set_fast_lut(esp_lcd_panel_handle_t panel, const uint8_t *lut, size_t size);

/**
 * @brief Restore the OTP waveform by reloading it through the controller
 *
 * This resets the fast-refresh counter to zero.
 */
esp_err_t epaper_panel_load_otp_lut(esp_lcd_panel_handle_t panel);

/**
 * @brief Register a refresh-done callback (ISR context)
 */
esp_err_t epaper_panel_register_event_callbacks(esp_lcd_panel_handle_t panel, epaper_panel_callbacks_t *cbs, void *user_ctx);

#ifdef __cplusplus
}
#endif
