# SSD1681 e-Paper driver (rewritten)

A small, explicit driver for SSD1681-based black/white/red e-paper panels
(200x200, SPI), implemented on top of `esp_lcd_panel_io`. It is designed for
*low-refresh displays*, not for emulating a TFT.

Design goals:

- Full and partial (fast) refresh are first-class APIs with the correct
  waveforms and wiring.
- One driver covers both black/white and black/red/white panels. MONO takes an
  LVGL I1 (1bpp) buffer; TRICOLOR takes I2 (2bpp) and splits it into the BW
  (0x24) and RED (0x26) planes in one pass. No RGB565 -> 1bpp conversion ever
  happens in the driver, and red is never left as a stale ghost.
- Refresh happens on demand: the LVGL refresh timer is deleted, a frame is
  rendered with `lv_refr_now()`, the waveform is started, and the BUSY pin
  tells us when it is safe to continue.
- Long-term reliability: forced full refresh after a configurable number of
  partial refreshes, deep sleep for storage, correct OTP waveform reload.

## API in 60 seconds

```c
esp_lcd_ssd1681_config_t cfg = {
    .busy_gpio_num = 18,
    .type = SSD1681_PANEL_MONO,          // or SSD1681_PANEL_TRICOLOR
    .fast_lut = { .bytes = my_waveshare_fast_lut }, // optional, MONO only, 159 bytes
    .fast_refresh_limit = 3,             // full refresh every 4th update
};
esp_lcd_panel_dev_config_t dev = {
    .reset_gpio_num = 4,
    .vendor_config = &cfg,
};
esp_lcd_new_panel_ssd1681(io, &dev, &panel);
epaper_panel_register_event_callbacks(panel, &cbs, ctx);
esp_lcd_panel_reset(panel);
esp_lcd_panel_init(panel);

// Fill the framebuffer, send it to the controller RAM:
esp_lcd_panel_draw_bitmap(panel, 0, 0, 200, 200, i1_bits);
// Start the waveform (returns immediately):
epaper_panel_refresh_screen(panel, SSD1681_REFRESH_FAST);
// ... wait for the BUSY interrupt or esp_lcd_ssd1681_wait_busy() ...
// image stays on the glass with no power; sleep only for long storage:
// esp_lcd_panel_disp_on_off(panel, false);
```

A complete LVGL 9.6 weather-station example lives in
`examples/epaper_weather_station/`. Set `PANEL_TYPE` in its `main.c` to
`SSD1681_PANEL_MONO` (I1, FAST + FULL) or `SSD1681_PANEL_TRICOLOR`
(I2 palette `[white, black, red, white]`, FULL only) to drive either panel.

### Panel types and framebuffers

| `type` | Buffer passed to `draw_bitmap()` | RAM planes | Refresh |
|--------|----------------------------------|-----------|---------|
| `SSD1681_PANEL_MONO` | LVGL I1, 1 bit/pixel, palette `[black, white]` | 0x24 = BW, 0x26 = previous BW frame | FULL + FAST |
| `SSD1681_PANEL_TRICOLOR` | LVGL I2, 2 bits/pixel, palette `[white, black, red, white]` | 0x24 = BW, 0x26 = RED | FULL only (FAST downgraded) |

For a 200x200 panel the buffer is 5 KB in MONO and 10 KB in TRICOLOR -- both
far below the 80 KB an RGB565 frame would need. The 2bpp index maps to the
planes as: black(1) -> BW=0/RED=0 (LUT0), red(2) -> BW=0/RED=1 (LUT2),
white(0/3) -> BW=1/RED=0 (LUT1). SSD1681 has no third plane to hold a
reference frame, so partial refresh is not possible on tri-color panels; the
driver silently uses the OTP waveform instead (as GxEPD2 does for 3-color
panels).

## LUT: what the 159 bytes actually mean

The SSD1681 does not "drive pixels" like an LCD. It applies a *waveform*: a
sequence of voltage pulses on the segment (source) and common (VCOM)
electrodes. The pigment particles in each microcapsule migrate towards one
electrode; after the pulses the pixel is optically stable with **zero power**.
This is why e-paper keeps an image when unplugged, and why a refresh takes
hundreds of milliseconds to seconds.

A waveform must be matched to the panel chemistry, the controller voltage
ranges and the ambient temperature. The SSD1681 stores several waveform
settings (WS) in its OTP, each associated with a temperature range. The
controller samples the on-chip sensor (or an external one) and automatically
selects the matching WS. That is the *correct* source of waveform data; a
driver should not invent its own.

The 159-byte "LUT" mirrors the WS layout from the SSD1681 datasheet (rev 0.13,
section 6.7, Figure 6-6):

| Bytes | Register | Meaning |
|-------|----------|---------|
| 0..59 | 0x32 | `VS[nX-LUTm]`: 4 voltage-level codes per byte, 12 groups x 4 phases x 5 LUTs. Codes: 00=VSS/DCVCOM, 01=VSH1, 10=VSL, 11=VSH2. |
| 60..143 | 0x32 | Timing: `TP[nA..nD]` phase lengths (frames, N+1), `SR[nAB]/SR[nCD]` sub-group repeats, `RP[n]` group repeats. All values are N+1. |
| 144..149 | 0x32 | `FR[0..11]`: frame rate per group (two 4-bit values per byte). |
| 150..152 | 0x32 | `XON[0AB..11CD]`: all-gate-on selection per sub-group. |
| 153 | 0x3F | EOPT: `0x22` normal; `0x07` keep source output before power-off. |
| 154 | 0x03 | VGH (gate high voltage, 0x17 = 20 V). |
| 155..157 | 0x04 | VSH1, VSH2, VSL (source voltages). |
| 158 | 0x2C | VCOM (0x20 = -0.8 V). |

`VS[nX-LUTm]` is where the "secret sauce" lives. The image RAM bit selects a
LUT: LUT0=black, LUT1=white, LUT2=red (LUT3 mirrors LUT2). For each phase of
each group, the LUT tells the controller which of the four source levels to
drive. Combined with the phase lengths and repeats, that defines the exact
particle trajectory. A full waveform typically has 10-30 frames with
carefully tuned durations: early frames shake particles loose, middle frames
move them, later frames settle them. The OTP contains separate waveforms for
cold, room and hot temperature because particle mobility changes strongly
with temperature.

**Practical advice**: copy a known-good waveform for your exact panel (the
Waveshare/GDEH0154D67 vendor table is supplied in the example, and epdiy
ships per-panel waveforms). Never tweak single bytes blindly: the voltage
codes are only meaningful together with the analog settings at bytes 154-158.

### Full vs fast refresh

- `SSD1681_REFRESH_FULL` issues command sequence `0xF7` (load temperature,
  display with OTP waveform, mode 1). Every pixel is driven through the whole
  waveform: high contrast, no ghosting, slow (1-2 s) and visible flashing.
- `SSD1681_REFRESH_FAST` writes a short MCU-supplied waveform and issues
  sequence `0xCF` (display mode 2, uses the previously written BW plane as the
  "old image" reference). Only changed pixels receive pulses: fast, no
  flash, but a faint ghost remains. After N partial refreshes the driver
  automatically falls back to a full refresh to restore DC balance and
  contrast. The Waveshare "KEEP" variant ends with EOPT=0x07 to avoid the
  greyish whites caused by cutting source output abruptly; it requires the
  panel to be powered down (or left in its idle, non-switching state).
  FAST is MONO-only: on a TRICOLOR panel `0x26` holds the RED plane, so a
  FAST request is downgraded to FULL.

## Why the old driver needed a rewrite

| Old code | Problem | Now |
|----------|---------|-----|
| `draw_bitmap()` with 4 mirror/swap branches and `process_bitmap()` | 200+ lines of untestable index math; broken for non-8-aligned areas; no rotation test coverage | One native RAM layout; rotation must be done in LVGL (documented), 1bpp horizontal input converted explicitly |
| `full_refresh` flag declared but never acted on | Misleading API | `refresh(mode)` with real OTP vs MCU waveforms |
| LUT sent 159 bytes to command 0x32 | Analog bytes 153-158 were silently dropped; "keep" EOPT never applied | 153 bytes to 0x32, 6 bytes to their own registers |
| `disp_on_off(true)` said to "restore LUT" but did not reload it | Custom LUT persisted forever | `disp_on_off(true)` reloads OTP waveform after wake reset |
| `set_bitmap_color()` wrote only one plane; red never cleared | Ghost red pixels | BW and RED planes are split from one palette buffer and written together on every draw |
| ISR callback assumed busy edge came from refresh only | False positives during init | Interrupt enabled only around a waveform |
| `busy` GPIO forced high on deep sleep; `wait_busy()` polled forever after sleep | Driver deadlock | Explicit busy state machine with timeout, reset-on-wake |
| LVGL 8 RGB565 -> mono conversion in `flush_cb` | Wasteful, color inversion bugs, 2x 32 KB buffers | LVGL 9.6 I1 palette-indexed buffer passed straight through |

The following were deliberately not carried over: `non_copy_mode` hacks,
`byte_reverse()` LUT, `set_bitmap_color()`, `set_custom_lut()` (replaced by
`set_fast_lut()`), and the `img_bitmap.c` demo asset.

## Hard-won rules for e-paper

1. **Refresh rate is a chemistry problem.** Do not update more often than
   necessary. The weather-station example updates every 15 minutes; the panel
   sleeps at ~0.7 uA in between.
2. **Always finish the waveform.** BUSY high means the high-voltage charge
   pump is active. Do not write RAM, enter deep sleep, or toggle reset while
   BUSY is high. The driver gates all operations on its busy state.
3. **Partial refreshes need a budget.** A few dozen fast updates without a
   full refresh will permanently grey the display. `fast_refresh_limit` is
   not optional for a product.
4. **Temperature matters.** The OTP waveform is selected by the controller's
   temperature sensor. For panels without a sensor, use the external sensor
   interface and load a matching waveform at startup.
5. **Power sequencing.** The image is bistable and uses no power once the
   waveform ends. There is no need to keep the controller's charge pump
   running between updates. Use `disp_on_off(false)` (deep sleep 1, RAM kept)
   before long storage; waking needs a hardware reset plus `init()`.

## Relationship to epdiy / FastEPD / GxEPD2

- [epdiy](https://github.com/vroland/epdiy) uses per-panel *waveform files*
  with multiple temperature ranges and draw modes (GC16, GL16, DU), then
  builds the output LUT in software for a parallel e-paper controller. Our
  driver delegates waveform selection to the SSD1681 OTP; the API shape
  (`fast_lut`, `refresh(mode)`) is intentionally close so panels can later be
  described by a waveform table rather than hard-coded numbers.
- [FastEPD](https://github.com/bitbank2/FastEPD) shows how much performance
  lies in the output stage (I2S + cached line buffers). Our SPI panel IO uses
  DMA; the significant optimization here is avoiding per-pixel conversion in
  the flush path.
- [GxEPD2](https://github.com/ZinggJM/GxEPD2) is the reference for the
  1.54" SSD1681 panel: the OTP waveform is loaded with `0xB1`, full refresh is
  `0xF7`, partial is `0xFC`/`0xCF`, and the first refresh must be full. The vendor
  tables in `examples/epaper_weather_station/main/lut_waveshare_154.h` come
  from the same family of reference code.

To adapt another panel: replace `fast_lut` with the vendor's table, set
`mux_height` if it is not 200, and (if the panel uses different OTP display
modes) change the two command sequences in `epaper_panel_refresh_screen()`.

## License

Apache-2.0; example code CC0-1.0.
