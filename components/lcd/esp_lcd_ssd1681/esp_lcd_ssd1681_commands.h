/*
 * SPDX-FileCopyrightText: 2023-2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

#include <stdint.h>

/* SSD1681 command set (datasheet rev 0.13).

 * X addresses are expressed in bytes of 8 pixels, Y addresses in rows. The
 * SSD1681 has two 200x200 bit planes: BW RAM (0x24) and RED RAM (0x26).
 * Their bit values map to LUTs as follows:
 *   (R,BW) = (0,0) -> LUT0 (black), (0,1) -> LUT1 (white)
 *   (1,0) -> LUT2 (red),          (1,1) -> LUT3 (= LUT2)
 */

#define SSD1681_CMD_SWRST                   0x12  /* Software reset */
#define SSD1681_CMD_SLEEP                   0x10  /* Deep sleep */
#define SSD1681_PARAM_SLEEP_MODE_1          0x01  /* Keep RAM; wake via hardware reset */

#define SSD1681_CMD_OUTPUT_CTRL             0x01  /* Driver output control: MUX, scan direction */
#define SSD1681_CMD_DATA_ENTRY_MODE         0x11  /* Address counter increment mode */
#define SSD1681_PARAM_DATA_ENTRY_MODE_Y_X   0x03  /* Y increment, X increment */
#define SSD1681_CMD_SET_RAMX_START_END      0x44  /* RAM X start/end, byte units */
#define SSD1681_CMD_SET_RAMY_START_END      0x45  /* RAM Y start/end, row units */
#define SSD1681_CMD_SET_INIT_X_ADDR         0x4E  /* RAM X address counter */
#define SSD1681_CMD_SET_INIT_Y_ADDR         0x4F  /* RAM Y address counter */

#define SSD1681_CMD_WRITE_BW_RAM            0x24
#define SSD1681_CMD_WRITE_RED_RAM           0x26
#define SSD1681_CMD_WRITE_PREV_RAM          0x26  /* "Previous image" plane in Display Mode 2 */
#define SSD1681_CMD_READ_RAM                0x27
#define SSD1681_CMD_READ_RAM_OPTION         0x41

#define SSD1681_CMD_SET_BORDER_WAVEFORM     0x3C  /* VBD border driving */
#define SSD1681_PARAM_BORDER_WAVEFORM       0x01
#define SSD1681_CMD_SET_TEMP_SENSOR         0x18  /* 0x80: internal sensor */

/* Command 0x21: RAM content option for the display update.
 * A[7:4] and A[3:0]: 0000 normal, 0100 bypass as 0, 1000 inverse. */
#define SSD1681_CMD_DISP_UPDATE_CTRL1       0x21
#define SSD1681_PARAM_INVERSE_BW            (1 << 3)
#define SSD1681_PARAM_INVERSE_RED           (1 << 7)

/* Command 0x22: display update sequence option.
 * 0xb1 = enable clock + load temperature + load OTP LUT mode 1 + disable clock
 * 0xf7 = clock + analog + temperature + OTP LUT + display mode 1 + power down
 * 0xcf = clock + analog + display mode 2 (previous RAM as reference) + power down
 */
#define SSD1681_CMD_DISP_UPDATE_CTRL2       0x22
#define SSD1681_PARAM_LOAD_OTP_LUT          0xb1
#define SSD1681_PARAM_DISPLAY_MODE_1        0xf7  /* full refresh: load OTP waveform and display */
#define SSD1681_PARAM_DISPLAY_MODE_2_PARTIAL 0xcf  /* fast/partial: display mode 2, current LUT */
#define SSD1681_PARAM_DISPLAY_MODE_1_TEMP   0xf7
#define SSD1681_CMD_MASTER_ACTIVATE         0x20

/* Waveform Setting (159 bytes).
 * 0..59:   VS[nX-LUTm], two 2-bit voltage codes per byte, LUT0..LUT4
 * 60..143: TP[nA/B/C/D], SR[nAB/CD], RP[n] for groups 0..11
 * 144..149: FR[0..11], frame rate (2 groups per byte)
 * 150..152: XON[0AB..11CD], all-gate-on selection
 * 153: EOPT (0x22 normal, 0x07 keep source output before power off)
 * 154: VGH, 155: VSH1, 156: VSH2, 157: VSL, 158: VCOM
 */
#define SSD1681_CMD_SET_LUT_REG             0x32
#define SSD1681_CMD_SET_END_OPTION          0x3F
#define SSD1681_CMD_SET_GATE_VOLTAGE        0x03
#define SSD1681_CMD_SET_SRC_VOLTAGE         0x04
#define SSD1681_CMD_SET_VCOM                0x2C

#define SSD1681_LUT_VS_BYTES                60
#define SSD1681_LUT_TIMING_BYTES            84
#define SSD1681_LUT_FRAME_RATE_BYTES        6
#define SSD1681_LUT_XON_BYTES               3
#define SSD1681_LUT_ANALOG_BYTES            6
#define SSD1681_LUT_SIZE                    159
