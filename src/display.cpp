// ============================================================================
//  display.cpp - Front-panel UI on the Topway LMT018DNBFWD-1 (ILI9163C,
//  128x160 portrait) driven in 8-bit 8080 parallel over the STM32 FSMC.
//
//  Bus: FSMC bank 1 / NE1, 8-bit data, D/C on FSMC A18 (PD13, measured) ->
//  command register at 0x60000000, data register at 0x60040000
//  (0x60000000 | 1 << TFT_DC_ADDR_BIT). Reset on PD11 (GPIO). Write-only use.
//  MADCTL (rotation, RGB/BGR), RAM offsets, inversion and FSMC timings are set
//  in board.h.
//
//  UI, same layout as the stock firmware's screen:
//    y   2.. 37  big current: measured, or the offer until CT_READY
//    y  42.. 89  voltage / energy / elapsed (left)  +  socket icon (right)
//    y 100..115  status line, 15 chars (WAITING EV / CHARGING / PAUSED / E<n> <fault>)
//    y 120..135  time of day HH:MM (left, --:-- until the ESP bridge sends TIME),
//                temperature (right)
//    y 142..159  preset row  10  16  24  AUTO, active one green + underlined;
//                AUTO in yellow while it has no network setpoint (fallback current)
//  Icon: no EV  -> plug inserted 1 s / detached 2 s (stock animation), green
//        B      -> inserted, yellow      fault -> detached, red      disabled -> grey
//        charging -> a battery filling up (white outline, green fill), as on the
//                    stock screen, from the vehicle's state of charge when known
//        off-grid ("INFO EDF 0" from the energy manager), B or C, charging
//        allowed -> a yellow sun for 3 s: every other 3 s beside the plug, after
//                    each complete fill beside the battery
//  READY (no EV, charging allowed) has its own screen:
//    y   7.. 23  READY
//    y  32.. 87  time of day in condensed 14x30 digits (--:-- until the first
//                TIME), left; the animated plug icon (8 px higher than usual), right
//    y  94..113  the greeting comes in, stays 2 s, leaves, 5 s of rest
//    y 120..159  temperature and preset row as above (the time is in the row)
//  No framebuffer: every field is a fixed-width cell drawn with its background
//  and redrawn only when its text or colour changes (no flicker).
// ============================================================================
#include "display.h"
#include "board.h"
#include "ui_assets.h"
#include <stdio.h>
#include <string.h>

// ---- RGB565 colours ----
#define C_BLACK  0x0000
#define C_WHITE  0xFFFF
#define C_GREEN  0x07E0
#define C_YELLOW 0xFFE0
#define C_RED    0xF800
#define C_GREY   0x8410
#define C_PRESET 0xA534          // unselected 10 / 16 / 24: 65 % grey (50 % is too dim)
#define DEG      "\xB0"          // degree sign = slot 127 of the small font

#define UI_W 128
#define UI_H 160

// ---------------------------------------------------------------------------
//  Low level: FSMC on the firmware, two hooks on the host simulation bench
// ---------------------------------------------------------------------------
#ifndef DISPLAY_HOST_SIM
#define LCD_REG   (*(volatile uint8_t *)0x60000000u)
#define LCD_DATA  (*(volatile uint8_t *)(0x60000000u | (1u << TFT_DC_ADDR_BIT)))
static inline void lcd_cmd(uint8_t c) { LCD_REG  = c; }
static inline void lcd_dat(uint8_t d) { LCD_DATA = d; }
static void lcd_bus_init() {
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_FSMC_CLK_ENABLE();
  GPIO_InitTypeDef g = {};
  g.Mode  = GPIO_MODE_AF_PP;
  g.Speed = GPIO_SPEED_FREQ_HIGH;
#if TFT_DC_ADDR_BIT == 16
  const uint32_t dc_pin = GPIO_PIN_11;                        // PD11 = A16
#elif TFT_DC_ADDR_BIT == 17
  const uint32_t dc_pin = GPIO_PIN_12;                        // PD12 = A17
#elif TFT_DC_ADDR_BIT == 18
  const uint32_t dc_pin = GPIO_PIN_13;                        // PD13 = A18
#else
#error "TFT_DC_ADDR_BIT: this build maps A16/A17/A18 (PD11/PD12/PD13) only"
#endif
  g.Pin   = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_7 |
            dc_pin | GPIO_PIN_14 | GPIO_PIN_15;               // D2 D3 NOE NWE NE1 D/C D0 D1
  HAL_GPIO_Init(GPIOD, &g);
  g.Pin   = GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10; // D4 D5 D6 D7
  HAL_GPIO_Init(GPIOE, &g);
  // BCR1: bank enabled, SRAM type, 8-bit bus, writes enabled, no wait / burst / mux.
  FSMC_Bank1->BTCR[0] = FSMC_BCRx_MBKEN | FSMC_BCRx_WREN;
  // BTR1 (access mode A): address setup, data phase, bus turnaround, in HCLK cycles.
  FSMC_Bank1->BTCR[1] = ((uint32_t)TFT_FSMC_ADDSET) | ((uint32_t)TFT_FSMC_DATAST << 8) | (1u << 16);
}
#else
void sim_lcd_cmd(uint8_t c);     // provided by tools/sim
void sim_lcd_dat(uint8_t d);
static inline void lcd_cmd(uint8_t c) { sim_lcd_cmd(c); }
static inline void lcd_dat(uint8_t d) { sim_lcd_dat(d); }
static void lcd_bus_init() {}
#endif

// ---------------------------------------------------------------------------
//  Panel init: ILI9163C (datasheet) by default, ST7735S kept as a fallback
// ---------------------------------------------------------------------------
static void panel_cmd(uint8_t c, const uint8_t *d, uint8_t n) { lcd_cmd(c); while (n--) lcd_dat(*d++); }

#if TFT_CONTROLLER_ILI9163C
// Register values are the ones used by the common open-source ILI9163C drivers
// (gamma tables, frame rate, power and VCOM settings); the datasheet gives the
// register map but no recommended sequence.
static void panel_init() {
  static const uint8_t colmod[]  = {0x05};                   // 16 bits / pixel
  static const uint8_t gamset[]  = {0x04};
  static const uint8_t gamrsel[] = {0x01};
  static const uint8_t dfunctr[] = {0xFF, 0x06};
  static const uint8_t pgam[]    = {0x36, 0x29, 0x12, 0x22, 0x1C, 0x15, 0x42, 0xB7,
                                    0x2F, 0x13, 0x12, 0x0A, 0x11, 0x0B, 0x06};
  static const uint8_t ngam[]    = {0x09, 0x16, 0x2D, 0x0D, 0x13, 0x15, 0x40, 0x48,
                                    0x53, 0x0C, 0x1D, 0x25, 0x2E, 0x34, 0x39};
  static const uint8_t frmctr1[] = {0x08, 0x02};
  static const uint8_t dinvctr[] = {0x07};
  static const uint8_t pwctr1[]  = {0x0A, 0x02};
  static const uint8_t pwctr2[]  = {0x02};
  static const uint8_t vcom1[]   = {0x50, 0x5B};
  static const uint8_t vcomoff[] = {0x40};
  static const uint8_t madctl[]  = {TFT_MADCTL};
  lcd_cmd(0x01); delay(120);                                 // SWRESET
  lcd_cmd(0x11); delay(120);                                 // SLPOUT (reset time <= 120 ms)
  panel_cmd(0x3A, colmod, 1);                                // COLMOD
  panel_cmd(0x26, gamset, 1);  panel_cmd(0xF2, gamrsel, 1);  // gamma curve + enable
  lcd_cmd(0x13);                                             // NORON
  panel_cmd(0xB6, dfunctr, 2);                               // display function
  panel_cmd(0xE0, pgam, 15);   panel_cmd(0xE1, ngam, 15);    // gamma tables
  panel_cmd(0xB1, frmctr1, 2);                               // frame rate
  panel_cmd(0xB4, dinvctr, 1);                               // inversion control
  panel_cmd(0xC0, pwctr1, 2);  panel_cmd(0xC1, pwctr2, 1);   // power
  panel_cmd(0xC5, vcom1, 2);   panel_cmd(0xC7, vcomoff, 1);  // VCOM
  lcd_cmd(TFT_INVERT ? 0x21 : 0x20);                         // INVON / INVOFF
  panel_cmd(0x36, madctl, 1);                                // MADCTL
  lcd_cmd(0x29); delay(20);                                  // DISPON
}
#else
static void panel_init() {                                   // ST7735S
  static const uint8_t frmctr[]  = {0x01, 0x2C, 0x2D};
  static const uint8_t frmctr3[] = {0x01, 0x2C, 0x2D, 0x01, 0x2C, 0x2D};
  static const uint8_t invctr[]  = {0x07};
  static const uint8_t pwctr1[]  = {0xA2, 0x02, 0x84};
  static const uint8_t pwctr2[]  = {0xC5};
  static const uint8_t pwctr3[]  = {0x0A, 0x00};
  static const uint8_t pwctr4[]  = {0x8A, 0x2A};
  static const uint8_t pwctr5[]  = {0x8A, 0xEE};
  static const uint8_t vmctr1[]  = {0x0E};
  static const uint8_t madctl[]  = {TFT_MADCTL};
  static const uint8_t colmod[]  = {0x05};                   // 16 bits / pixel
  static const uint8_t gmctrp[]  = {0x02, 0x1C, 0x07, 0x12, 0x37, 0x32, 0x29, 0x2D,
                                    0x29, 0x25, 0x2B, 0x39, 0x00, 0x01, 0x03, 0x10};
  static const uint8_t gmctrn[]  = {0x03, 0x1D, 0x07, 0x06, 0x2E, 0x2C, 0x29, 0x2D,
                                    0x2E, 0x2E, 0x37, 0x3F, 0x00, 0x00, 0x02, 0x10};
  lcd_cmd(0x01); delay(150);                                 // SWRESET
  lcd_cmd(0x11); delay(255);                                 // SLPOUT (>= 120 ms)
  panel_cmd(0xB1, frmctr, 3);  panel_cmd(0xB2, frmctr, 3);  panel_cmd(0xB3, frmctr3, 6);
  panel_cmd(0xB4, invctr, 1);
  panel_cmd(0xC0, pwctr1, 3);  panel_cmd(0xC1, pwctr2, 1);  panel_cmd(0xC2, pwctr3, 2);
  panel_cmd(0xC3, pwctr4, 2);  panel_cmd(0xC4, pwctr5, 2);  panel_cmd(0xC5, vmctr1, 1);
  lcd_cmd(TFT_INVERT ? 0x21 : 0x20);                         // INVON / INVOFF
  panel_cmd(0x36, madctl, 1);                                // MADCTL
  panel_cmd(0x3A, colmod, 1);                                // COLMOD
  panel_cmd(0xE0, gmctrp, 16); panel_cmd(0xE1, gmctrn, 16);  // gamma
  lcd_cmd(0x13); delay(10);                                  // NORON
  lcd_cmd(0x29); delay(100);                                 // DISPON
}
#endif

// ---------------------------------------------------------------------------
//  Drawing primitives
// ---------------------------------------------------------------------------
static void lcd_window(int x0, int y0, int x1, int y1) {
  x0 += TFT_COL_OFFSET; x1 += TFT_COL_OFFSET; y0 += TFT_ROW_OFFSET; y1 += TFT_ROW_OFFSET;
  lcd_cmd(0x2A); lcd_dat(0); lcd_dat((uint8_t)x0); lcd_dat(0); lcd_dat((uint8_t)x1);  // CASET
  lcd_cmd(0x2B); lcd_dat(0); lcd_dat((uint8_t)y0); lcd_dat(0); lcd_dat((uint8_t)y1);  // RASET
  lcd_cmd(0x2C);                                                                   // RAMWR
}
static inline void px(uint16_t c) { lcd_dat((uint8_t)(c >> 8)); lcd_dat((uint8_t)c); }

static void fill_rect(int x, int y, int w, int h, uint16_t c) {
  if (w <= 0 || h <= 0) return;
  lcd_window(x, y, x + w - 1, y + h - 1);
  for (uint32_t n = (uint32_t)w * h; n; n--) px(c);
}
// 1 bpp bitmap, MSB first, rows padded to a byte; every pixel of the box is written.
static void blit_1bpp(int x, int y, int w, int h, const uint8_t *bits, uint16_t fg, uint16_t bg) {
  int rb = (w + 7) / 8;
  lcd_window(x, y, x + w - 1, y + h - 1);
  for (int r = 0; r < h; r++) {
    const uint8_t *row = bits + r * rb;
    for (int c = 0; c < w; c++) px((row[c >> 3] & (0x80 >> (c & 7))) ? fg : bg);
  }
}
#define SMALL_RB ((FONT_SMALL_W + 7) / 8)
#define BIG_RB   ((FONT_BIG_W + 7) / 8)

static void text_small(int x, int y, const char *s, uint16_t fg, uint16_t bg) {
  for (; *s; s++, x += FONT_SMALL_W) {
    uint8_t ch = (uint8_t)*s; int idx;
    if (ch == 0xB0)                               idx = FONT_SMALL_COUNT - 1;   // degree
    else if (ch < FONT_SMALL_FIRST || ch > 126)   idx = '?' - FONT_SMALL_FIRST;
    else                                          idx = ch - FONT_SMALL_FIRST;
    blit_1bpp(x, y, FONT_SMALL_W, FONT_SMALL_H, FONT_SMALL + idx * FONT_SMALL_H * SMALL_RB, fg, bg);
  }
}
static void text_big(int x, int y, const char *s, uint16_t fg, uint16_t bg) {
  static const char set[] = FONT_BIG_CHARS;
  for (; *s; s++, x += FONT_BIG_W) {
    const char *p = (*s == ' ') ? nullptr : strchr(set, *s);
    if (!p) { fill_rect(x, y, FONT_BIG_W, FONT_BIG_H, bg); continue; }
    blit_1bpp(x, y, FONT_BIG_W, FONT_BIG_H, FONT_BIG + (p - set) * FONT_BIG_H * BIG_RB, fg, bg);
  }
}

// Cached text cell: redrawn only when the text or the colour changes.
struct Field { char txt[20]; uint16_t fg; bool valid; };
static void put_small(Field &f, int x, int y, const char *txt, uint16_t fg) {
  if (f.valid && f.fg == fg && !strcmp(f.txt, txt)) return;
  strncpy(f.txt, txt, sizeof f.txt - 1); f.txt[sizeof f.txt - 1] = 0; f.fg = fg; f.valid = true;
  text_small(x, y, txt, fg, C_BLACK);
}
static void put_big(Field &f, int x, int y, const char *txt, uint16_t fg) {
  if (f.valid && f.fg == fg && !strcmp(f.txt, txt)) return;
  strncpy(f.txt, txt, sizeof f.txt - 1); f.txt[sizeof f.txt - 1] = 0; f.fg = fg; f.valid = true;
  text_big(x, y, txt, fg, C_BLACK);
}

// ---------------------------------------------------------------------------
//  Layout
// ---------------------------------------------------------------------------
#define X_LEFT     4
#define Y_BIG      2
#define Y_VOLT    42
#define Y_KWH     58
#define Y_TIME    74
#define X_ICON    84
#define Y_ICON    40
#define Y_STATUS 100
#define Y_TEMP   120      // bottom info line: clock (left) + temperature (right)
#define X_TEMP   (UI_W - X_LEFT - 7 * FONT_SMALL_W)   // "%5.1f°C" right-aligned (68..123)
#define Y_PRESET 142
#define Y_ULINE  158

#define Y_ICON_R  32      // READY screen: the plug icon, 8 px higher than on the others
#define X_CLOCK    4      //   "HH:MM" in 14x30 digits 3 px apart, narrow ':' (75 px, left of the icon)
#define Y_CLOCK   45      //   centred on the icon (32 + (56 - 30) / 2)
#define CLOCK_GAP  3      //   between two characters
#define Y_READY    3      //   "READY" (ink 7..23)
#define Y_GREET   94      //   greeting line (GREET_H high), the whole width
#define C_PINK    0xFB56
#define SUN_PERIOD_MS 3000U   // off EDF: the sun's turn (after a plug period or a battery fill)

static Field f_big, f_volt, f_kwh, f_time, f_status, f_temp, f_clock, f_bigclock;

// ---- Time of day in condensed 14x30 digits, CLOCK_GAP apart (READY screen) ------
#define CLOCK_RB ((FONT_CLOCK_W + 7) / 8)
// ':' is drawn narrow: the centre half of its cell.
static void text_clock(int x, int y, const char *s, uint16_t fg, uint16_t bg) {
  static const char set[] = FONT_CLOCK_CHARS;
  for (; *s; s++) {
    const char *p = strchr(set, *s);
    int w = (*s == ':') ? FONT_CLOCK_W / 2 : FONT_CLOCK_W, c0 = (FONT_CLOCK_W - w) / 2;
    if (!p) { fill_rect(x, y, w, FONT_CLOCK_H, bg); x += w; continue; }
    const uint8_t *row = FONT_CLOCK + (p - set) * FONT_CLOCK_H * CLOCK_RB;
    lcd_window(x, y, x + w - 1, y + FONT_CLOCK_H - 1);
    for (int r = 0; r < FONT_CLOCK_H; r++, row += CLOCK_RB)
      for (int c = c0; c < c0 + w; c++) px((row[c >> 3] & (0x80 >> (c & 7))) ? fg : bg);
    x += w;
    if (s[1]) { fill_rect(x, y, CLOCK_GAP, FONT_CLOCK_H, bg); x += CLOCK_GAP; }
  }
}
static void put_clock(Field &f, int x, int y, const char *txt, uint16_t fg) {
  if (f.valid && f.fg == fg && !strcmp(f.txt, txt)) return;
  strncpy(f.txt, txt, sizeof f.txt - 1); f.txt[sizeof f.txt - 1] = 0; f.fg = fg; f.valid = true;
  text_clock(x, y, txt, fg, C_BLACK);
}

// ---- READY screen greeting -------------------------------------------------------
// The greeting (TXT_GREET: ui_assets.h, or your own src/greeting_local.h; at most
// the line's width) comes in from the right, stays GREET_HOLD_MS whole and centred,
// leaves to the left, then the line rests GREET_REST_MS and it comes again. Smooth,
// one pixel at a time, drawn from display_tick().
#define GREET_PX_S    40U
#define GREET_HOLD_MS 2000U
#define GREET_REST_MS 5000U      // rest between two passes
#define X_GREET       ((UI_W - GREET_W) / 2)
static uint32_t g_greet_t0 = 0;
static int      g_greet_x = 0x7FFF;          // text left edge last drawn, 0x7FFF = draw again

// The whole line with the text's left edge at xt (off the panel = blank line).
static void greet_draw(int xt) {
  const int rb = (GREET_W + 7) / 8;
  lcd_window(0, Y_GREET, UI_W - 1, Y_GREET + GREET_H - 1);
  for (int r = 0; r < GREET_H; r++)
    for (int c = 0; c < UI_W; c++) {
      int g = c - xt;
      px((g >= 0 && g < GREET_W && (TXT_GREET[r * rb + (g >> 3)] & (0x80 >> (g & 7)))) ? C_PINK : C_BLACK);
    }
}
static void greet_tick(uint32_t now) {
  const uint32_t t_in  = (UI_W - X_GREET) * 1000U / GREET_PX_S;          // right edge -> centred
  const uint32_t t_out = (X_GREET + GREET_W) * 1000U / GREET_PX_S;       // centred -> past the left edge
  uint32_t t = (now - g_greet_t0) % (t_in + GREET_HOLD_MS + t_out + GREET_REST_MS);
  int xt;
  if (t < t_in)                               xt = UI_W - (int)(t * GREET_PX_S / 1000U);
  else if ((t -= t_in) < GREET_HOLD_MS)       xt = X_GREET;
  else if ((t -= GREET_HOLD_MS) < t_out)      xt = X_GREET - (int)(t * GREET_PX_S / 1000U);
  else                                        xt = UI_W;                // resting, blank line
  if (xt != g_greet_x) { greet_draw(xt); g_greet_x = xt; }
}
static Field f_diag[10];                       // display_diag() lines
static DisplayData g_d;
static bool g_have_data = false;
static uint8_t  cur_frame = 0xFF; static uint16_t cur_color = 0;
static uint8_t  cur_sel = 0xFF; static bool cur_auto_warn = false;
static int8_t   cur_ready = -1;                // screen drawn: 1 READY, 0 the others, -1 none yet

static void invalidate() {
  f_big.valid = f_volt.valid = f_kwh.valid = f_time.valid = f_status.valid = f_temp.valid = f_clock.valid = false;
  f_bigclock.valid = false;
  for (uint8_t k = 0; k < 10; k++) f_diag[k].valid = false;
  cur_frame = 0xFF; cur_sel = 0xFF; cur_ready = -1;
}

// READY and the other states share only the bottom line and the presets:
// switching wipes everything above them and draws the fixed parts.
static void screen_select(bool ready) {
  if ((int8_t)ready == cur_ready) return;
  cur_ready = (int8_t)ready;
  fill_rect(0, 0, UI_W, Y_TEMP, C_BLACK);
  f_big.valid = f_volt.valid = f_kwh.valid = f_time.valid = f_status.valid = f_bigclock.valid = false;
  cur_frame = 0xFF;
  if (ready) {
    blit_1bpp((UI_W - READY_W) / 2, Y_READY, READY_W, READY_H, TXT_READY, C_WHITE, C_BLACK);
    g_greet_t0 = millis(); g_greet_x = 0x7FFF;
  }
}

static const char *fault_name(uint8_t e) {
  switch (e) {
    case 1: return "CP -12V";   case 2: return "STATE D/E"; case 3: return "OVERTEMP";
    case 4: return "TIMER";     case 5: return "OVERCURR";  case 6: return "WELDED";
    case 7: return "ADC";       case 8: return "NTC";       case 9: return "CP CHECK";
    case 10: return "C BEFORE B";
    default: return "FAULT";
  }
}

// ---- Charging battery (the stock firmware's animation, redrawn) -------------------
// While the vehicle charges, a battery fills up in the icon cell: white outline, green
// fill whose surface curves down to the right like a liquid, from the vehicle's state
// of charge when the energy manager sends it ("INFO SOC"), else from empty. Full in
// BATT_FILL_MS from empty, BATT_HOLD_MS full, then again from the start - off-grid
// with the sun's turn (SUN_PERIOD_MS) after each complete fill. Only the cells that
// change are redrawn, one column at a time (no flicker).
#define BATT_FILL_MS  4500U
#define BATT_HOLD_MS   700U
#define BATT_X  (X_ICON + 9)          // fill area, inside the outline with 1 px of black around
#define BATT_Y  (Y_ICON + 9)
#define BATT_W  22
#define BATT_H  44
static const int8_t BATT_WAVE[BATT_W] = {   // the surface: 3 px higher on the left, 3 lower on the right
  3, 3, 3, 3, 2, 2, 2, 2, 1, 1, 0, 0, -1, -1, -2, -2, -2, -2, -3, -3, -3, -3 };
static uint8_t  g_batt_col[BATT_W];   // filled height of each column, as drawn
static bool     g_batt_run = false;   // charging: the animation is running
static bool     g_batt_sun = false;   // off-grid: the sun's turn, between two fills
static uint32_t g_batt_t0 = 0;        // start of the current fill (or of the sun's turn)

static void batt_frame() {            // outline + empty interior; batt_draw() fills it
  fill_rect(X_ICON, Y_ICON, ICON_W, ICON_H, C_BLACK);
  fill_rect(X_ICON + 14, Y_ICON + 2, 12, 4, C_WHITE);       // terminal
  fill_rect(X_ICON + 6, Y_ICON + 6, 28, 2, C_WHITE);        // top
  fill_rect(X_ICON + 6, Y_ICON + 54, 28, 2, C_WHITE);       // bottom
  fill_rect(X_ICON + 6, Y_ICON + 8, 2, 46, C_WHITE);        // left
  fill_rect(X_ICON + 32, Y_ICON + 8, 2, 46, C_WHITE);       // right
  memset(g_batt_col, 0, sizeof g_batt_col);
}
// Off-grid (the house on its own solar / battery supply, "INFO EDF 0"), the sun shows
// while a vehicle is connected and charging is allowed - never on a fault.
static bool sun_shows() { return g_d.on_edf == 0 && g_d.enabled && (g_d.state == 1 || g_d.state == 2); }
// The fill starts from the state of charge (or 0) and rises to 100 at a steady pace,
// over 1 s at least (a nearly full battery would flicker).
static uint32_t batt_from() { return (g_d.soc >= 0 && g_d.soc <= 100) ? (uint32_t)g_d.soc : 0u; }
static uint32_t batt_fill_ms() {
  const uint32_t fill = BATT_FILL_MS * (100u - batt_from()) / 100u;
  return fill < 1000u ? 1000u : fill;
}
// The sequence, at every tick while charging: fill, hold full, then off-grid the sun's
// turn, then a new fill from the start. Back on EDF the sun gives way at once.
static void batt_step(uint32_t now) {
  if (!g_batt_run) { g_batt_run = true; g_batt_sun = false; g_batt_t0 = now; return; }
  const uint32_t t = now - g_batt_t0;
  if (g_batt_sun) {
    if (t >= SUN_PERIOD_MS || !sun_shows()) { g_batt_sun = false; g_batt_t0 = now; }
  } else if (t >= batt_fill_ms() + BATT_HOLD_MS) {
    g_batt_sun = sun_shows(); g_batt_t0 = now;
  }
}
static uint8_t batt_level(uint32_t now) {    // fill level in % at `now`
  const uint32_t t = now - g_batt_t0, fill = batt_fill_ms(), from = batt_from();
  return (uint8_t)(t < fill ? from + (100u - from) * t / fill : 100u);
}
static void batt_draw(uint8_t level) {
  const int h = (int)level * (BATT_H + 6) / 100 - 3;     // the curve spans 6 px: empty at 0, flat at 100
  for (int c = 0; c < BATT_W; c++) {
    int hc = h + BATT_WAVE[c];
    if (hc < 0) hc = 0; else if (hc > BATT_H) hc = BATT_H;
    const int old = g_batt_col[c];
    if (hc > old)      fill_rect(BATT_X + c, BATT_Y + BATT_H - hc, 1, hc - old, C_GREEN);
    else if (hc < old) fill_rect(BATT_X + c, BATT_Y + BATT_H - old, 1, old - hc, C_BLACK);
    g_batt_col[c] = (uint8_t)hc;
  }
}

static void draw_icon(uint8_t frame, uint16_t color) {       // 0 plugged, 1 unplugged, 2 sun, 3 battery
  if (frame == 3) { batt_frame(); return; }
  blit_1bpp(X_ICON, cur_ready == 1 ? Y_ICON_R : Y_ICON, ICON_W, ICON_H,
            frame == 2 ? ICON_SUN : frame ? ICON_UNPLUGGED : ICON_PLUGGED, color, C_BLACK);
}
// Icon from the current data and the clock (animation). Off-grid (sun_shows), the
// plug and the sun take turns every SUN_PERIOD_MS; while charging, the sun comes
// after each complete fill of the battery (batt_step).
static void icon_update(uint32_t now) {
  uint8_t frame; uint16_t col;
  const bool charging = (g_d.state == 2 && g_d.contactor);
  if (charging) batt_step(now); else g_batt_run = false;
  switch (g_d.state) {
    case 0:  frame = ((now / 1000) % 3) ? 1 : 0;                 // inserted 1 s, detached 2 s
             col = g_d.enabled ? C_GREEN : C_GREY; break;
    case 1:  frame = 0; col = g_d.enabled ? C_YELLOW : C_GREY; break;
    case 2:  frame = charging ? 3 : 0; col = C_GREEN; break;      // the battery fills while charging
    default: frame = 1; col = C_RED; break;
  }
  if (charging ? g_batt_sun : (sun_shows() && (now / SUN_PERIOD_MS) % 2)) { frame = 2; col = C_YELLOW; }
  if (frame != cur_frame || col != cur_color) { draw_icon(frame, col); cur_frame = frame; cur_color = col; }
  if (frame == 3) batt_draw(batt_level(now));
}

// auto_warn: AUTO is selected but has no network setpoint (the station runs on
// its fallback current). The selected AUTO is then yellow instead of green.
static void draw_presets(uint8_t sel, bool auto_warn) {
  if (sel == cur_sel && auto_warn == cur_auto_warn) return;
  cur_sel = sel; cur_auto_warn = auto_warn;
  static const struct { const char *txt; uint8_t x; } P[4] = {{"10", 4}, {"16", 33}, {"24", 62}, {"AUTO", 91}};
  for (uint8_t k = 0; k < 4; k++) {
    bool on = (k == sel);
    uint16_t col = !on ? C_PRESET : (k == 3 && auto_warn) ? C_YELLOW : C_GREEN;
    text_small(P[k].x, Y_PRESET, P[k].txt, col, C_BLACK);
    fill_rect(P[k].x, Y_ULINE, (int)strlen(P[k].txt) * FONT_SMALL_W, 2, on ? col : C_BLACK);
  }
}

// ---------------------------------------------------------------------------
//  Public API
// ---------------------------------------------------------------------------
void display_init() {
  // /RES low while the bus comes up (datasheet: pulse >= 1 us), then released;
  // the panel is ready at most 120 ms later. Backlight is hard-wired on.
  pinMode(PIN_TFT_RST, OUTPUT);
  digitalWrite(PIN_TFT_RST, LOW);
#if TFT_HAS_BL
  pinMode(PIN_TFT_BL, OUTPUT); digitalWrite(PIN_TFT_BL, TFT_BL_ACTIVE);
#endif
  lcd_bus_init();
  delay(10);
  digitalWrite(PIN_TFT_RST, HIGH);
  delay(120);
  panel_init();
  fill_rect(0, 0, UI_W, UI_H, C_BLACK);
  invalidate();
  g_have_data = false;
  text_small(X_LEFT, Y_STATUS, "evse-fw", C_GREY, C_BLACK);
}

void display_show(const DisplayData &d) {
  char b[24];
  g_d = d; g_have_data = true;
  const bool ready = (d.state == 0 && d.enabled);
  screen_select(ready);

  if (ready) {
    if (d.clock_min >= 0 && d.clock_min < 24 * 60) snprintf(b, sizeof b, "%02d:%02d", d.clock_min / 60, d.clock_min % 60);
    else                                           snprintf(b, sizeof b, "--:--");
    put_clock(f_bigclock, X_CLOCK, Y_CLOCK, b, d.clock_min >= 0 ? C_WHITE : C_GREY);
    icon_update(millis());
  } else {
    // big figure: measured current once the CT is calibrated, the offer before
    float big = d.ct_ready ? d.amps : d.offered_a;
    snprintf(b, sizeof b, "%4.1fA", (double)big);
    put_big(f_big, X_LEFT, Y_BIG, b, d.ct_ready ? C_WHITE : C_GREY);

    snprintf(b, sizeof b, "%5.1fV", (double)d.volts);          put_small(f_volt, X_LEFT, Y_VOLT, b, C_WHITE);
    snprintf(b, sizeof b, "%5.2fkWh", (double)d.kwh);          put_small(f_kwh,  X_LEFT, Y_KWH,  b, C_WHITE);
    uint32_t s = d.elapsed_s;
    snprintf(b, sizeof b, "%02lu:%02lu:%02lu", (unsigned long)(s / 3600), (unsigned long)(s / 60 % 60),
             (unsigned long)(s % 60));                           put_small(f_time, X_LEFT, Y_TIME, b, C_WHITE);

    // status line, 15 characters from x=4 (4..123), padded to erase the previous text
    const char *st; uint16_t col; char fb[20];
    if (d.state == 3)          { snprintf(fb, sizeof fb, "E%u %s", d.err, fault_name(d.err)); st = fb; col = C_RED; }
    else if (!d.enabled)       { st = "DISABLED"; col = C_GREY; }
    else if (d.state == 2)     { st = d.contactor ? "CHARGING" : "STOPPING"; col = C_GREEN; }
    else                       { st = d.offered_a > 0 ? "WAITING EV" : (d.mode_auto && !d.link_ok ? "NO LINK" : "PAUSED"); col = C_YELLOW; }
    snprintf(b, sizeof b, "%-15.15s", st);                      put_small(f_status, X_LEFT, Y_STATUS, b, col);
    icon_update(millis());
  }

  // bottom line: the time of day on the left - blank on the READY screen, where it
  // stands larger beside the plug - and the temperature on the right
  if (ready)                                       put_small(f_clock, X_LEFT, Y_TEMP, "     ", C_WHITE);
  else if (d.clock_min >= 0 && d.clock_min < 24 * 60) {
    snprintf(b, sizeof b, "%02d:%02d", d.clock_min / 60, d.clock_min % 60);
    put_small(f_clock, X_LEFT, Y_TEMP, b, C_WHITE);
  } else                                           put_small(f_clock, X_LEFT, Y_TEMP, "--:--", C_GREY);
  snprintf(b, sizeof b, "%5.1f" DEG "C", (double)d.tempC);   put_small(f_temp, X_TEMP, Y_TEMP, b, C_WHITE);

  draw_presets(d.sel, d.mode_auto && !d.link_ok);
}

void display_tick() {
  static uint32_t last = 0;
  uint32_t now = millis();
  if (now - last < 50) return;
  last = now;
  if (!g_have_data) return;
  icon_update(now);
  if (cur_ready == 1) greet_tick(now);
}

void display_diag(const char *const *lines, uint8_t n) {
  if (n > 10) n = 10;
  for (uint8_t k = 0; k < n; k++) {
    char b[17];
    snprintf(b, sizeof b, "%-16.16s", lines[k]);
    put_small(f_diag[k], 0, k * FONT_SMALL_H, b, C_WHITE);
  }
}

void display_clear() {
  fill_rect(0, 0, UI_W, UI_H, C_BLACK);
  invalidate();
  g_have_data = false;
}
