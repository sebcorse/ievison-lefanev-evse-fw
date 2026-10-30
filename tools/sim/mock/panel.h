// Virtual ILI9163C for the host bench: decodes CASET / RASET / RAMWR and the
// 16-bit pixel stream written by src/display.cpp (DISPLAY_HOST_SIM hooks) into
// a 128x160 RGB565 frame buffer. Used by sim_ui.cpp (screenshots) and
// sim_live.cpp (interactive simulator). Not compiled into the firmware.
#pragma once
#include <stdint.h>
#include <stdio.h>
#include <string>
#include <vector>

static const int PANEL_W = 128, PANEL_H = 160;
static uint16_t g_fb[PANEL_H][PANEL_W];
static bool     g_panel_dirty = false;
static long     g_panel_out_of_range = 0, g_panel_pixels = 0, g_panel_bytes = 0;

static uint8_t  panel_cmd_ = 0;
static std::vector<uint8_t> panel_args_;
static int      panel_xs = 0, panel_xe = PANEL_W - 1, panel_ys = 0, panel_ye = PANEL_H - 1;
static int      panel_cx = 0, panel_cy = 0, panel_phase = 0;
static uint16_t panel_pix = 0;

void sim_lcd_cmd(uint8_t c) {
  panel_cmd_ = c; panel_args_.clear(); panel_phase = 0; g_panel_bytes++;
  if (c == 0x2C) { panel_cx = panel_xs; panel_cy = panel_ys; }
}
void sim_lcd_dat(uint8_t d) {
  g_panel_bytes++;
  if (panel_cmd_ == 0x2A || panel_cmd_ == 0x2B) {
    panel_args_.push_back(d);
    if (panel_args_.size() == 4) {
      int s = (panel_args_[0] << 8) | panel_args_[1], e = (panel_args_[2] << 8) | panel_args_[3];
      if (panel_cmd_ == 0x2A) { panel_xs = s; panel_xe = e; } else { panel_ys = s; panel_ye = e; }
    }
    return;
  }
  if (panel_cmd_ != 0x2C) return;
  if (panel_phase == 0) { panel_pix = (uint16_t)(d << 8); panel_phase = 1; return; }
  panel_pix |= d; panel_phase = 0; g_panel_pixels++;
  if (panel_cx >= 0 && panel_cx < PANEL_W && panel_cy >= 0 && panel_cy < PANEL_H) {
    g_fb[panel_cy][panel_cx] = panel_pix; g_panel_dirty = true;
  } else g_panel_out_of_range++;
  if (++panel_cx > panel_xe) { panel_cx = panel_xs; if (++panel_cy > panel_ye) panel_cy = panel_ys; }
}

static inline void panel_rgb(uint16_t p, uint8_t rgb[3]) {
  rgb[0] = (uint8_t)(((p >> 11) & 31) * 255 / 31);
  rgb[1] = (uint8_t)(((p >> 5) & 63) * 255 / 63);
  rgb[2] = (uint8_t)((p & 31) * 255 / 31);
}
// Frame as a hex string of 128*160*3 bytes (RGB888), row-major.
static void panel_hex(std::string &out) {
  static const char *H = "0123456789abcdef";
  out.clear(); out.reserve(PANEL_W * PANEL_H * 6);
  for (int y = 0; y < PANEL_H; y++) for (int x = 0; x < PANEL_W; x++) {
    uint8_t rgb[3]; panel_rgb(g_fb[y][x], rgb);
    for (int k = 0; k < 3; k++) { out += H[rgb[k] >> 4]; out += H[rgb[k] & 15]; }
  }
}
static void panel_save_ppm(const char *path) {
  FILE *f = fopen(path, "wb");
  if (!f) return;
  fprintf(f, "P6\n%d %d\n255\n", PANEL_W, PANEL_H);
  for (int y = 0; y < PANEL_H; y++) for (int x = 0; x < PANEL_W; x++) {
    uint8_t rgb[3]; panel_rgb(g_fb[y][x], rgb); fwrite(rgb, 1, 3, f);
  }
  fclose(f);
}
