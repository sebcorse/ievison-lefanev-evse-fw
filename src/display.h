// ============================================================================
//  display.h - Front-panel UI on the Topway LMT018DNBFWD-1 (ILI9163C,
//  128x160 portrait, 8-bit parallel over the FSMC). Same layout as the stock
//  screen plus an AUTO preset, fault codes, the time of day, a READY screen with a
//  big clock, and a sun when the energy manager reports an off-grid supply.
// ============================================================================
#pragma once
#include <Arduino.h>

struct DisplayData {
  uint8_t  state;        // 0 = A (no EV), 1 = B (connected), 2 = C (charging), 3 = fault
  uint8_t  err;          // ErrCode (0 = none), shown on the status line
  float    offered_a;    // current offered through the CP duty (0 = no offer)
  float    target_a;     // target current (setpoint / preset after clamping)
  float    amps;         // measured charging current (0 until CT_READY)
  bool     ct_ready;     // false: the big figure shows the offer instead
  float    volts;        // mains voltage
  float    tempC;        // board temperature
  float    kwh;          // session energy
  uint32_t elapsed_s;    // session duration
  uint8_t  sel;          // 0..2 = 10/16/24 A presets, 3 = AUTO
  bool     mode_auto;    // AUTO (setpoint from the ESP bridge) or manual preset
  bool     link_ok;      // AUTO: a setpoint arrived within NET_TIMEOUT_MS
  bool     enabled;      // charging allowed (ENABLE 1)
  bool     contactor;    // contactor closed
  int16_t  clock_min;    // local time, minutes since midnight (TIME from the ESP bridge), -1 = unknown
  int8_t   on_edf;       // house supply ("INFO EDF" from the energy manager): 1 = on EDF (the grid),
                         // 0 = its own solar / battery supply (sun icon), -1 = unknown or stale
};

void display_init();                         // bus + panel init, black screen
void display_show(const DisplayData &d);     // full UI refresh (call at ~1 Hz)
void display_tick();                         // call every loop: icon animation

// Diagnostic build: up to 10 lines of 16 characters, small font.
void display_diag(const char *const *lines, uint8_t n);
// Black screen and forget what is drawn: needed when switching between the UI
// (display_show) and the diag text, both of which only redraw what changed.
void display_clear();
