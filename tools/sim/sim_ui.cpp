// Host rendering of the front-panel UI: src/display.cpp drives a virtual
// ILI9163C (CASET / RASET / RAMWR decoded into a 128x160 RGB565 frame buffer)
// and every UI state is saved as a PPM screenshot, tools/sim/ui_<name>.ppm.
// tools/sim/ui_sheet.py tiles them into docs/ui-preview.png.
#define DISPLAY_HOST_SIM
#include "Arduino.h"
static uint32_t g_ms = 0;
uint32_t millis() { return g_ms; }
uint32_t micros() { return g_ms * 1000u; }
void delay(uint32_t ms) { g_ms += ms; }
void pinMode(uint32_t, uint32_t) {}
void digitalWrite(uint32_t, uint32_t) {}
int  digitalRead(uint32_t) { return 1; }
uint16_t analogRead(uint32_t) { return 0; }
SimSerial SerialJ12;
size_t SimSerial::write(const uint8_t *, size_t n) { return n; }
TIM_TypeDef tim3_dummy; TIM_TypeDef *TIM3 = &tim3_dummy;

// ---- virtual ILI9163C (mock/panel.h) ----
#include "panel.h"
static void save(const char *name) {
  char path[256]; snprintf(path, sizeof path, "ui_%s.ppm", name);
  panel_save_ppm(path);
  printf("%-17s pixels=%6ld bus_bytes=%6ld out_of_range=%ld\n", name, g_panel_pixels, g_panel_bytes, g_panel_out_of_range);
  g_panel_pixels = g_panel_bytes = 0;
}

#include "../../src/display.cpp"

static DisplayData base() {
  DisplayData d = {};
  d.state = 0; d.target_a = 6; d.volts = 225.3f; d.tempC = 28.8f; d.sel = 3;
  d.mode_auto = true; d.link_ok = true; d.enabled = true;
  d.ct_ready = true;                   // CT calibrated (CT_READY)
  d.clock_min = 14 * 60 + 32;          // "TIME 14:32:00" received from the ESP bridge
  d.on_edf = -1;                       // no "INFO EDF" yet: the plug icon only
  return d;
}
// Off EDF the plug and the sun take turns every 3 s; the bolt blinks every 500 ms.
static const uint32_t T_PLUG = 102500, T_SUN = 105500;    // bolt lit at both

int main() {
  display_init();                      save("boot");

  // READY screen: the greeting comes in (3.1 s), stays 2 s, leaves (3.1 s), rests 5 s, while
  // the plug icon is inserted 1 s / detached 2 s: (ms / 1000) % 3 == 0 -> inserted
  DisplayData d = base();
  g_ms = 3000; display_show(d); display_tick();
  const struct { uint32_t at; const char *name; } R[] = {
    {4500, "ready_in"}, {6500, "ready_greet"}, {7500, "ready_greet_b"}, {9700, "ready_out"},
    {13500, "ready_rest"}, {15000, "ready_rest_b"} };
  for (auto &r : R) { while (g_ms < r.at) { g_ms += 50; display_tick(); } save(r.name); }
  display_show(d); printf("READY re-show, nothing changed: bus_bytes=%ld\n", g_panel_bytes); g_panel_bytes = 0;
  d.clock_min = -1;                    // no TIME yet since boot
  display_show(d);
  while (g_ms < 3000 + 18250 + 4100) { g_ms += 50; display_tick(); }      // the next greeting, held
  save("noclock");

  d = base(); d.state = 1; d.offered_a = 16; d.target_a = 16; d.sel = 1; d.mode_auto = false; d.elapsed_s = 12;
  g_ms = T_PLUG; display_show(d);      save("connected");

  d = base(); d.state = 2; d.contactor = true; d.offered_a = 11; d.target_a = 11; d.amps = 10.8f;
  d.kwh = 3.42f; d.elapsed_s = 3725; d.clock_min = 13 * 60 + 5; d.on_edf = 0;   // off-grid supply (sun icon)
  g_ms = T_PLUG; display_show(d);      save("charging_a");
  g_ms += 500;  display_tick();        save("charging_b");
  display_show(d); printf("re-show, nothing changed: bus_bytes=%ld\n", g_panel_bytes); g_panel_bytes = 0;
  g_ms = T_SUN; display_show(d);       save("charging_sun");
  d.on_edf = 1;                        // the same on EDF: the plug stays
  display_show(d);                     save("charging_edf");

  // before the CT calibration (CT_READY 0): the offer stands in for the current, in grey
  d = base(); d.state = 2; d.contactor = true; d.offered_a = 16; d.target_a = 16; d.sel = 1; d.mode_auto = false;
  d.kwh = 0.12f; d.elapsed_s = 95; d.ct_ready = false;
  g_ms = T_PLUG; display_show(d);      save("charging_man");

  // the same with the calibrated CT
  d = base(); d.state = 2; d.contactor = true; d.offered_a = 16; d.target_a = 16; d.sel = 1; d.mode_auto = false;
  d.amps = 15.8f; d.kwh = 4.75f; d.elapsed_s = 4500; d.clock_min = 18 * 60 + 20;
  display_show(d);                     save("charging_man_ct");

  // forced charge from Home Assistant, at night
  d = base(); d.state = 2; d.contactor = true; d.offered_a = 16; d.target_a = 16; d.amps = 15.9f;
  d.kwh = 8.12f; d.elapsed_s = 7300; d.clock_min = 23 * 60 + 47; d.on_edf = 1;
  display_show(d);                     save("charging_forced");

  // AUTO without a network setpoint for 30 s: the station charges at its 6 A fallback
  d = base(); d.state = 2; d.contactor = true; d.offered_a = 6; d.target_a = 6; d.amps = 5.9f;
  d.kwh = 1.05f; d.elapsed_s = 2900; d.link_ok = false;
  display_show(d);                     save("charging_fallback");

  d = base(); d.state = 1; d.elapsed_s = 600; d.kwh = 0.85f; d.clock_min = 16 * 60 + 41; d.on_edf = 0;
  g_ms = T_PLUG; display_show(d);      save("paused");
  g_ms = T_SUN;  display_show(d);      save("paused_sun");
  d = base(); d.state = 1; d.link_ok = false;                   display_show(d); save("nolink");
  // faults and DISABLED never show the sun, even off EDF (here at the sun's turn)
  d = base(); d.state = 3; d.err = 1; d.target_a = 11; d.on_edf = 0; display_show(d); save("fault_e1");
  d = base(); d.state = 3; d.err = 3; d.tempC = 71.2f; d.on_edf = 0; display_show(d); save("fault_e3");
  d = base(); d.state = 3; d.err = 10; d.on_edf = 0;            display_show(d); save("fault_e10");
  d = base(); d.enabled = false; d.on_edf = 0;                  display_show(d); save("disabled");

  display_init();
  const char *lines[8] = {"DIAG tok1 adc9", "CP+1795 CP-1505", "n112 duty 11.6", "CT PC3 156",
                          "V2930 NTC 880", "offer 6.0A", "chg1 rdy1 btn1", "C1 D1 P1 S1"};
  display_diag(lines, 8);              save("diag");
  return 0;
}
