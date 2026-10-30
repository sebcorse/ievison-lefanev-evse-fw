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
  d.soc = -1;                          // no "INFO SOC": the charging battery fills from empty
  return d;
}
// Off EDF the plug and the sun take turns every 3 s (the charging battery: the sun
// after each complete fill).
static const uint32_t T_PLUG = 102500, T_SUN = 105500;
// The charging screen `d` with its battery `ms` into a fill: the charge stops for a
// moment (the animation starts over), then the clock runs.
static void charge_for(const DisplayData &d, uint32_t ms) {
  DisplayData stop = d; stop.contactor = false;
  display_show(stop); display_show(d);
  const uint32_t end = g_ms + ms;
  while (g_ms < end) { g_ms += 50; display_tick(); }
}

// "simui anim": the charging screen every 100 ms for 21.8 s, written as ui_anim_NNN.ppm
// (ui_anim.py makes an animated GIF of them), off-grid ("INFO EDF 0"): the battery
// fills from empty (4.5 s), stays full (0.7 s), gives way to the sun (3 s), and again;
// "INFO SOC 60" comes during the second sun, then the fill starts from 60 % (1.8 s).
// It ends on the sun, so that the GIF loops on a fill starting over.
static int anim() {
  display_init();
  DisplayData d = base(); d.state = 2; d.contactor = true; d.offered_a = 16; d.target_a = 16;
  d.amps = 15.8f; d.kwh = 1.24f; d.elapsed_s = 1800; d.clock_min = 13 * 60 + 5; d.on_edf = 0;
  g_ms = 102000; display_show(d);
  for (int k = 0; k < 218; k++) {
    if (k == 150) d.soc = 60;
    if (k % 10 == 0) { d.elapsed_s++; display_show(d); }       // the 1 s refresh
    g_ms += 50; display_tick(); g_ms += 50; display_tick();
    char path[32]; snprintf(path, sizeof path, "ui_anim_%03d.ppm", k);
    panel_save_ppm(path);
  }
  printf("anim: 218 frames, ui_anim_000.ppm .. ui_anim_217.ppm\n");
  return 0;
}

int main(int argc, char **argv) {
  if (argc > 1 && !strcmp(argv[1], "anim")) return anim();
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
  g_ms = T_PLUG; display_show(d);      // charging starts: the battery fills from empty in 4.5 s
  while (g_ms < T_PLUG + 1500) { g_ms += 50; display_tick(); }   save("charging_a");
  while (g_ms < T_PLUG + 2400) { g_ms += 50; display_tick(); }   save("charging_b");
  display_show(d); printf("re-show, nothing changed: bus_bytes=%ld\n", g_panel_bytes); g_panel_bytes = 0;
  while (g_ms < T_PLUG + 6000) { g_ms += 50; display_tick(); }   save("charging_sun");   // full, then the sun
  d.on_edf = 1; display_show(d);       // back on EDF: the sun gives way, a new fill starts
  while (g_ms < T_PLUG + 8500) { g_ms += 50; display_tick(); }   save("charging_edf");
  d.soc = 60;                          // "INFO SOC 60": the fill goes from 60 %
  display_show(d);                     save("charging_soc");

  // before the CT calibration (CT_READY 0): the offer stands in for the current, in grey
  d = base(); d.state = 2; d.contactor = true; d.offered_a = 16; d.target_a = 16; d.sel = 1; d.mode_auto = false;
  d.kwh = 0.12f; d.elapsed_s = 95; d.ct_ready = false;
  charge_for(d, 1000);                 save("charging_man");

  // the same with the calibrated CT
  d = base(); d.state = 2; d.contactor = true; d.offered_a = 16; d.target_a = 16; d.sel = 1; d.mode_auto = false;
  d.amps = 15.8f; d.kwh = 4.75f; d.elapsed_s = 4500; d.clock_min = 18 * 60 + 20;
  charge_for(d, 3200);                 save("charging_man_ct");

  // forced charge from Home Assistant, at night
  d = base(); d.state = 2; d.contactor = true; d.offered_a = 16; d.target_a = 16; d.amps = 15.9f;
  d.kwh = 8.12f; d.elapsed_s = 7300; d.clock_min = 23 * 60 + 47; d.on_edf = 1;
  charge_for(d, 4000);                 save("charging_forced");

  // AUTO without a network setpoint for 30 s: the station charges at its 6 A fallback
  d = base(); d.state = 2; d.contactor = true; d.offered_a = 6; d.target_a = 6; d.amps = 5.9f;
  d.kwh = 1.05f; d.elapsed_s = 2900; d.link_ok = false;
  charge_for(d, 600);                  save("charging_fallback");

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
