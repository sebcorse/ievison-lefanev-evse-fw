// Host-side simulation harness for evse-fw/src/main.cpp (read-only use of the sources).
// The firmware sources are #included unchanged; only Arduino/HAL are mocked.
#define DISPLAY_HOST_SIM          // display.cpp talks to the two hooks below, not the FSMC
#include "Arduino.h"
#include "IWatchdog.h"
#include <vector>
#include <random>
#include <algorithm>

// ---------------- simulation clock & config ----------------
static uint64_t g_now_us = 0;
static double   ADC_US   = 10.0;     // duration of one ADC read (measured on the station: adc=10)
static double   TAU_US   = 0.0;      // CP front-end RC time constant (0 = ideal)
static double   NOISE_P  = 0.0;      // probability of an isolated outlier per CP sample
static int      NOISE_MAG = 0;       // outlier magnitude (ADC counts, signed)
static bool     VERBOSE  = false;
static std::mt19937 rng(12345);

// ---------------- EV model ----------------
static bool   ev_plugged = false, ev_diode = true, ev_wants = true, ev_S2 = false;
static double ev_leak_ohm = 0;              // > 0: no EV, a plain resistor CP-PE (water in the plug...)
static bool   ev_auto = true;               // reacts to the PWM like a real EV
static double EV_CLOSE_MS = 300;            // PWM seen this long -> close S2
static double EV_OPEN_MS  = 1000;           // PWM lost this long -> open S2
static double EV_RAMP_MS  = 1000;           // current ramp-down after PWM loss / duty drop
static uint64_t ev_pwm_since = 0, ev_nopwm_since = 0; static bool ev_pwm_prev = false;
static double ev_I = 0, ev_I_at_loss = 0; static uint64_t ev_t_loss = 0;
static int sim_ntc = 1008;              // NTC "heat" value: ~31.7 degC (measured on the station)
static double g_np = 0.01; static int g_nm = -400;

// ---------------- recorders ----------------
static bool pin_contactor = false; static uint64_t t_contactor_change = 0;
static int n_close = 0, n_open = 0, n_break_under_load = 0; static double max_break_I = 0;
static std::vector<std::string> events;
static void ev_log(const char *fmt, ...) {
  char b[256]; va_list ap; va_start(ap, fmt); vsnprintf(b, sizeof b, fmt, ap); va_end(ap);
  char t[300]; snprintf(t, sizeof t, "[%9.3f s] %s", g_now_us / 1e6, b); events.push_back(t);
  if (VERBOSE) puts(t);
}

SimSerial SerialJ12;
TIM_TypeDef tim3_dummy; TIM_TypeDef *TIM3 = &tim3_dummy;
IWatchdogClass IWatchdog;
void IWatchdogClass::begin(uint32_t) { started = 1; last_reload_us = g_now_us; }
void IWatchdogClass::reload() {
  if (started) { uint64_t gap = g_now_us - last_reload_us; if (gap > max_gap_us) max_gap_us = gap; }
  last_reload_us = g_now_us;
}

static uint64_t tx_end_us = 0;
static std::vector<std::string> status_lines;
size_t SimSerial::write(const uint8_t *buf, size_t n) {
  // 115200 8N1 = 86.8 us/byte, 64-byte TX ring: block while more than 64 bytes are queued
  if (tx_end_us < g_now_us) tx_end_us = g_now_us;
  tx_end_us += (uint64_t)(n * 86.8);
  if (tx_end_us - g_now_us > (uint64_t)(64 * 86.8)) g_now_us = tx_end_us - (uint64_t)(64 * 86.8);
  for (size_t i = 0; i < n; i++) {
    char c = (char)buf[i];
    if (c == '\n') { status_lines.push_back(line); if (VERBOSE) ::printf("    UART> %s\n", line.c_str()); line.clear(); }
    else line += c;
  }
  return n;
}
static void send_cmd(const char *s) { for (const char *p = s; *p; p++) SerialJ12.rx.push_back(*p); SerialJ12.rx.push_back('\n'); ev_log("ESP -> \"%s\"", s); }

uint32_t millis() { return (uint32_t)(g_now_us / 1000); }
uint32_t micros() { return (uint32_t)g_now_us; }
void delay(uint32_t ms) { g_now_us += (uint64_t)ms * 1000; }
void pinMode(uint32_t, uint32_t) {}
int  digitalRead(uint32_t pin) { return pin == PE14 ? 1 : 0; }

// ---------------- firmware under test ----------------
#include "../../src/main.cpp"
#include "../../src/display.cpp"
void sim_lcd_cmd(uint8_t) {}       // the panel is not modelled here (see sim_ui.cpp)
void sim_lcd_dat(uint8_t) {}

// ---------------- models that need the firmware's statics ----------------
static bool pwm_present() { return cp_compare_ticks > 0 && cp_compare_ticks < CP_ARR; }
static double ev_seen_duty() { double d = cp_compare_ticks / 10.0; return getenv("INVERT") ? 100.0 - d : d; }
static double offered_A() { return pwm_present() ? DUTY_TO_AMP(cp_compare_ticks / 10.0) : 0.0; }

void digitalWrite(uint32_t pin, uint32_t val) {
  if (pin != PB1) return;
  bool on = (val == CONTACTOR_ON);
  if (on != pin_contactor) {
    if (on) { n_close++; double d = ev_seen_duty(); double a = d <= 85 ? (d >= 10 ? d * 0.6 : 0) : (d - 64) * 2.5;
      ev_log("CONTACTOR CLOSE (firmware offer %.1f A, duty seen by the EV %.1f %% = %.0f A, I_ev %.1f A)", offered_A(), d, a, ev_I); }
    else {
      n_open++;
      ev_log("CONTACTOR OPEN  (I_ev %.1f A, closed for %.1f s)", ev_I, (g_now_us - t_contactor_change) / 1e6);
      if (ev_I > 1.0) { n_break_under_load++; if (ev_I > max_break_I) max_break_I = ev_I; }
    }
    pin_contactor = on; t_contactor_change = g_now_us;
  }
}

static void ev_update() {
  bool pwm = pwm_present();
  if (pwm != ev_pwm_prev) {
    if (pwm) ev_pwm_since = g_now_us; else { ev_nopwm_since = g_now_us; ev_I_at_loss = ev_I; ev_t_loss = g_now_us; }
    ev_log("CP %s (duty ticks %u)", pwm ? "PWM ON" : "PWM OFF (standby)", (unsigned)cp_compare_ticks);
    ev_pwm_prev = pwm;
  }
  if (!ev_plugged) { ev_S2 = false; ev_I = 0; return; }
  if (ev_auto) {
    if (!ev_S2 && ev_wants && pwm && (g_now_us - ev_pwm_since) >= EV_CLOSE_MS * 1000) { ev_S2 = true; ev_log("EV closes S2 (state C)"); }
    if (ev_S2 && !pwm && (g_now_us - ev_nopwm_since) >= EV_OPEN_MS * 1000) { ev_S2 = false; ev_log("EV opens S2 (state B)"); }
  }
  // current drawn by the EV
  if (pin_contactor && ev_S2) {
    if (pwm) { double tgt = offered_A(); if (ev_I < tgt) ev_I = tgt; else if (ev_I > tgt) ev_I = std::max(tgt, ev_I - 24.0 * 1.2e-3 / (EV_RAMP_MS / 1000.0)); }
    else { double k = 1.0 - (g_now_us - ev_t_loss) / (EV_RAMP_MS * 1000.0); ev_I = ev_I_at_loss * (k > 0 ? k : 0); }
  } else ev_I = 0;
}

static double cp_level_V(bool high) {
  if (ev_leak_ohm > 0) { double v = 12.0 * ev_leak_ohm / (ev_leak_ohm + 1000.0); return high ? v : -v; }
  if (!ev_plugged) return high ? 12.0 : -12.0;
  double pos = ev_S2 ? 6.0 : 9.0;
  if (high) return pos;
  if (ev_diode) return -12.0;
  return ev_S2 ? -12.0 * 882.0 / 1882.0 : -12.0 * 2740.0 / 3740.0;
}
// CP front-end measured on the station (see src/board.h):
//   PIN_CP_POS (PC1) = positive half, fine scale: +12 V ~3667, clamped at ~0 for 0 V and below
//   PIN_CP_NEG (PC5) = whole CP, coarse scale:    +12 V ~2300, -12 V ~1475 (~34 counts/V)
static double cp_pos_adc(double v) { return v > 0 ? v * (3667.0 / 12.0) : 0.0; }
static double cp_neg_adc(double v) { return 1887.5 + v * (825.0 / 24.0); }

// CP voltage at time t (first-order RC on the last edge)
static double cp_v_at(uint64_t t) {
  uint16_t ticks = cp_compare_ticks;
  uint64_t ph = t % 1000;
  bool high = ph < ticks;
  if (getenv("INVERT")) high = !high;   // inverted CP output stage
  double tgt = cp_level_V(high);
  if (TAU_US <= 0 || ticks == 0 || ticks >= CP_ARR) return tgt;
  double since = high ? (double)ph : (double)(ph - ticks);
  double prev = cp_level_V(!high);
  return tgt + (prev - tgt) * exp(-since / TAU_US);
}
// PC5 sees the output stage BEFORE the 1 kOhm (measured): +12 V in the
// high phase, -12 V in the low phase, whatever the vehicle does.
static double cp_drive_v_at(uint64_t t) {
  uint16_t ticks = cp_compare_ticks;
  bool high = (t % 1000) < ticks;
  if (getenv("INVERT")) high = !high;
  return high ? 12.0 : -12.0;
}
// Outliers: NOISE_MAG keeps its historical unit (counts of the old 63 counts/V
// inverted front-end, + = lower CP voltage) so the scenario arguments stay valid.
static double cp_v_noisy(uint64_t t) {
  double v = cp_v_at(t);
  if (NOISE_P > 0 && std::uniform_real_distribution<double>(0, 1)(rng) < NOISE_P) v -= NOISE_MAG / 63.0;
  return v;
}

uint16_t analogRead(uint32_t pin) {
  uint64_t t_sample = g_now_us + (uint64_t)(ADC_US > 7 ? ADC_US - 7 : ADC_US);
  g_now_us += (uint64_t)ADC_US;
  double v = 0;
  switch (pin) {
    case PIN_CP_POS: v = cp_pos_adc(cp_v_noisy(t_sample)); break;
    case PIN_CP_NEG: v = cp_neg_adc(cp_drive_v_at(t_sample)); break;   // output stage, not the line
    case PIN_NTC:    v = 4095 - sim_ntc; break;          // PC2 falls when hot; sim_ntc is the "heat" value
    case PIN_VSENSE: { double s = sin(2 * M_PI * 50.0 * (double)t_sample / 1e6);
                       v = s > 0 ? 2920.0 * s : 0.0; break; }   // half-wave rectified mains
    case PIN_CT:     v = CT_ZERO_COUNT + ev_I / CT_A_PER_COUNT; break;   // mean of the half-wave CT signal (PC3)
    default: v = 0; break;
  }
  if (v < 0) v = 0; if (v > 4095) v = 4095;
  return (uint16_t)lround(v);
}

// ---------------- scenarios ----------------
struct Act { double t_s; std::function<void()> f; };
static uint64_t loop_max_us = 0;
static void run(double end_s, std::vector<Act> acts) {
  setup();
  size_t k = 0; uint64_t end_us = (uint64_t)(end_s * 1e6);
  bool prev_latched = false; uint8_t prev_err = 0;
  while (g_now_us < end_us) {
    while (k < acts.size() && g_now_us >= (uint64_t)(acts[k].t_s * 1e6)) { acts[k].f(); k++; }
    ev_update();
    uint64_t t0 = g_now_us;
    loop();
    g_now_us += 15;                       // misc loop overhead
    if (g_now_us - t0 > loop_max_us) loop_max_us = g_now_us - t0;
    if (g_latched != prev_latched || g_err != prev_err) {
      ev_log("latched=%d err=%u fault_count=%u", g_latched ? 1 : 0, (unsigned)g_err, (unsigned)g_fault_count);
      prev_latched = g_latched; prev_err = g_err;
    }
  }
}
static void summary(const char *name) {
  printf("=== %s ===\n", name);
  for (auto &e : events) printf("%s\n", e.c_str());
  printf("closes=%d opens=%d breaks_under_load(>1A)=%d max_break_I=%.1f A | IWDG max gap=%.1f ms | loop max=%.1f ms\n",
         n_close, n_open, n_break_under_load, max_break_I, IWatchdog.max_gap_us / 1000.0, loop_max_us / 1000.0);
  if (!status_lines.empty()) printf("first UART line: %s\nlast UART line : %s (len %zu)\n", status_lines.front().c_str(),
                                    status_lines.back().c_str(), status_lines.back().size());
}

int main(int argc, char **argv) {
  int sc = argc > 1 ? atoi(argv[1]) : 1;
  if (getenv("SEED")) rng.seed(atoi(getenv("SEED")));
  if (argc > 2) ADC_US = atof(argv[2]);
  if (argc > 3) TAU_US = atof(argv[3]);
  if (argc > 4) VERBOSE = atoi(argv[4]) != 0;
  if (argc > 5) g_np = atof(argv[5]);
  if (argc > 6) g_nm = atoi(argv[6]);
  switch (sc) {
    case 1: // EV plugged at boot, charges; ENABLE 0 at 120 s -> how long until the contactor opens?
      ev_plugged = true;
      run(130, { {120.0, [] { send_cmd("ENABLE 0"); }} });
      summary("S1 ENABLE 0 while charging (graceful stop?)"); break;
    case 2: // solar pause: SET 0 at 20 s (closed at ~0.6 s), resume SET 10 at 70 s
      ev_plugged = true;
      run(200, { {1.0, [] { send_cmd("SET 16"); }}, {20.0, [] { send_cmd("SET 0"); }},
                 {40.0, [] { send_cmd("SET 0"); }}, {70.0, [] { send_cmd("SET 10"); }},
                 {100.0, [] { send_cmd("SET 10"); }}, {130.0, [] { send_cmd("SET 10"); }} });
      summary("S2 solar pause / resume (anti short-cycle)"); break;
    case 3: // EV without diode
      ev_plugged = true; ev_diode = false;
      run(260, {});
      summary("S3 no diode: retries"); break;
    case 4: // thermal: NTC above PANIC 10..15 s, then cold
      ev_plugged = true;
      run(200, { {10.0, [] { sim_ntc = 3500; ev_log("NTC -> 3500"); }}, {15.0, [] { sim_ntc = 2900; ev_log("NTC -> 2900"); }},
                 {40.0, [] { sim_ntc = 1000; ev_log("NTC -> 1000"); }} });
      summary("S4 thermal cut / cooldown"); break;
    case 5: // ENABLE toggling every 2 s
      ev_plugged = true; {
        std::vector<Act> a;
        for (int i = 0; i < 20; i++) a.push_back({ 5.0 + i * 2.0, [i] { send_cmd(i % 2 ? "ENABLE 1" : "ENABLE 0"); } });
        run(50, a);
      }
      summary("S5 ENABLE toggled every 2 s"); break;
    case 6: // EV opens S2 by itself while charging (end of charge)
      ev_plugged = true;
      run(40, { {30.0, [] { ev_auto = false; ev_S2 = false; ev_I = 0; ev_log("EV opens S2 by itself (I->0)"); }} });
      summary("S6 EV ends charge (C->B)"); break;
    case 7: // isolated CP outliers, 1 h
      ev_plugged = true; NOISE_P = 0.01; NOISE_MAG = -400;
      run(3600, {});
      summary("S7 1% low outliers (-400 counts) for 1 h"); break;
    case 8: // isolated CP outliers HIGH (could fake the diode), EV without diode
      ev_plugged = true; ev_diode = false; NOISE_P = 0.05; NOISE_MAG = +1500;
      run(300, {});
      summary("S8 no diode + 5% high outliers"); break;
    case 9: // state B only (EV not ready, S2 open) under PWM, slow front-end -> false C?
      ev_plugged = true; ev_wants = false;
      run(60, {});
      summary("S9 EV in B with PWM (never wants)"); break;
    case 11: // charging established, then low outliers start, then the EV opens S2 (end of charge)
      ev_plugged = true;
      run(60, { {10.0, [] { NOISE_P = g_np; NOISE_MAG = g_nm; ev_log("noise ON p=%.4f mag=%d", g_np, g_nm); }},
                {20.0, [] { ev_auto = false; ev_S2 = false; ev_I = 0; ev_log("EV opens S2 by itself (I->0)"); }} });
      summary("S11 noise then EV opens S2"); break;
    case 12: // charging established, noise, then CP shorted to PE (state E) is simulated by unplug+fault? -> use diode loss
      ev_plugged = true;
      run(60, { {10.0, [] { NOISE_P = g_np; NOISE_MAG = g_nm; ev_log("noise ON p=%.4f mag=%d", g_np, g_nm); }},
                {20.0, [] { ev_diode = false; ev_log("EV diode lost (short)"); }} });
      summary("S12 noise then diode lost while charging"); break;
    case 13: // as S11 but offering 24 A (40 % duty: more samples on the high plateau)
      ev_plugged = true;
      run(60, { {0.1, [] { send_cmd("SET 24"); }}, {10.0, [] { NOISE_P = g_np; NOISE_MAG = g_nm; ev_log("noise ON p=%.4f mag=%d", g_np, g_nm); }},
                {20.0, [] { ev_auto = false; ev_S2 = false; ev_I = 0; ev_log("EV opens S2 by itself (I->0)"); }} });
      summary("S13 24 A, noise then EV opens S2"); break;
    case 14: // EV plugged at boot, noise from t=0 (no PWM yet: all samples on the plateau)
      ev_plugged = true; NOISE_P = g_np; NOISE_MAG = g_nm;
      run(120, {});
      summary("S14 noise from boot, EV plugged"); break;
    case 15: { // raw-state statistics: duty given in argv[7] (amps), EV state argv[8] (0 none,1 B,2 C)
      double a = argc > 7 ? atof(argv[7]) : 6; int evs = argc > 8 ? atoi(argv[8]) : 1;
      ev_auto = false; ev_plugged = evs > 0; ev_S2 = evs == 2; NOISE_P = g_np; NOISE_MAG = g_nm;
      setup(); cp_set_duty(a >= 100 ? 100.0f : AMP_TO_DUTY(a));
      int cnt[4] = {0,0,0,0}; long nsum = 0; int pos_min = 99999, pos_max = 0;
      for (int i = 0; i < 20000; i++) { g_now_us += 137; CpSample smp = cp_burst(); nsum += smp.n;
        if (smp.pos < pos_min) pos_min = smp.pos; if (smp.pos > pos_max) pos_max = smp.pos; cnt[cp_state_raw()]++; }
      printf("S15 offer=%.1f A ev=%d adc_us=%.0f tau=%.0f noise p=%.4f mag=%d : raw A=%d B=%d C=%d F=%d (of 20000) | burst n avg=%.1f | pos range %d..%d\n",
             a, evs, ADC_US, TAU_US, g_np, g_nm, cnt[0], cnt[1], cnt[2], cnt[3], nsum / 20000.0, pos_min, pos_max);
      break; }
#ifdef DIAG_MODE
    case 20: // diag: EV plugged, CHARGE ON at 5 s, SCAN at 20 s, CHARGE ON again 25 s, EV opens S2 at 40 s, unplug 50 s
      ev_plugged = true;
      run(60, { {5.0, [] { send_cmd("CHARGE ON"); }}, {20.0, [] { send_cmd("SCAN"); }}, {25.0, [] { send_cmd("CHARGE ON"); }},
                {40.0, [] { ev_auto = false; ev_S2 = false; ev_I = 0; ev_log("EV opens S2 by itself (I->0)"); }},
                {45.0, [] { ev_S2 = true; ev_log("EV closes S2 again"); }},
                {50.0, [] { ev_plugged = false; ev_log("EV unplugged"); }}, {52.0, [] { ev_plugged = true; ev_S2 = true; ev_log("EV replugged in C"); }} });
      summary("D20 diag interlock / SCAN");
      { int scan = 0; for (auto &l : status_lines) if (!strncmp(l.c_str(), "SCAN", 4)) { scan++; if (scan <= 3) puts(l.c_str()); } printf("SCAN lines: %d\n", scan); }
      break;
    case 21: // diag: no CHARGE ON at all, EV plugged and in C for 60 s
      ev_plugged = true;
      run(60, {});
      summary("D21 diag without CHARGE ON"); break;
    case 23: // diag: CHARGE ON with the EV in C
      ev_plugged = true;
      run(10, { {2.0, [] { send_cmd("CHARGE ON"); }} });
      summary("D23 diag CHARGE ON"); break;
    case 22: // diag: OFFER bounds
      ev_plugged = true;
      run(10, { {1.0, [] { send_cmd("OFFER 32"); }}, {2.0, [] { send_cmd("OFFER nan"); }}, {3.0, [] { send_cmd("OFFER 5.9"); }}, {4.0, [] { send_cmd("OFFER 16"); }} });
      summary("D22 diag OFFER bounds"); printf("diag_offer_a=%.1f\n", diag_offer_a); break;
    case 24: // diag v2: standby at boot, OFFER 10 at 2 s, CHARGE ON at 4 s, SCAN at 8 s (under load), unplug at 20 s
      ev_plugged = true;
      run(25, { {2.0, [] { send_cmd("OFFER 10"); }}, {4.0, [] { send_cmd("CHARGE ON"); }}, {8.0, [] { send_cmd("SCAN"); }},
                {20.0, [] { ev_plugged = false; ev_log("EV unplugged"); }} });
      summary("D24 diag v2: OFFER then CHARGE ON, SCAN under load, unplug");
      { int scan = 0; for (auto &l : status_lines) if (!strncmp(l.c_str(), "SCAN", 4)) scan++; printf("SCAN lines: %d\n", scan);
        for (auto &l : status_lines) if (!strncmp(l.c_str(), "DIAG", 4) && strstr(l.c_str(), "chg=1")) { printf("first chg=1 line: %s\n", l.c_str()); break; } }
      break;
#endif
    case 16: // evse build, EV plugged, 20 s
      ev_plugged = true; run(20, {}); summary("S16 evse boot with EV"); break;
#ifndef DIAG_MODE
    case 25: { // TIME (screen clock): accepted forms, rejected lines, midnight rollover, no TIME for days
      setup();
      struct { const char *line; int expect; } T[] = {     // expect = clock_minutes() after the line
        {"TIME 12:34", 754},    {"TIME 24:00", 754},     {"TIME 12:60", 754},   {"TIME 1:23", 754},
        {"TIME 12:34:5", 754},  {"TIME 12:34:56x", 754}, {"TIME", 754},         {"TIME 07:05:00", 425},
        {"TIME 23:59:30", 1439} };
      int bad = 0;
      printf("=== S25 TIME command ===\nbefore any TIME: %d (expect -1)\n", clock_minutes());
      if (clock_minutes() != -1) bad++;
      for (auto &t : T) {
        send_cmd(t.line); esp_poll();
        int m = clock_minutes(); if (m != t.expect) bad++;
        printf("%-16s -> %4d (expect %4d)\n", t.line, m, t.expect);
      }
      g_now_us += 31 * 1000000ULL;                        // 23:59:30 + 31 s
      int m = clock_minutes(); if (m != 0) bad++;
      printf("31 s later      -> %4d (expect    0, midnight rollover)\n", m);
      int prev = m, steps = 0;                            // no TIME any more: it keeps counting,
      for (int k = 0; k < 60 * 24 * 60; k++) {            // also across the millis() wrap (49.7 d)
        g_now_us += 60 * 1000000ULL;
        int n = clock_minutes(); if (n != (prev + 1) % 1440) steps++;
        prev = n;
      }
      if (steps) bad++;
      printf("60 days without TIME, read every minute: %d irregular step(s) (expect 0), now %d\n", steps, prev);
      send_cmd("TIME 08:00"); esp_poll();
      m = clock_minutes(); if (m != 480) bad++;
      printf("TIME 08:00      -> %4d (expect  480)\n%s\n", m, bad ? "FAIL" : "PASS");
      break; }
    case 26: { // INFO EDF (house supply, screen only): accepted lines, ignored ones, forgotten when stale
      setup();
      struct { const char *line; int expect; } T[] = {     // expect = edf_now() after the line
        {"INFO EDF 0", 0}, {"INFO EDF 1", 1}, {"INFO EDF 2", 1}, {"INFO EDF", 1}, {"INFO EDF 0 ", 1},
        {"INFO SOL 2400", 1}, {"EDF 0", 1}, {"INFO EDF 0", 0} };
      int bad = 0;
      printf("=== S26 INFO EDF ===\nbefore any INFO: %d (expect -1)\n", edf_now());
      if (edf_now() != -1) bad++;
      for (auto &t : T) {
        send_cmd(t.line); esp_poll();
        int e = edf_now(); if (e != t.expect) bad++;
        printf("%-16s -> %2d (expect %2d)\n", t.line, e, t.expect);
      }
      g_now_us += (uint64_t)(EDF_STALE_MS - 1000) * 1000ULL;
      int e = edf_now(); if (e != 0) bad++;
      printf("%lu s later     -> %2d (expect  0)\n", (unsigned long)(EDF_STALE_MS / 1000 - 1), e);
      g_now_us += 2000ULL * 1000ULL;
      e = edf_now(); if (e != -1) bad++;
      printf("stale           -> %2d (expect -1)\n%s\n", e, bad ? "FAIL" : "PASS");
      break; }
    case 27: { // PRESET (the Home Assistant page): every position, bad lines ignored, MODE MAN after
      setup();
      struct { const char *line; int sel; } T[] = {         // sel = g_sel after the line (3 = AUTO)
        {"PRESET 16", 1}, {"PRESET 24", 2}, {"PRESET 12", 2}, {"PRESET 16A", 2}, {"PRESET", 2},
        {"PRESET auto", 2}, {"PRESET 10", 0}, {"PRESET AUTO", 3}, {"MODE MAN", 0}, {"PRESET AUTO", 3} };
      int bad = 0;
      printf("=== S27 PRESET ===\n");
      for (auto &t : T) {
        send_cmd(t.line); esp_poll();
        bool ok = g_sel == t.sel && (g_mode == MODE_AUTO) == (t.sel == SEL_AUTO);
        if (!ok) bad++;
        printf("%-12s -> sel %d %s (expect %d)\n", t.line, g_sel, g_mode == MODE_AUTO ? "AUTO" : "MAN", t.sel);
      }
      printf("%s\n", bad ? "FAIL" : "PASS");
      break; }
#endif
    case 18: // no EV, a static 882 Ohm CP-PE leak (C window, no B ever): must never close (ERR 10)
      ev_leak_ohm = 882.0; run(120, {}); summary("S18 static CP-PE leak in the C window"); break;
    case 19: // EV still in C at boot (S2 closed, e.g. after a power cut); it opens S2 after 1 s
             // without PWM (EV_OPEN_MS): must close once B has been seen, after one ERR 10 retry
      ev_plugged = true; ev_S2 = true; run(60, {}); summary("S19 EV in C at boot"); break;
    case 17: // boot after a WATCHDOG reset with the EV plugged: first closing must wait MIN_PAUSE_MS
      sim_rcc_iwdg_flag() = 1; ev_plugged = true; run(70, {}); summary("S17 boot after IWDG reset (R7)"); break;
    case 10: // unplug while charging
      ev_plugged = true;
      run(40, { {30.0, [] { ev_plugged = false; ev_log("EV unplugged"); }} });
      summary("S10 unplug while charging"); break;
  }
  return 0;
}
