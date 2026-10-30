// Interactive, real-time simulator of the firmware. tools/sim/live.py is the
// browser front-end; this binary speaks a line protocol on stdin / stdout.
// Same EV / CP / ADC model as sim.cpp (copied, kept in sync by hand so the
// regression bench stays untouched) + the virtual ILI9163C of mock/panel.h.
// Built as `simlive` (operational firmware) and `simlive_diag` (-DDIAG_MODE).
//
// stdin, one command per line:
//   plug | unplug         EV connected / removed
//   ready <0|1>           EV wants to charge (closes S2 under PWM) / stops
//   diode <0|1>           EV diode present / missing
//   ntc <counts>          NTC ADC value (board.h: 3000 throttle, 3300 panic)
//   tau <us>              CP front-end RC time constant
//   btn [ms]              press the front button (default 150 ms)
//   uart <text>           a line to the firmware UART ("SET 11", "MODE MAN"...)
//   speed <x>             simulation speed, 0 = pause
//   quit
// stdout, one message per line:
//   STATE {json}          ~5 Hz: sim time, EV, contactor, firmware variables
//   UART <line>           every line the firmware prints
//   LOG <text>            events (contactor, EV, faults, commands)
//   FRAME <hex>           128x160 RGB888 when the screen changed, <= 10 fps
#define DISPLAY_HOST_SIM
#include "Arduino.h"
#include "IWatchdog.h"
#include <vector>
#include <random>
#include <algorithm>
#include <chrono>
#include <poll.h>
#include <unistd.h>

// ---------------- simulation clock & config ----------------
static uint64_t g_now_us = 0;
static double   ADC_US   = 10.0;     // measured on the station (adc=10)
static double   TAU_US   = 10.0;     // realistic front-end by default
static double   NOISE_P  = 0.0;
static int      NOISE_MAG = 0;
static std::mt19937 rng(12345);

// ---------------- EV model (same as sim.cpp) ----------------
static bool   ev_plugged = false, ev_diode = true, ev_wants = true, ev_S2 = false;
static double ev_leak_ohm = 0;              // > 0: no EV, a plain resistor CP-PE (water in the plug...)
static bool   ev_auto = true;
static double EV_CLOSE_MS = 300, EV_OPEN_MS = 1000, EV_RAMP_MS = 1000;
static uint64_t ev_pwm_since = 0, ev_nopwm_since = 0; static bool ev_pwm_prev = false;
static double ev_I = 0, ev_I_at_loss = 0; static uint64_t ev_t_loss = 0;
static int sim_ntc = 1008;              // NTC "heat" value: ~31.7 degC (measured on the station)
static bool sim_btn = false; static uint64_t btn_until = 0;

// ---------------- recorders ----------------
static bool pin_contactor = false; static uint64_t t_contactor_change = 0;
static std::vector<std::string> events;
static void ev_log(const char *fmt, ...) {
  char b[256]; va_list ap; va_start(ap, fmt); vsnprintf(b, sizeof b, fmt, ap); va_end(ap);
  char t[300]; snprintf(t, sizeof t, "[%9.3f s] %s", g_now_us / 1e6, b); events.push_back(t);
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
  if (tx_end_us < g_now_us) tx_end_us = g_now_us;
  tx_end_us += (uint64_t)(n * 86.8);
  if (tx_end_us - g_now_us > (uint64_t)(64 * 86.8)) g_now_us = tx_end_us - (uint64_t)(64 * 86.8);
  for (size_t i = 0; i < n; i++) {
    char c = (char)buf[i];
    if (c == '\n') { status_lines.push_back(line); line.clear(); } else line += c;
  }
  return n;
}
static void send_cmd(const char *s) { for (const char *p = s; *p; p++) SerialJ12.rx.push_back(*p); SerialJ12.rx.push_back('\n'); ev_log("ESP -> \"%s\"", s); }

uint32_t millis() { return (uint32_t)(g_now_us / 1000); }
uint32_t micros() { return (uint32_t)g_now_us; }
void delay(uint32_t ms) { g_now_us += (uint64_t)ms * 1000; }
void pinMode(uint32_t, uint32_t) {}
int  digitalRead(uint32_t pin) { return pin == PE14 ? (sim_btn ? 0 : 1) : 0; }

// ---------------- firmware under test ----------------
#include "../../src/main.cpp"
#include "../../src/display.cpp"
#include "panel.h"

static bool pwm_present() { return cp_compare_ticks > 0 && cp_compare_ticks < CP_ARR; }
static double ev_seen_duty() { double d = cp_compare_ticks / 10.0; return getenv("INVERT") ? 100.0 - d : d; }
static double offered_A() { return pwm_present() ? DUTY_TO_AMP(cp_compare_ticks / 10.0) : 0.0; }

void digitalWrite(uint32_t pin, uint32_t val) {
  if (pin != PB1) return;
  bool on = (val == CONTACTOR_ON);
  if (on != pin_contactor) {
    if (on) ev_log("CONTACTOR CLOSE (offer %.1f A, duty seen by the EV %.1f %%, I_ev %.1f A)", offered_A(), ev_seen_duty(), ev_I);
    else {
      ev_log("CONTACTOR OPEN  (I_ev %.1f A, closed for %.1f s)%s", ev_I, (g_now_us - t_contactor_change) / 1e6,
             ev_I > 1.0 ? "  ** BREAK UNDER LOAD **" : "");
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
    if (ev_S2 && !ev_wants) { ev_S2 = false; ev_log("EV opens S2 by itself (charge complete)"); }
  }
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

// ---------------- live loop ----------------
using clk = std::chrono::steady_clock;
static double wall_us(clk::time_point ref) { return std::chrono::duration<double, std::micro>(clk::now() - ref).count(); }

static bool stdin_line(std::string &out) {
  struct pollfd p = { 0, POLLIN, 0 };
  if (poll(&p, 1, 0) <= 0) return false;
  static std::string buf; char c;
  while (read(0, &c, 1) == 1) {
    if (c == '\n') { out = buf; buf.clear(); return true; }
    buf += c;
    struct pollfd q = { 0, POLLIN, 0 }; if (poll(&q, 1, 0) <= 0) break;
  }
  return false;
}

static void emit_state(double speed) {
  printf("STATE {\"t\":%.3f,\"speed\":%g,\"plugged\":%d,\"S2\":%d,\"wants\":%d,\"diode\":%d,\"I\":%.1f,"
         "\"duty\":%.1f,\"offer\":%.1f,\"pwm\":%d,\"contactor\":%d,\"ntc\":%d,\"tau\":%g,"
         "\"sel\":%u,\"mode\":\"%s\",\"enabled\":%d,\"setpoint\":%.1f,\"latched\":%d,\"err\":%u,\"btn\":%d}\n",
         g_now_us / 1e6, speed, ev_plugged, ev_S2, ev_wants, ev_diode, ev_I,
         cp_compare_ticks / 10.0, offered_A(), pwm_present(), pin_contactor, sim_ntc, TAU_US,
         (unsigned)g_sel, g_mode == MODE_AUTO ? "AUTO" : "MAN", g_enabled ? 1 : 0, (double)g_setpoint_a,
         g_latched ? 1 : 0, (unsigned)g_err, sim_btn ? 1 : 0);
}

int main() {
  setvbuf(stdout, nullptr, _IOLBF, 0);
  double speed = 1.0; bool running = true;
  setup();
  ev_log("firmware booted (%s build)",
#ifdef DIAG_MODE
         "diag"
#else
         "evse"
#endif
  );
  auto wall_ref = clk::now(); uint64_t sim_ref = g_now_us;
  auto t_state = clk::now(), t_frame = clk::now();
  size_t ev_sent = 0, uart_sent = 0;
  std::string cmd, hex;

  while (running) {
    while (stdin_line(cmd)) {
      const char *c = cmd.c_str();
      if      (cmd == "plug")                 { ev_plugged = true;  ev_log("EV plugged"); }
      else if (cmd == "unplug")               { ev_plugged = false; ev_log("EV unplugged"); }
      else if (!strncmp(c, "ready ", 6))      { ev_wants = atoi(c + 6) != 0; ev_log("EV wants to charge: %d", ev_wants); }
      else if (!strncmp(c, "diode ", 6))      { ev_diode = atoi(c + 6) != 0; ev_log("EV diode: %d", ev_diode); }
      else if (!strncmp(c, "ntc ", 4))        { sim_ntc = atoi(c + 4); ev_log("NTC -> %d", sim_ntc); }
      else if (!strncmp(c, "tau ", 4))        { TAU_US = atof(c + 4); ev_log("CP tau -> %g us", TAU_US); }
      else if (!strncmp(c, "btn", 3))         { int ms = atoi(c + 3); if (ms <= 0) ms = 150; sim_btn = true; btn_until = g_now_us + (uint64_t)ms * 1000; ev_log("button pressed %d ms", ms); }
      else if (!strncmp(c, "uart ", 5))       { send_cmd(c + 5); }
      else if (!strncmp(c, "speed ", 6))      { speed = atof(c + 6); if (speed < 0) speed = 0; wall_ref = clk::now(); sim_ref = g_now_us; ev_log("speed x%g", speed); }
      else if (cmd == "quit")                 { running = false; }
      else ev_log("unknown command: %s", c);
    }
    if (speed > 0) {
      uint64_t target = sim_ref + (uint64_t)(wall_us(wall_ref) * speed);
      int n = 0;
      while (g_now_us < target && n++ < 500) {
        if (sim_btn && g_now_us >= btn_until) sim_btn = false;
        ev_update();
        loop();
        g_now_us += 15;
      }
      if (g_now_us >= target) usleep(4000);
      else { wall_ref = clk::now(); sim_ref = g_now_us; }   // cannot keep up: resync, no spiral
    } else usleep(20000);

    for (; ev_sent < events.size(); ev_sent++)   printf("LOG %s\n", events[ev_sent].c_str());
    for (; uart_sent < status_lines.size(); uart_sent++) printf("UART %s\n", status_lines[uart_sent].c_str());
    if (events.size() > 4000)       { events.erase(events.begin(), events.begin() + 2000); ev_sent -= 2000; }
    if (status_lines.size() > 4000) { status_lines.erase(status_lines.begin(), status_lines.begin() + 2000); uart_sent -= 2000; }
    auto now = clk::now();
    if (std::chrono::duration<double>(now - t_state).count() >= 0.2) { t_state = now; emit_state(speed); }
    if (g_panel_dirty && std::chrono::duration<double>(now - t_frame).count() >= 0.1) {
      t_frame = now; g_panel_dirty = false; panel_hex(hex); printf("FRAME %s\n", hex.c_str());
    }
  }
  return 0;
}
