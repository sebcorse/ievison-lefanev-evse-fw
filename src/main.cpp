// ============================================================================
//  main.cpp - Custom EVSE firmware (IEVISON / Lefanev LE-F30 charging station)
//  STM32F103VET6 · PlatformIO + STM32duino
//
//  Scope: replicate the stock behaviour (CP, contactor, monitoring, screen,
//  front button, LED bar) and ADD a current setpoint over the RS-232 port (J12),
//  so that an energy manager can modulate the charging current (variable CP duty
//  cycle). Earth-leakage protection is autonomous hardware (FM2147) -> nothing to
//  code here.
//
//  State-machine / safety logic is modelled on the OpenEVSE project (state
//  debounce, two-stage thermal management). The diode check, impossible on this
//  board, is replaced by a sequence check (B before C, see cp_track_sequence).
//  Every behaviour is also replayed on a host simulation bench (tools/sim).
// ============================================================================
#include <Arduino.h>
#include <IWatchdog.h>
#include <algorithm>
#include <math.h>
#include <string.h>
#include "board.h"

#ifdef ARDUINO_ARCH_STM32
Uart SerialJ12(SERIAL_ESP_RX, SERIAL_ESP_TX);   // USART1 -> SP3232 -> J12 (see board.h)
#endif
#include "display.h"

// ---------------------------------------------------------------------------
//  Clock: 12 MHz HSE crystal -> PLL x6 -> 72 MHz (override the 8 MHz default)
// ---------------------------------------------------------------------------
extern "C" void SystemClock_Config(void) {
  // Started by the ROM bootloader's "go" (remote update, see bootloader_check):
  // the clocks are still the bootloader's (PLL on HSI) and HAL refuses to
  // reconfigure a PLL that drives SYSCLK -> HAL_ERROR -> the while(1) below.
  // Back to the reset state first (a no-op after a normal power-on/reset).
  HAL_RCC_DeInit();
  RCC_OscInitTypeDef osc = {};
  osc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  osc.HSEState       = RCC_HSE_ON;
  osc.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  osc.PLL.PLLState   = RCC_PLL_ON;
  osc.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
  osc.PLL.PLLMUL     = RCC_PLL_MUL6;             // 12 MHz x6 = 72 MHz
  if (HAL_RCC_OscConfig(&osc) != HAL_OK) { while (1) {} }

  RCC_ClkInitTypeDef clk = {};
  clk.ClockType      = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK |
                       RCC_CLOCKTYPE_PCLK1  | RCC_CLOCKTYPE_PCLK2;
  clk.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
  clk.AHBCLKDivider  = RCC_SYSCLK_DIV1;          // HCLK  = 72 MHz
  clk.APB1CLKDivider = RCC_HCLK_DIV2;            // PCLK1 = 36 MHz (max)
  clk.APB2CLKDivider = RCC_HCLK_DIV1;            // PCLK2 = 72 MHz
  if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_2) != HAL_OK) { while (1) {} }

  // ADC clock: the F1 ADC is limited to 14 MHz. The reset value of ADCPRE is
  // PCLK2/2 = 36 MHz, and STM32duino leaves this to the variant's clock config
  // (which this function replaces) -> set it explicitly. 72/6 = 12 MHz.
  RCC_PeriphCLKInitTypeDef pclk = {};
  pclk.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  pclk.AdcClockSelection    = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&pclk) != HAL_OK) { while (1) {} }
}

// ---------------------------------------------------------------------------
//  Fault causes (reported as ERR=<n> in the status line)
// ---------------------------------------------------------------------------
enum ErrCode : uint8_t {
  ERR_NONE = 0, ERR_DIODE = 1, ERR_STATE_DE = 2, ERR_OVERTEMP = 3, ERR_TIMER = 4,
  ERR_OVERCURRENT = 5, ERR_WELD = 6, ERR_ADC = 7, ERR_NTC = 8, ERR_POLARITY = 9,
  ERR_SEQUENCE = 10   // C plateau without the B plateau first (leak? see cp_track_sequence)
};

// ---------------------------------------------------------------------------
//  Reset cause: after a watchdog reset, hold the first closing for a while
//  (a reset loop must not cycle the contactor under load).
// ---------------------------------------------------------------------------
static bool g_boot_after_iwdg = false;
static void boot_reset_cause() {
  g_boot_after_iwdg = (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST) != 0);
  __HAL_RCC_CLEAR_RESET_FLAGS();
}

// ---------------------------------------------------------------------------
//  Contactor (default-safe state = OPEN), with state tracking for the cycle
//  limits, the graceful stop and the welded-contact rule.
// ---------------------------------------------------------------------------
static bool     g_contactor_closed = false;
static uint32_t g_t_contactor      = 0;        // time of the last change
static bool     g_had_open         = false;    // an opening happened (or a watchdog reset)
static uint32_t g_last_open_ms     = 0;
static void contactor(bool on) {
  if (on != g_contactor_closed) {
    g_contactor_closed = on; g_t_contactor = millis();
    if (!on) { g_had_open = true; g_last_open_ms = millis(); }
  }
  digitalWrite(PIN_CONTACTOR, on ? CONTACTOR_ON : CONTACTOR_OFF);
}
// Re-closing is allowed only MIN_PAUSE_MS after ANY opening, and MIN_PAUSE_MS
// after boot when the reset came from the watchdog.
static bool may_close() {
  return !g_had_open || (millis() - g_last_open_ms) >= MIN_PAUSE_MS;
}

// ---------------------------------------------------------------------------
//  ADC: direct ADC1 driver, used for EVERY analog read (never analogRead()).
//  STM32duino's analogRead() re-initialises, re-calibrates and resets ADC1 on
//  each call: ~100 us per read (only 11 samples per CP burst) and enough
//  disturbance to flip the CP stage while its input floated. Here ADC1 is set up
//  and calibrated once, then a read is a
//  single software-triggered conversion: ~7 us on the two CP channels (71.5
//  cycles), ~21 us elsewhere (239.5 cycles, for the high-impedance dividers).
//  The host simulation (tools/sim) has no ADC registers and keeps analogRead().
// ---------------------------------------------------------------------------
static bool     g_adc_ok       = false;   // calibration finished (BOOT line)
static uint32_t g_adc_timeouts = 0;       // conversions that never ended (should stay 0)
static uint16_t g_adc_pin_mask = 0;       // channels whose pin is already in analog mode

static uint8_t adc_channel(uint32_t pin) {
  switch (pin) {
    case PA0: return 0;  case PA1: return 1;  case PA2: return 2;  case PA3: return 3;
    case PA4: return 4;  case PA5: return 5;  case PA6: return 6;  case PA7: return 7;
    case PB0: return 8;  case PB1: return 9;
    case PC0: return 10; case PC1: return 11; case PC2: return 12; case PC3: return 13;
    case PC4: return 14; case PC5: return 15;
    default:  return 0xFF;
  }
}

static void adc_init() {
#ifdef ARDUINO_ARCH_STM32
  __HAL_RCC_ADC1_CLK_ENABLE();                   // ADC clock = PCLK2/6 = 12 MHz (SystemClock_Config)
  ADC1->CR1  = 0;                                // independent mode, no scan
  ADC1->CR2  = ADC_CR2_EXTSEL | ADC_CR2_EXTTRIG; // regular group started by SWSTART, single conversion
  ADC1->SQR1 = 0;                                // one conversion per start
  ADC1->SMPR2 = 0x3FFFFFFFu;                     // ch0..9   : 239.5 cycles
  ADC1->SMPR1 = 0x00FFFFFFu;                     // ch10..17 : 239.5 cycles ...
  const uint8_t cpch[2] = { adc_channel(PIN_CP_POS), adc_channel(PIN_CP_NEG) };
  for (uint8_t ch : cpch) {                      // ... except the CP channels: 71.5 cycles
    if (ch >= 10 && ch <= 17) ADC1->SMPR1 = (ADC1->SMPR1 & ~(7u << (3 * (ch - 10)))) | (6u << (3 * (ch - 10)));
    else if (ch <= 9)         ADC1->SMPR2 = (ADC1->SMPR2 & ~(7u << (3 * ch)))        | (6u << (3 * ch));
  }
  ADC1->CR2 |= ADC_CR2_ADON;                     // power up (tSTAB 1 us, >= 2 ADC clocks before CAL)
  delayMicroseconds(10);
  uint32_t t0 = micros();
  ADC1->CR2 |= ADC_CR2_RSTCAL; while ((ADC1->CR2 & ADC_CR2_RSTCAL) && (micros() - t0) < 2000) {}
  t0 = micros();
  ADC1->CR2 |= ADC_CR2_CAL;    while ((ADC1->CR2 & ADC_CR2_CAL)    && (micros() - t0) < 2000) {}
  g_adc_ok = !(ADC1->CR2 & (ADC_CR2_CAL | ADC_CR2_RSTCAL));
#else
  g_adc_ok = true;
#endif
}

static uint16_t adc_read(uint32_t pin) {
#ifdef ARDUINO_ARCH_STM32
  // Outputs must never be turned into analog inputs (a floating CP drive or
  // contactor line is exactly the failure this driver exists to avoid).
  if (pin == PIN_CP_GEN || pin == PIN_CONTACTOR) return 0;
  uint8_t ch = adc_channel(pin);
  if (ch > 15) return 0;
  if (!(g_adc_pin_mask & (1u << ch))) { pinMode(pin, INPUT_ANALOG); g_adc_pin_mask |= (uint16_t)(1u << ch); }
  ADC1->SQR3 = ch;
  ADC1->CR2 |= ADC_CR2_SWSTART;
  uint32_t guard = 4000;                         // >> 21 us at 72 MHz
  while (!(ADC1->SR & ADC_SR_EOC)) { if (!--guard) { g_adc_timeouts++; return 0; } }
  return (uint16_t)ADC1->DR;                     // reading DR clears EOC
#else
  return analogRead(pin);
#endif
}

// ---------------------------------------------------------------------------
//  CP generation: 1 kHz HARDWARE PWM on PIN_CP_GEN = PA7 = TIM3_CH2 (see
//  board.h). TIM3 runs at 1 tick = 1 us (explicit prescaler
//  from getTimerClkFreq()) with a period of CP_ARR = 1000 ticks -> 1 kHz. PWM
//  mode 1: the output is high (= +12 V) while CNT < CCR, so CCR in ticks IS the
//  +12 V time: duty 0..100 % <=> CCR 0..1000 ; 100 % = CCR > ARR = never low =
//  steady +12 V (state A, no offer) ; 0 % = steady -12 V.
//  setOverflow(1000, MICROSEC_FORMAT) would let the library pick the prescaler
//  (ARR 35999, duty 36x too short): hence the explicit prescaler and the runtime
//  guard g_timer_ok (safe state if the timer differs).
// ---------------------------------------------------------------------------
static HardwareTimer *cpTimer = nullptr;
static const uint16_t CP_ARR = 1000;                  // ticks per period (1 us each)
static volatile uint16_t cp_compare_ticks = CP_ARR;   // 0..1000 = +12 V time in 0.1 %
static bool g_timer_ok = false;                       // runtime guard, see cp_init()
// The negative half can only be checked while a low phase exists well inside the
// period: CCR <= 650 ticks (duty <= 65 % = 39 A, above our 24 A cap).
static const uint16_t CP_NEG_MAX_TICKS = 650;

static void cp_set_duty(float pct) {
  // Anything not strictly inside [0, 100] - NaN included, its comparisons are
  // false - falls back to 100 % = standby, +12 V, no offer.
  if (!(pct >= 0.0f && pct <= 100.0f)) pct = 100.0f;
  cp_compare_ticks = (uint16_t)(pct * (CP_ARR / 100.0f) + 0.5f);
  if (cpTimer) cpTimer->setCaptureCompare(CP_TIM_CHANNEL, cp_compare_ticks, TICK_COMPARE_FORMAT);
}
static inline bool cp_pwm_active() { return cp_compare_ticks <= CP_NEG_MAX_TICKS; }

static void cp_init() {
  pinMode(PIN_CP_GEN, OUTPUT);              // never leave the CP drive floating:
  digitalWrite(PIN_CP_GEN, CP_GEN_INVERTED ? LOW : HIGH);   // +12 V (state A) until the PWM runs
  cpTimer = new HardwareTimer(TIM3);        // TIM3 is free (tone = TIM6, Servo = TIM7)
  uint32_t psc = cpTimer->getTimerClkFreq() / 1000000UL;   // 72 MHz -> 1 tick = 1 us
  cpTimer->setPrescaleFactor(psc);
  cpTimer->setOverflow(CP_ARR, TICK_FORMAT);              // period = 1000 ticks
  cpTimer->setMode(CP_TIM_CHANNEL, CP_GEN_INVERTED ? TIMER_OUTPUT_COMPARE_PWM2 : TIMER_OUTPUT_COMPARE_PWM1,
                   CP_TIM_PIN);
  cpTimer->setCaptureCompare(CP_TIM_CHANNEL, CP_ARR, TICK_COMPARE_FORMAT);   // 100 % = standby
  cpTimer->resume();
  // Guard: every CP timing below assumes 1 tick = 1 us and CP_ARR ticks/period.
  g_timer_ok = (cpTimer->getOverflow(TICK_FORMAT) == CP_ARR) &&
               (cpTimer->getPrescaleFactor() == psc) && (psc >= 8);
}

// One-line boot report: timer parameters read back from the hardware, the
// ADC calibration result, the measured duration of one CP read (the burst
// depends on it) and the reset cause.
static uint32_t g_adc_us = 0;
static void cp_report_boot(Print &out) {
  (void)adc_read(PIN_CP_POS);                         // first call also sets the pin to analog
  uint32_t t0 = micros(); (void)adc_read(PIN_CP_POS); g_adc_us = micros() - t0;
  out.printf("BOOT fw=evse-fw build=" __DATE__ "_" __TIME__ " tim_clk=%lu psc=%lu ovf=%lu timer_ok=%d adc_ok=%d adc_us=%lu iwdg_rst=%d\n",
             (unsigned long)cpTimer->getTimerClkFreq(),
             (unsigned long)cpTimer->getPrescaleFactor(),
             (unsigned long)cpTimer->getOverflow(TICK_FORMAT), g_timer_ok ? 1 : 0,
             g_adc_ok ? 1 : 0, (unsigned long)g_adc_us, g_boot_after_iwdg ? 1 : 0);
}

// ---------------------------------------------------------------------------
//  CP reading (as OpenEVSE's ReadPilot), TWO channels: PIN_CP_POS = PC1 sees
//  the positive half with a fine
//  scale (+12/+9/+6 V -> ~3667/2750/1834, clamped at ~0 for 0 V and below);
//  PIN_CP_NEG = PC5 sees the whole CP on a coarse scale (+12 V ~2300, -12 V
//  ~1475). Each is sampled for a bit more than one full period; the SECOND
//  highest POS sample gives the high plateau (A/B/C) and the SECOND lowest NEG
//  sample the level of the negative half (diode check); one isolated outlier
//  cannot fake a level. Edges only produce values in between, so they never disturb
//  the extremes. The fraction of POS samples above its mid-level measures the
//  duty cycle actually present on the CP: compared with the commanded duty it
//  catches an inverted or stuck output stage (10 % commanded would read as
//  90 % = 65 A for the EV).
// ---------------------------------------------------------------------------
enum CpState { CP_A, CP_B, CP_C, CP_FAULT };
struct CpSample { uint16_t pos; uint16_t neg; uint16_t n; uint16_t duty; };  // duty in 0.1 %
static const uint16_t CP_BURST_MAX = 160;             // sample PAIRS (~16 us each -> ~70 per burst)
static const uint16_t CP_BURST_MIN_N = 25;            // ~8 would accept 137 us reads

// The two channels are sampled in two consecutive bursts, never interleaved:
// PIN_CP_NEG is a very high-impedance node and, interleaved, it read back
// mostly the charge left on the ADC sample-and-hold by PIN_CP_POS (~3627
// during the +12 V phase instead of ~13). The first
// conversion after a channel change is discarded for the same reason.
// k-th smallest value (k from 1) of v[0..n-1]; reorders v.
static uint16_t kth_smallest(uint16_t *v, uint16_t n, uint16_t k) {
  if (n == 0) return 0;
  if (k < 1) k = 1;
  if (k > n) k = n;
  std::nth_element(v, v + (k - 1), v + n);
  return v[k - 1];
}
static uint16_t g_burst_n_neg = 0;                    // samples in the last NEG burst (diag)
static CpSample cp_burst(uint32_t duration_us = 1100) {
  static uint16_t bpos[CP_BURST_MAX], bneg[CP_BURST_MAX];
  uint16_t n = 0, m = 0;
  (void)adc_read(PIN_CP_POS);
  uint32_t t0 = micros();
  do {
    uint16_t v = adc_read(PIN_CP_POS);
    if (n < CP_BURST_MAX) bpos[n++] = v;
  } while ((uint32_t)(micros() - t0) < duration_us);
  (void)adc_read(PIN_CP_NEG);
  t0 = micros();
  do {
    uint16_t v = adc_read(PIN_CP_NEG);
    if (m < CP_BURST_MAX) bneg[m++] = v;
  } while ((uint32_t)(micros() - t0) < duration_us);
  g_burst_n_neg = m;
  // Rank filters scaled to the burst size (a fixed "2nd extreme" stops working
  // once a burst holds ~110 samples: with 5 % outliers there are ~5 per burst).
  //  - high plateau: the k-th highest POS sample, k = n/32 (>= 2): ~3 of 110,
  //    still inside the +V plateau at 10 % duty (~11 samples there);
  //  - negative half: the 10th percentile of NEG, k = n/10 (>= 2): tolerates
  //    up to ~9 % outliers and stays on the -V plateau up to 65 % duty.
  // nth_element() reorders the buffers; nothing below depends on their order.
  uint16_t kp = (n / 32 >= 2) ? (uint16_t)(n / 32) : 2, kn = (m / 10 >= 2) ? (uint16_t)(m / 10) : 2;
  CpSample s;
  s.n   = (n < m) ? n : m;                            // the weaker of the two bursts
  s.pos = kth_smallest(bpos, n, (n >= kp) ? (uint16_t)(n - kp + 1) : 1);   // high plateau level
  s.neg = kth_smallest(bneg, m, kn);                  // negative half level (lower = deeper)
  uint16_t plo = kth_smallest(bpos, n, 2);            // the -V half, clamped near 0 on POS
  uint16_t mid = (uint16_t)(((uint32_t)plo + s.pos) / 2);
  uint16_t on_high = 0;
  for (uint16_t i = 0; i < n; i++) if (bpos[i] > mid) on_high++;   // samples on the +V plateau
  s.duty = n ? (uint16_t)((uint32_t)on_high * 1000UL / n) : 0;
  return s;
}

// Duty self-check: commanded vs measured, tolerance widened for sparse bursts.
static bool cp_duty_plausible(const CpSample &s) {
  int expected = cp_compare_ticks;                     // 0.1 % units
  int tol = 120; int by_n = s.n ? (4000 / s.n) : 1000; // 12 % or 4 samples' worth
  if (by_n > tol) tol = by_n;
  int diff = (int)s.duty - expected; if (diff < 0) diff = -diff;
  return diff <= tol;
}

static ErrCode g_cp_err = ERR_NONE;        // cause of the last raw CP_FAULT

// ---------------------------------------------------------------------------
//  Sequence check, in place of the diode check this board cannot do (no input
//  sees the negative half of the CP line, see board.h).
//  C is accepted only once the B plateau (+9 V, S2 open) has been seen since
//  the vehicle was connected: a real EV shows B before it closes S2 in answer
//  to the PWM, whereas a resistive CP-PE leak (water in the plug...) sitting
//  in the C window never does. The flag survives solar pauses and is cleared
//  only after 8 bursts in a row at the A level (EV really gone), so one A
//  outlier does not clear it. Limits: a leak that starts in the B window and
//  drifts into C, or a vehicle without its diode, are still accepted.
// ---------------------------------------------------------------------------
static bool    g_seen_b = false;
static uint8_t g_a_run  = 0;               // consecutive bursts at the A level
static void cp_track_sequence(const CpSample &s) {
  if (s.pos >= CP_POS_AB) { if (g_a_run < 255) g_a_run++; if (g_a_run >= 8) g_seen_b = false; }
  else { g_a_run = 0; if (s.pos >= CP_POS_BC) g_seen_b = true; }
}

// Instantaneous (un-debounced) CP state. Rules (IEC 61851-1, as in OpenEVSE):
//  - the high plateau decides A / B / C ; D (+3 V, ventilation) and E (0 V,
//    short to PE) are treated as faults -> no charge ;
//  - ENTERING C requires the PWM to be active, the -12 V half to be generated
//    (output-stage check on PC5), the measured duty to match the commanded one
//    and the B plateau to have been seen first (sequence check). Without
//    PWM a "ready" plateau is reported as B, so the state machine first posts
//    the offer and re-evaluates ;
//  - EXCEPT while the contactor is already closed: a C plateau with the offer
//    withdrawn is the graceful ramp-down and stays C until the EV
//    opens S2 or the 3 s timer opens the contactor.
static CpState cp_state_raw() {
  CpSample s = cp_burst();
  cp_track_sequence(s);
  if (!g_adc_ok || s.n < CP_BURST_MIN_N) { g_cp_err = ERR_ADC; return CP_FAULT; }  // ADC not calibrated or far too slow
  if (s.pos >= CP_POS_AB)   return CP_A;
  if (s.pos <  CP_POS_CD)   { g_cp_err = ERR_STATE_DE; return CP_FAULT; }  // D / E / short to PE
  bool plateau_c = (s.pos < CP_POS_BC);
  if (!cp_pwm_active()) return (plateau_c && g_contactor_closed) ? CP_C : CP_B;
  if (s.neg > CP_NEG_MAX)     { g_cp_err = ERR_DIODE;    return CP_FAULT; }  // -12 V half not generated
  if (!cp_duty_plausible(s))  { g_cp_err = ERR_POLARITY; return CP_FAULT; }  // output stage / timing
  if (plateau_c && !g_contactor_closed && !g_seen_b) { g_cp_err = ERR_SEQUENCE; return CP_FAULT; }
  return plateau_c ? CP_C : CP_B;
}

// Debounced state (OpenEVSE-style): a candidate must persist before we commit.
// Leaving C (the EV opened S2, or a fault appeared) uses a short delay: IEC asks
// <= 100 ms to de-energise; with the 2nd-extreme filter 50 ms is not noise-prone.
// Plug-in detection (A -> B) tolerates a few contradicting bursts inside its
// window (sustained outliers must not hide a connected EV). Every
// other transition - in particular INTO C, which energises the connector -
// still requires an UNINTERRUPTED run of identical raw states.
static CpState cp_state_debounced() {
  static CpState committed = CP_A, cand = CP_A;
  static uint32_t t_cand = 0;
  static uint8_t  glitches = 0;
  CpState raw = cp_state_raw();
  if (raw != cand) {
    bool tolerant = (committed == CP_A && cand == CP_B);
    if (!tolerant || ++glitches > 3) { cand = raw; t_cand = millis(); glitches = 0; }
  } else if (glitches) glitches--;
  uint32_t need = DELAY_STATE_MS;
  if (cand == CP_A)                                        need = DELAY_STATE_A_MS;
  else if (committed == CP_C && (cand == CP_B || cand == CP_FAULT)) need = DELAY_STATE_CB_MS;
  if (cand != committed && (millis() - t_cand) >= need) { committed = cand; glitches = 0; }
  return committed;
}

// ---------------------------------------------------------------------------
//  Monitoring: raw ADC + conversion to engineering units (constants in board.h)
// ---------------------------------------------------------------------------
// NTC "heat" value: the PC2 reading FALLS when the sensor warms up, so it is
// mirrored here and every threshold keeps "higher = hotter".
static uint16_t ntc_raw()    { return (uint16_t)(4095 - adc_read(PIN_NTC)); }
// Mains: PIN_VSENSE carries a half-wave rectified 50 Hz signal -> keep the peak
// over one full period (20 ms); V_PER_COUNT converts that peak to RMS volts.
static uint16_t vsense_raw() {
  uint16_t pk = 0; uint32_t t0 = micros();
  do { uint16_t v = adc_read(PIN_VSENSE); if (v > pk) pk = v; } while ((uint32_t)(micros() - t0) < 20000);
  return pk;
}

static float volts() { return vsense_raw() * V_PER_COUNT; }

// Temperature from the "heat" value h (see board.h): Beta model anchored on the
// measured point (NTC_REF_HEAT <-> NTC_REF_C).
static float tempC_from(uint16_t h) {
  if (h == 0)    return -50.0f;                       // open sensor (also caught by NTC_PLAUSIBLE_MIN)
  if (h >= 4095) return 150.0f;                       // shorted sensor
  float ratio     = (4095.0f - h) / (float)h;         // R_ntc / R_fixed
  float ratio_ref = (4095.0f - NTC_REF_HEAT) / (float)NTC_REF_HEAT;
  float inv_t = 1.0f / (NTC_REF_C + 273.15f) + logf(ratio / ratio_ref) / NTC_BETA;
  return 1.0f / inv_t - 273.15f;
}

// AC RMS (in ADC counts, DC bias removed) over one measured 50 Hz cycle on a pin.
static float rms_counts(uint32_t pin) {
  uint32_t n = 0, sum = 0; uint64_t sumsq = 0;
  uint32_t t0 = micros();
  do {
    uint16_t s = adc_read(pin);
    sum += s; sumsq += (uint64_t)s * s; n++;
  } while ((uint32_t)(micros() - t0) < 20000);
  float mean = sum / (float)n;
  float var  = (float)((double)sumsq / n - (double)mean * mean);
  return var > 0 ? sqrtf(var) : 0.0f;
}
// Mean ADC value (in counts) over one measured 50 Hz cycle on a pin.
static float mean_counts(uint32_t pin) {
  uint32_t n = 0, sum = 0;
  uint32_t t0 = micros();
  do { sum += adc_read(pin); n++; } while ((uint32_t)(micros() - t0) < 20000);
  return sum / (float)n;
}
// CT current (amps). PIN_CT (PC3) carries a half-wave rectified 50 Hz signal:
// its MEAN over one mains period above the zero
// offset is proportional to the current (an AC RMS would not be). Used for the
// display and the metering once CT_MEASURE is set, for the protections only once
// CT_READY is set - the caller reports 0 before that.
static float current_a() {
  float c = mean_counts(PIN_CT) - (float)CT_ZERO_COUNT;
  return c > 0 ? c * CT_A_PER_COUNT : 0.0f;
}

// Session metering (energy + duration), integrated while charging.
static float    g_session_wh = 0;
static uint32_t g_session_start_ms = 0, g_meter_last_ms = 0;
static void meter_reset() {
  g_session_wh = 0; g_session_start_ms = millis(); g_meter_last_ms = millis();
}
static void meter_update(bool charging, float v, float i) {
  uint32_t now = millis();
  float dt_h = (now - g_meter_last_ms) / 3600000.0f;   // ms -> hours
  g_meter_last_ms = now;
  if (charging && v > 0 && i > 0) g_session_wh += v * i * dt_h;
}
static float    session_kwh() { return g_session_wh / 1000.0f; }
static uint32_t session_min() { return (millis() - g_session_start_ms) / 60000UL; }

// ---------------------------------------------------------------------------
//  Thermal management: median-filtered NTC, hysteresis on both thresholds,
//  latched cut with a minimum cooldown, plausibility window (an open or
//  unplugged sensor is a fault).
// ---------------------------------------------------------------------------
enum ThermalLevel { THERM_OK, THERM_THROTTLE, THERM_CUT, THERM_SENSOR };

static uint16_t ntc_filtered() {
  static uint16_t ring[NTC_MEDIAN_N];
  static uint8_t  idx = 0, filled = 0;
  static uint32_t t_last = 0;
  if (!filled || millis() - t_last >= 100) {
    ring[idx] = ntc_raw(); idx = (idx + 1) % NTC_MEDIAN_N;
    if (filled < NTC_MEDIAN_N) filled++;
    t_last = millis();
  }
  uint16_t tmp[NTC_MEDIAN_N];
  memcpy(tmp, ring, sizeof(tmp));
  for (uint8_t i = 1; i < filled; i++) {            // insertion sort of the filled part
    uint16_t v = tmp[i]; int j = i - 1;
    while (j >= 0 && tmp[j] > v) { tmp[j + 1] = tmp[j]; j--; }
    tmp[j + 1] = v;
  }
  return tmp[filled / 2];
}

static uint16_t     g_ntc_f = 0;                   // last filtered value (for the status line)
static ThermalLevel thermal_update() {
  static ThermalLevel lvl = THERM_OK;
  static uint32_t t_cut = 0;
  uint16_t t = ntc_filtered();
  g_ntc_f = t;
#if NTC_PLAUSIBLE_MIN > 0
  if (t < NTC_PLAUSIBLE_MIN || t > NTC_PLAUSIBLE_MAX) return THERM_SENSOR;
#endif
  switch (lvl) {
    case THERM_OK:
      if      (t > NTC_ADC_PANIC)    { lvl = THERM_CUT; t_cut = millis(); }
      else if (t > NTC_ADC_THROTTLE)   lvl = THERM_THROTTLE;
      break;
    case THERM_THROTTLE:
      if      (t > NTC_ADC_PANIC)    { lvl = THERM_CUT; t_cut = millis(); }
      else if (t < NTC_ADC_THROTTLE_OFF) lvl = THERM_OK;
      break;
    case THERM_CUT:
      if (t < NTC_ADC_PANIC_OFF && millis() - t_cut >= THERMAL_COOLDOWN_MS) lvl = THERM_THROTTLE;
      break;
    default: break;
  }
  return lvl;
}

// ---------------------------------------------------------------------------
//  Fault latching: a fault opens the contactor and withdraws the
//  offer for at least FAULT_LATCH_MS. Then: EV unplugged (A) -> clear everything;
//  EV still connected (B) -> one retry, up to FAULT_MAX_RETRIES, after which we
//  wait for the EV to be unplugged. Welded contacts never clear:
//  the hardware must be inspected, a power cycle is required.
// ---------------------------------------------------------------------------
static bool     g_latched     = false;
static uint32_t g_fault_since = 0;
static uint8_t  g_fault_count = 0;
static ErrCode  g_err         = ERR_NONE;          // current / last fault cause

static void fault_enter(ErrCode code) {
  if (!g_latched) {
    g_latched = true; g_fault_since = millis();
    // ERR_SEQUENCE is not counted: the contactor cannot close while B has not
    // been seen, so retrying it forever is harmless (and lets an EV that was
    // still in C after a power cut resume as soon as it opens S2).
    if (code != ERR_SEQUENCE && g_fault_count < 255) g_fault_count++;
  }
  if (code == ERR_WELD || g_err != ERR_WELD) g_err = code;   // WELD is sticky
}
// Called while latched and the underlying condition has cleared: may release.
static void fault_maintain(CpState st) {
  if (g_err == ERR_WELD) return;                            // permanent until power cycle
  bool seq = (g_err == ERR_SEQUENCE);
  if (millis() - g_fault_since < (seq ? SEQ_RETRY_MS : FAULT_LATCH_MS)) return;
  if (st == CP_A) { g_latched = false; g_fault_count = 0; g_err = ERR_NONE; }
  else if (st == CP_B && (seq || g_fault_count < FAULT_MAX_RETRIES)) { g_latched = false; g_err = ERR_NONE; }
}

// ---------------------------------------------------------------------------
//  Button (presets) + modes + charge enable
// ---------------------------------------------------------------------------
enum Mode { MODE_MANUAL, MODE_AUTO };
static const uint8_t SEL_AUTO = 3;                 // button: 0..2 = PRESETS_A, 3 = AUTO
// Boot policy: start in AUTO. With no network order the existing fallback offers
// NET_FALLBACK_A (6 A) after NET_TIMEOUT_MS, so the station stays autonomous when
// the energy manager is unreachable - at the floor, not at 10 A. The energy manager
// should re-send MODE / ENABLE / SET periodically to take over quickly after a reset.
// MANUAL presets are only reached through the button or "MODE MAN".
static uint8_t  g_sel         = SEL_AUTO;          // selection (10/16/24/AUTO), button OR MQTT
static uint8_t  g_last_manual = 0;                 // last manual position (0..2)
static Mode     g_mode        = MODE_AUTO;         // derived from g_sel
static float    g_setpoint_a  = NET_FALLBACK_A;    // network setpoint (used in AUTO)
static uint32_t g_last_net_ms = 0;
static bool     g_enabled     = true;              // manual charge enable/stop

static void apply_sel() {
  if (g_sel < SEL_AUTO) { g_mode = MODE_MANUAL; g_last_manual = g_sel; }
  else                    g_mode = MODE_AUTO;
}

static void button_poll() {
  static bool     prev = true;                     // active low
  static uint32_t t_edge = 0;
  bool now = digitalRead(PIN_BUTTON);
  if (prev && !now && (millis() - t_edge > 200)) { // falling edge + debounce
    t_edge = millis();
    g_sel = (g_sel + 1) % 4;                        // 10 -> 16 -> 24 -> AUTO -> ...
    apply_sel();
  }
  prev = now;
}

// Network setpoint, with the fallback when the network is silent.
static float net_setpoint() {
  return (millis() - g_last_net_ms > NET_TIMEOUT_MS) ? NET_FALLBACK_A : g_setpoint_a;
}

// Target current, clamped: IEC floor, button-preset cap, hardware cap.
static float target_amps() {
  float a = (g_mode == MODE_AUTO) ? net_setpoint() : PRESETS_A[g_sel];  // g_sel is 0..2 in MANUAL
  if (a < EVSE_AMP_MIN) a = EVSE_AMP_MIN;
  if (a > EVSE_AMP_MAX) a = EVSE_AMP_MAX;
  return a;
}

// Should we be charging at all? Manual stop, or in AUTO a setpoint below the
// floor = "pause" for solar tracking (not enough surplus to offer >= 6 A). Uses
// the same fallback as target_amps(), so a dead link ends a pause too.
static bool charge_wanted() {
  if (!g_enabled) return false;
  if (g_mode == MODE_AUTO && net_setpoint() < EVSE_AMP_MIN) return false;
  return true;
}

// ---------------------------------------------------------------------------
//  Line reader shared by both builds: newline-terminated, '\r' ignored, a line
//  longer than the buffer is DISCARDED as a whole (a fragment must never be
//  interpreted as a command). Returns true when a complete line is in buf.
// ---------------------------------------------------------------------------
struct LineReader { char buf[32]; uint8_t n = 0; bool overflow = false; };
static bool read_line(Stream &in, LineReader &lr) {
  while (in.available()) {
    char c = (char)in.read();
    if (c == '\n') {
      bool ok = !lr.overflow;
      lr.buf[lr.n] = 0; lr.n = 0; lr.overflow = false;
      if (ok) return true;
    } else if (c >= 0x20 && c < 0x7F) {           // printable only: a bootloader sync (0x7F), the
      // bytes a rebooting ESP spits out or a bad frame must not prefix the next
      // command ("BOOTLOADER" right after a bootloader sync byte, for instance)
      if (lr.n < sizeof(lr.buf) - 1) lr.buf[lr.n++] = c; else lr.overflow = true;
    }
  }
  return false;
}
// Bounded float parsing: rejects garbage, nan/inf and anything outside [lo, hi].
static bool parse_amps(const char *s, float lo, float hi, float &out) {
  char *end = nullptr;
  float a = strtof(s, &end);
  if (end == s || !isfinite(a) || a < lo || a > hi) return false;
  out = a; return true;
}

// ---------------------------------------------------------------------------
//  Time of day, for the screen only (nothing in the charging logic uses it).
//  The board has no RTC battery and no network: the ESP bridge sends
//  "TIME HH:MM[:SS]" (local time, from its SNTP clock) every minute and
//  millis() carries it in between, for as long as the station runs (the
//  crystal drifts by seconds a day; a lost link already shows as a yellow
//  AUTO). Unknown (-1) only until the first TIME after boot.
// ---------------------------------------------------------------------------
static bool     g_clock_ok = false;
static uint32_t g_clock_s  = 0;            // seconds since midnight at g_clock_ms
static uint32_t g_clock_ms = 0;

static bool two_digits(const char *p, uint32_t &v) {
  if (p[0] < '0' || p[0] > '9' || p[1] < '0' || p[1] > '9') return false;
  v = (uint32_t)(p[0] - '0') * 10u + (uint32_t)(p[1] - '0');
  return true;
}
// "HH:MM" or "HH:MM:SS", 24-hour; anything else leaves the clock untouched.
static void clock_command(const char *s) {
  uint32_t h, m, sec = 0;
  if (!two_digits(s, h) || s[2] != ':' || !two_digits(s + 3, m)) return;
  if (s[5] == ':') { if (!two_digits(s + 6, sec) || s[8]) return; }
  else if (s[5]) return;
  if (h > 23 || m > 59 || sec > 59) return;
  g_clock_s = h * 3600u + m * 60u + sec; g_clock_ms = millis(); g_clock_ok = true;
}
// Minutes since midnight, -1 before the first TIME. The base moves forward by
// whole seconds on every call (every second), so millis() wrapping around
// (49.7 days) changes nothing, even if TIME stops for good.
static int16_t clock_minutes() {
  if (!g_clock_ok) return -1;
  uint32_t s = (millis() - g_clock_ms) / 1000u;
  g_clock_s = (g_clock_s + s) % 86400u; g_clock_ms += s * 1000u;
  return (int16_t)(g_clock_s / 60u);
}

// ---------------------------------------------------------------------------
//  House supply, for the screen only: the energy manager sends "INFO EDF 1"
//  while the house draws on EDF (the grid) and "INFO EDF 0" while it runs on
//  its own solar / battery supply, every minute and on change. Off EDF, the
//  plug icon takes turns with a sun. Unknown (-1) until the first line and
//  EDF_STALE_MS after the last one: no sun on old news.
//  Likewise "INFO SOC <0-100>", the vehicle's state of charge when the energy
//  manager knows it: the charging battery on the screen fills from there.
//  Forgotten SOC_STALE_MS after the last one.
// ---------------------------------------------------------------------------
static int8_t   g_edf = -1;
static uint32_t g_edf_ms = 0;
static int8_t   g_soc = -1;                // vehicle state of charge, %
static uint32_t g_soc_ms = 0;
static void info_command(const char *s) {
  if (!strcmp(s, "EDF 0") || !strcmp(s, "EDF 1")) { g_edf = (int8_t)(s[4] - '0'); g_edf_ms = millis(); }
  else if (!strncmp(s, "SOC ", 4)) {       // 1 to 3 digits, 0..100, nothing else
    const char *p = s + 4; int v = 0, n = 0;
    while (*p >= '0' && *p <= '9' && n < 4) { v = v * 10 + (*p++ - '0'); n++; }
    if (n >= 1 && n <= 3 && !*p && v <= 100) { g_soc = (int8_t)v; g_soc_ms = millis(); }
  }
}
static int8_t edf_now() {                  // latched to -1 once stale (safe across a millis() wrap)
  if (g_edf >= 0 && millis() - g_edf_ms > EDF_STALE_MS) g_edf = -1;
  return g_edf;
}
static int8_t soc_now() {                  // the same for the state of charge
  if (g_soc >= 0 && millis() - g_soc_ms > SOC_STALE_MS) g_soc = -1;
  return g_soc;
}

// ---------------------------------------------------------------------------
//  ESP bridge (UART, 115200 8N1, newline-terminated ASCII lines)
//    ESP  -> STM32 : "SET <amps>"             set the current setpoint (float, AUTO)
//                    "MODE AUTO" | "MODE MAN"  switch mode remotely (also on button)
//                    "PRESET 10|16|24|AUTO"    select a button position remotely:
//                                              10/16/24 = manual (fixed current), AUTO = SET
//                    "ENABLE 1"  | "ENABLE 0"  allow / stop charging (manual)
//                    "TIME HH:MM[:SS]"         time of day for the screen (every minute)
//                    "INFO EDF 1" | "INFO EDF 0"  house on EDF / off EDF, for the screen
//                                              (from the energy manager, every minute + on change)
//                    "INFO SOC <0-100>"        the vehicle's state of charge, for the screen
//    STM32 -> ESP  : "STATE=<A|B|C|F> ERR=<n> OFFER=<a> AMP=<a> VOLT=<v> TEMP=<t>
//                     NTC=<raw> ENERGY=<kWh> TMIN=<min> EN=<0|1> MODE=<AUTO|MAN>
//                     SEL=<10|16|24|AUTO>"  (SEL = the button position, also in state A)
//    STATE=F is reported for the whole latched period. ERR: 0 none, 1 CP -12 V
//    half not generated, 2 state D/E, 3 over-temperature, 4 timer, 5 over-current,
//    6 welded contacts, 7 ADC, 8 NTC sensor, 9 CP duty/polarity self-check,
//    10 C seen before B (sequence check).
//  Stopping a charge = withdraw the CP offer (CP back to +12 V) so the EV ramps
//  down, then open the contactor after a short delay (graceful, no break under
//  load). Triggered by "ENABLE 0" or, in AUTO, a setpoint below EVSE_AMP_MIN.
//  (The [env:diag] build streams raw ADC instead, for calibration.)
// ---------------------------------------------------------------------------
// ---------------------------------------------------------------------------
//  Firmware update without the SWD probe. "BOOTLOADER" (UART command) reboots
//  into the STM32 system bootloader (ROM, AN2606), which listens on USART1 =
//  PA9/PA10 = J12, i.e. on the ESP bridge (8E1, auto-baud on 0x7F):
//  tools/stm32_bridge_flash.py can then rewrite the firmware through the bridge.
//  Accepted only with no
//  vehicle (CP state A) and the contactor open (g_boot_ok, set by the loops).
//  The request survives the reset in backup register DR1; the jump happens at
//  the very start of setup(), BEFORE the watchdog is started (it cannot be
//  stopped once running and the ROM bootloader does not feed it), with the CP
//  held at +12 V and the contactor open (the bootloader leaves PA7/PB1 alone).
//  Leave the bootloader with the flasher's "go", a reset or a power cycle.
// ---------------------------------------------------------------------------
static const uint16_t BOOT_MAGIC = 0xB007;
static bool g_boot_ok = false;                     // no vehicle, contactor open

static void bootloader_request() {
  SERIAL_ESP.printf("BOOTLOADER: rebooting into the STM32 system bootloader (USART1, 8E1)\n");
  SERIAL_ESP.flush();
  delay(20);
#ifdef ARDUINO_ARCH_STM32
  __HAL_RCC_PWR_CLK_ENABLE(); __HAL_RCC_BKP_CLK_ENABLE();
  HAL_PWR_EnableBkUpAccess();
  BKP->DR1 = BOOT_MAGIC;
  HAL_PWR_DisableBkUpAccess();
  NVIC_SystemReset();
#endif
}

static void bootloader_check() {                   // first call of setup()
#ifdef ARDUINO_ARCH_STM32
  __HAL_RCC_PWR_CLK_ENABLE(); __HAL_RCC_BKP_CLK_ENABLE();
  if ((BKP->DR1 & 0xFFFFu) != BOOT_MAGIC) return;
  HAL_PWR_EnableBkUpAccess(); BKP->DR1 = 0; HAL_PWR_DisableBkUpAccess();   // one shot
  pinMode(PIN_CONTACTOR, OUTPUT); digitalWrite(PIN_CONTACTOR, CONTACTOR_OFF);
  pinMode(PIN_CP_GEN, OUTPUT);    digitalWrite(PIN_CP_GEN, CP_GEN_INVERTED ? LOW : HIGH);   // +12 V
  HAL_RCC_DeInit();                                // back to HSI, PLL off (the ROM expects reset clocks)
  SysTick->CTRL = 0; SysTick->LOAD = 0; SysTick->VAL = 0;
  __disable_irq();
  for (uint8_t i = 0; i < 8; i++) { NVIC->ICER[i] = 0xFFFFFFFFu; NVIC->ICPR[i] = 0xFFFFFFFFu; }
  __enable_irq();
  const uint32_t sysmem = 0x1FFFF000u;             // STM32F10x system memory
  __set_MSP(*(volatile uint32_t *)sysmem);
  ((void (*)(void))(*(volatile uint32_t *)(sysmem + 4)))();
  while (1) {}
#endif
}

// ---- Front LED bar (74HC595 on the LED board, wiring and colours in board.h) ----
// Same look as the original firmware as far as known: dark when idle, green when a
// vehicle is plugged, green filling down from the top while charging, red blinking
// on a fault. "LEDBAR <hex> [s] [red]" shows a raw pattern for s seconds (default
// 10, max 60): bit k = 595 output k = position k from the top; 0 = green, 1 = red
// if red = 1 (default) else dark. "LEDDEMO <amps> [s]" plays the charging animation
// as if the vehicle drew that current; "LEDSPEED <ms at 6 A> <ms at 24 A>" retunes
// its speed until the next reset.
static uint8_t  g_led_bits = 0;
static uint32_t g_led_ms_6a = LED_STEP_MS_6A, g_led_ms_24a = LED_STEP_MS_24A;
static float    g_led_amps = 0.0f;                    // shown (smoothed) current, from the 1 s status
static uint32_t g_led_step_t = 0;
static uint8_t  g_led_step_n = 0;
static bool     g_led_demo = false;
static uint32_t g_led_demo_t0 = 0, g_led_demo_ms = 0;
static float    g_led_demo_amps = 0.0f;
static bool     g_led_red = false;
static bool     g_led_test = false;
static uint32_t g_led_test_t0 = 0, g_led_test_ms = 0;
static uint8_t  g_led_test_bits = 0;
static bool     g_led_test_red = true;

static void led_bar_write(uint8_t bits, bool red) {
  if (bits != g_led_bits) {
    for (int8_t b = 7; b >= 0; b--) {                 // MSB first: bit 7 ends in QH, bit 0 in QA
      digitalWrite(PIN_LED_SER, ((bits >> b) & 1) ? HIGH : LOW);
      digitalWrite(PIN_LED_SCK, HIGH);
      digitalWrite(PIN_LED_SCK, LOW);
    }
    digitalWrite(PIN_LED_RCK, HIGH);                  // the outputs change only on the latch
    digitalWrite(PIN_LED_RCK, LOW);
    g_led_bits = bits;
  }
  if (red != g_led_red) { digitalWrite(PIN_LED_RED, red ? HIGH : LOW); g_led_red = red; }
}

static void led_bar_init() {                          // dark: all ones, red off
  pinMode(PIN_LED_RED, OUTPUT); digitalWrite(PIN_LED_RED, LOW);
  pinMode(PIN_LED_SER, OUTPUT); digitalWrite(PIN_LED_SER, LOW);
  pinMode(PIN_LED_SCK, OUTPUT); digitalWrite(PIN_LED_SCK, LOW);
  pinMode(PIN_LED_RCK, OUTPUT); digitalWrite(PIN_LED_RCK, LOW);
  g_led_bits = 0x00; g_led_red = false;               // forces the first shift
  led_bar_write(0xFF, false);
}

static uint8_t led_green_top(uint8_t n) {             // the n top positions green, the rest dark
  uint8_t bits = 0xFF;
  for (uint8_t k = 0; k < n && k < LED_POSITIONS; k++) bits &= (uint8_t)~(1u << k);
  return bits;
}

// Step period for a current: the step FREQUENCY is interpolated linearly between
// its values at 6 A and 24 A (clamped).
static uint32_t led_step_ms(float amps) {
  float a = amps < 6.0f ? 6.0f : amps > 24.0f ? 24.0f : amps;
  float f6 = 1000.0f / (float)g_led_ms_6a, f24 = 1000.0f / (float)g_led_ms_24a;
  float f = f6 + (f24 - f6) * (a - 6.0f) / 18.0f;
  return (uint32_t)(1000.0f / f);
}

static void led_bar_update(CpState st, bool charging, bool fault) {
  const uint32_t now = millis();
  if (g_led_test) {
    if (now - g_led_test_t0 < g_led_test_ms) { led_bar_write(g_led_test_bits, g_led_test_red); return; }
    g_led_test = false;
  }
  float amps = g_led_amps;
  if (g_led_demo) {
    if (now - g_led_demo_t0 < g_led_demo_ms) { charging = true; fault = false; amps = g_led_demo_amps; }
    else g_led_demo = false;
  }
  if (fault)                         led_bar_write(0xFF, (now / 500) & 1);    // red blinking
  else if (charging) {                                                        // green flowing down
    if (now - g_led_step_t >= led_step_ms(amps)) {
      g_led_step_t = now;
      g_led_step_n = (uint8_t)((g_led_step_n + 1) % (LED_POSITIONS + 1));
    }
    led_bar_write(led_green_top(g_led_step_n), false);
  }
  else if (st == CP_B || st == CP_C) led_bar_write(0x00, false);              // all green
  else                               led_bar_write(0xFF, false);              // dark
}

static void led_demo_command(const char *arg) {       // "LEDDEMO <amps> [s]"
  char *end;
  float a = strtof(arg, &end);
  if (end == arg || a < 0.0f || a > 80.0f) { SERIAL_ESP.printf("LEDDEMO <amps> [1-120 s]\n"); return; }
  long s = strtol(end, nullptr, 10);
  if (s < 1 || s > 120) s = 15;
  g_led_demo_amps = a; g_led_demo_ms = 1000UL * (uint32_t)s; g_led_demo_t0 = millis(); g_led_demo = true;
  SERIAL_ESP.printf("LEDDEMO %.1f A for %ld s: step %lu ms\n", (double)a, s, (unsigned long)led_step_ms(a));
}

static void led_speed_command(const char *arg) {      // "LEDSPEED <ms at 6 A> <ms at 24 A>"
  char *end;
  long m6 = strtol(arg, &end, 10), m24 = strtol(end, nullptr, 10);
  if (m6 < 30 || m6 > 3000 || m24 < 30 || m24 > 3000) {
    SERIAL_ESP.printf("LEDSPEED <ms at 6 A> <ms at 24 A> (30-3000), now %lu / %lu\n",
                      (unsigned long)g_led_ms_6a, (unsigned long)g_led_ms_24a);
    return;
  }
  g_led_ms_6a = (uint32_t)m6; g_led_ms_24a = (uint32_t)m24;
  SERIAL_ESP.printf("LEDSPEED %lu ms at 6 A, %lu ms at 24 A (until reset)\n",
                    (unsigned long)g_led_ms_6a, (unsigned long)g_led_ms_24a);
}

static void led_bar_command(const char *arg) {        // "LEDBAR <hex> [s] [red]"
  char *end, *end2;
  unsigned long v = strtoul(arg, &end, 16);
  if (end == arg || v > 0xFF) { SERIAL_ESP.printf("LEDBAR <00-FF> [1-60 s] [red 0/1]\n"); return; }
  long s = strtol(end, &end2, 10);
  if (s < 1 || s > 60) s = 10;
  long red = 1;
  if (end2 != end) {
    red = strtol(end2, &end, 10);
    if (end == end2 || red < 0 || red > 1) red = 1;
  }
  g_led_test_bits = (uint8_t)v; g_led_test_ms = 1000UL * (uint32_t)s; g_led_test_red = red != 0;
  g_led_test_t0 = millis(); g_led_test = true;
  SERIAL_ESP.printf("LEDBAR %02X for %ld s, red %s\n", (unsigned)v, s, red ? "on" : "off");
}

#ifdef LED_PROBE
// ---- Pin probe (test build: -D LED_PROBE, [env:ledprobe]) ----------------------
// Finds which free pin drives an unknown part of the board (written to find the
// LED bar). The wave / WS2812 / 595 commands work on port A's free pins (PA2 and
// PA7 are the mains sense and the CP); PINPULL on every free pin.
// Nothing is driven before PINSCAN has found the pin floating (it follows a weak
// pull both ways): a pin held by another part of the board is left alone.
//   PINSCAN                      pull each pin up, then down; report floating / held
//   PINWAVE <pin> <hz> <s>       square wave on a floating pin (1-20 Hz, 1-30 s)
//   PINWS <pin> <r> <g> <b> [n]  one WS2812-type frame (GRB, 800 kHz) for n LEDs
//   PIN595 <data> <clk> <latch> <hex> [n]
//                                shift n bytes (MSB first) like a 74HC595 / CD4094,
//                                then a latch pulse; the three pins stay driven
//   PINOFF                       every probe pin back to a floating input
//   PINPULL <pin|ALL> <u|d|z>    WEAK pull-up / pull-down / floating on a pin the firmware
//                                does not use, never driven (40 kOhm: harmless whatever
//                                sits on the other side). With a multimeter on the LED
//                                board's wires, the wire that follows belongs to that pin.
static const uint32_t PROBE_PIN[]  = { PA0, PA1, PA3, PA4, PA5, PA6 };
static const char    *PROBE_NAME[] = { "PA0", "PA1", "PA3", "PA4", "PA5", "PA6" };
static const int      PROBE_N = 6;
static bool     probe_floating[PROBE_N] = {false};
static int      wave_k = -1;
static uint32_t wave_half_ms = 0, wave_end = 0, wave_t = 0;
static bool     wave_high = false;

static int probe_find(const char *name) {
  for (int k = 0; k < PROBE_N; k++) if (!strcmp(name, PROBE_NAME[k])) return k;
  return -1;
}
static void probe_release(int k) { pinMode(PROBE_PIN[k], INPUT); }   // floating, as at reset
static void probe_off() {
  for (int k = 0; k < PROBE_N; k++) probe_release(k);
  wave_k = -1;
}

static void probe_scan() {
  probe_off();
  for (int k = 0; k < PROBE_N; k++) {
    pinMode(PROBE_PIN[k], INPUT_PULLUP);   delayMicroseconds(500); int up = digitalRead(PROBE_PIN[k]);
    pinMode(PROBE_PIN[k], INPUT_PULLDOWN); delayMicroseconds(500); int dn = digitalRead(PROBE_PIN[k]);
    probe_release(k);
    probe_floating[k] = (up == 1 && dn == 0);
    SERIAL_ESP.printf("PINSCAN %s pu=%d pd=%d %s\n", PROBE_NAME[k], up, dn,
                      probe_floating[k] ? "floating" : up ? "held high" : "held low");
  }
}

// Square wave, advanced from loop(): the loop period (~45 ms) sets the resolution.
static void probe_tick() {
  if (wave_k < 0) return;
  if ((int32_t)(millis() - wave_end) >= 0) {
    probe_release(wave_k); wave_k = -1;
    SERIAL_ESP.printf("PINWAVE done\n");
    return;
  }
  if (millis() - wave_t >= wave_half_ms) {
    wave_t = millis(); wave_high = !wave_high;
    digitalWrite(PROBE_PIN[wave_k], wave_high ? HIGH : LOW);
  }
}

// One frame for WS2812-type LEDs: 1.25 us per bit, '0' = 0.4 us high, '1' = 0.8 us,
// GRB order, MSB first, timed on the cycle counter (72 MHz) with interrupts off
// (~0.5 ms for 16 LEDs). The pin then stays low: a floating data line latches noise.
static void probe_ws2812(int k, uint8_t r, uint8_t g, uint8_t b, int n) {
  GPIO_TypeDef *port = digitalPinToPort(PROBE_PIN[k]);
  uint32_t mask = digitalPinToBitMask(PROBE_PIN[k]);
  pinMode(PROBE_PIN[k], OUTPUT); digitalWrite(PROBE_PIN[k], LOW);
  delayMicroseconds(300);                                   // reset (> 280 us for the B version)
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  const uint32_t T_BIT = 90, T_0H = 29, T_1H = 58;          // 1.25 / 0.40 / 0.80 us at 72 MHz
  const uint32_t grb = ((uint32_t)g << 16) | ((uint32_t)r << 8) | b;
  __disable_irq();
  uint32_t t = DWT->CYCCNT;
  for (int led = 0; led < n; led++)
    for (int bit = 23; bit >= 0; bit--) {
      uint32_t th = ((grb >> bit) & 1) ? T_1H : T_0H;
      while (DWT->CYCCNT - t < T_BIT) {}
      t = DWT->CYCCNT;
      port->BSRR = mask;
      while (DWT->CYCCNT - t < th) {}
      port->BRR = mask;
    }
  __enable_irq();
  delayMicroseconds(300);                                   // latch
}

// Three-wire shift register (data, clock, latch), slow edges (5 us). The latch
// pulse ends low, so a pin that is in fact an active-low output enable leaves the
// outputs on.
static void probe_595(int kd, int kc, int kl, uint8_t v, int n) {
  const int ks[3] = { kd, kc, kl };
  for (int k : ks) { pinMode(PROBE_PIN[k], OUTPUT); digitalWrite(PROBE_PIN[k], LOW); }
  if (wave_k == kd || wave_k == kc || wave_k == kl) wave_k = -1;
  for (int byte = 0; byte < n; byte++)
    for (int bit = 7; bit >= 0; bit--) {
      digitalWrite(PROBE_PIN[kd], ((v >> bit) & 1) ? HIGH : LOW); delayMicroseconds(5);
      digitalWrite(PROBE_PIN[kc], HIGH); delayMicroseconds(5);
      digitalWrite(PROBE_PIN[kc], LOW);  delayMicroseconds(5);
    }
  digitalWrite(PROBE_PIN[kl], HIGH); delayMicroseconds(5);
  digitalWrite(PROBE_PIN[kl], LOW);
}

// Every pin the firmware leaves free (board.h, display.cpp; UART, SWD and oscillators
// excluded), PE2-PE5 included: this test build does not drive the LED bar.
struct PullPin { const char *name; uint32_t pin; };
static const PullPin PULL_PINS[] = {
  {"PA0", PA0}, {"PA1", PA1}, {"PA3", PA3}, {"PA4", PA4}, {"PA5", PA5}, {"PA6", PA6},
  {"PA8", PA8}, {"PA11", PA11}, {"PA12", PA12}, {"PA15", PA15},
  {"PB0", PB0}, {"PB2", PB2}, {"PB3", PB3}, {"PB4", PB4}, {"PB5", PB5}, {"PB6", PB6},
  {"PB7", PB7}, {"PB9", PB9}, {"PB10", PB10}, {"PB11", PB11}, {"PB12", PB12},
  {"PB13", PB13}, {"PB14", PB14}, {"PB15", PB15},
  {"PC4", PC4}, {"PC6", PC6}, {"PC7", PC7}, {"PC8", PC8}, {"PC9", PC9}, {"PC10", PC10},
  {"PC11", PC11}, {"PC12", PC12}, {"PC13", PC13},
  {"PD2", PD2}, {"PD3", PD3}, {"PD6", PD6}, {"PD8", PD8}, {"PD9", PD9}, {"PD10", PD10},
  {"PD12", PD12},
  {"PE0", PE0}, {"PE1", PE1}, {"PE2", PE2}, {"PE3", PE3}, {"PE4", PE4}, {"PE5", PE5},
  {"PE6", PE6}, {"PE11", PE11}, {"PE12", PE12}, {"PE13", PE13}, {"PE15", PE15},
};

static void probe_pull(const char *arg) {
  char name[8] = ""; char m = 0;
  if (sscanf(arg, "%7s %c", name, &m) < 2 || (m != 'u' && m != 'd' && m != 'z')) {
    SERIAL_ESP.printf("PINPULL <pin|ALL> <u|d|z>\n");
    return;
  }
  const uint32_t mode = (m == 'u') ? INPUT_PULLUP : (m == 'd') ? INPUT_PULLDOWN : INPUT;
  const char *how = (m == 'u') ? "up" : (m == 'd') ? "down" : "floating";
  bool all = !strcmp(name, "ALL");
  for (const PullPin &p : PULL_PINS) {
    if (!all && strcmp(name, p.name)) continue;
    if (wave_k >= 0 && PROBE_PIN[wave_k] == p.pin) wave_k = -1;   // stop a square wave on it
    pinMode(p.pin, mode);
    if (!all) { SERIAL_ESP.printf("PINPULL %s %s\n", p.name, how); return; }
  }
  if (all) SERIAL_ESP.printf("PINPULL ALL %s\n", how);
  else     SERIAL_ESP.printf("PINPULL refused: %s is not a free pin\n", name);
}

static void probe_command(const char *buf) {
  if (!strcmp(buf, "PINSCAN")) { probe_scan(); return; }
  if (!strncmp(buf, "PINPULL ", 8)) { probe_pull(buf + 8); return; }
  if (!strcmp(buf, "PINOFF"))  { probe_off(); SERIAL_ESP.printf("PINOFF done\n"); return; }
  if (!strncmp(buf, "PIN595 ", 7)) {
    char d[8] = "", c[8] = "", l[8] = ""; unsigned v = 0; int n = 2;
    int got = sscanf(buf + 7, "%7s %7s %7s %x %d", d, c, l, &v, &n);
    int kd = probe_find(d), kc = probe_find(c), kl = probe_find(l);
    if (got < 4 || kd < 0 || kc < 0 || kl < 0 || kd == kc || kd == kl || kc == kl
        || !probe_floating[kd] || !probe_floating[kc] || !probe_floating[kl]) {
      SERIAL_ESP.printf("PROBE refused: PIN595 <data> <clk> <latch> <hex> [n], three distinct floating pins\n");
      return;
    }
    if (n < 1 || n > 8) n = 2;
    probe_595(kd, kc, kl, (uint8_t)v, n);
    SERIAL_ESP.printf("PIN595 data=%s clk=%s latch=%s %02X x%d sent\n", d, c, l, v & 0xFF, n);
    return;
  }
  bool wave = !strncmp(buf, "PINWAVE ", 8), ws = !strncmp(buf, "PINWS ", 6);
  if (!wave && !ws) return;
  char name[8] = ""; int a = 0, b = 0, c = 0, n = 16;
  int got = sscanf(buf + (wave ? 8 : 6), "%7s %d %d %d %d", name, &a, &b, &c, &n);
  int k = probe_find(name);
  if (k < 0 || !probe_floating[k]) {
    SERIAL_ESP.printf("PROBE refused: %s is not a floating probe pin (PINSCAN first)\n", name);
    return;
  }
  if (wave) {
    if (got < 3 || a < 1 || a > 20 || b < 1 || b > 30) { SERIAL_ESP.printf("PINWAVE <pin> <1-20 Hz> <1-30 s>\n"); return; }
    if (wave_k >= 0) probe_release(wave_k);
    pinMode(PROBE_PIN[k], OUTPUT); digitalWrite(PROBE_PIN[k], LOW);
    wave_k = k; wave_high = false; wave_t = millis();
    wave_half_ms = 500U / (uint32_t)a; wave_end = millis() + 1000U * (uint32_t)b;
    SERIAL_ESP.printf("PINWAVE %s %d Hz %d s\n", PROBE_NAME[k], a, b);
  } else {
    if (got < 4) { SERIAL_ESP.printf("PINWS <pin> <r> <g> <b> [n]\n"); return; }
    if (n < 1 || n > 64) n = 16;
    if (wave_k == k) wave_k = -1;
    probe_ws2812(k, (uint8_t)a, (uint8_t)b, (uint8_t)c, n);
    SERIAL_ESP.printf("PINWS %s %d %d %d n=%d sent\n", PROBE_NAME[k], a, b, c, n);
  }
}
#endif

static void esp_poll() {
  static LineReader lr;
  while (read_line(SERIAL_ESP, lr)) {
    const char *buf = lr.buf;
#ifdef LED_PROBE
    if (!strncmp(buf, "PIN", 3)) { probe_command(buf); continue; }
#endif
    if (!strncmp(buf, "SET ", 4)) {
      float a;                                       // an invalid line does NOT refresh the watchdog
      if (parse_amps(buf + 4, 0.0f, 80.0f, a)) { g_setpoint_a = a; g_last_net_ms = millis(); }
    }
    else if (!strcmp(buf, "MODE AUTO")) { g_sel = SEL_AUTO;     apply_sel(); }
    else if (!strcmp(buf, "MODE MAN"))  { g_sel = g_last_manual; apply_sel(); }
    else if (!strncmp(buf, "PRESET ", 7)) {          // same effect as pressing the button there
      const char *v = buf + 7;
      if (!strcmp(v, "AUTO")) { g_sel = SEL_AUTO; apply_sel(); }
      else for (uint8_t k = 0; k < SEL_AUTO; k++) {   // exact "10" / "16" / "24", nothing else
        char t[4]; snprintf(t, sizeof t, "%u", (unsigned)PRESETS_A[k]);
        if (!strcmp(v, t)) { g_sel = k; apply_sel(); }
      }
    }
    else if (!strcmp(buf, "ENABLE 1"))  g_enabled = true;
    else if (!strcmp(buf, "ENABLE 0"))  g_enabled = false;
    else if (!strncmp(buf, "TIME ", 5)) clock_command(buf + 5);
    else if (!strncmp(buf, "INFO ", 5)) info_command(buf + 5);
    else if (!strcmp(buf, "BOOTLOADER")) {
      if (g_boot_ok) bootloader_request();
      else SERIAL_ESP.printf("BOOTLOADER refused: unplug the vehicle first (state A, contactor open)\n");
    }
    else if (!strcmp(buf, "VERSION")) cp_report_boot(SERIAL_ESP);   // BOOT line again (build date)
    else if (!strncmp(buf, "LEDBAR ", 7))   led_bar_command(buf + 7);
    else if (!strncmp(buf, "LEDDEMO ", 8))  led_demo_command(buf + 8);
    else if (!strncmp(buf, "LEDSPEED", 8))  led_speed_command(buf + 8);
  }
}
static void esp_status(CpState st, float v, float i) {
  const char *S = (st == CP_A) ? "A" : (st == CP_B) ? "B" : (st == CP_C) ? "C" : "F";
  // Offered current = applied CP duty (standby = 100% duty -> no real offer -> 0).
  float offered = (cp_compare_ticks < CP_ARR) ? DUTY_TO_AMP(cp_compare_ticks / 10.0f) : 0.0f;
  static const char *const SEL_TXT[] = { "10", "16", "24", "AUTO" };
  SERIAL_ESP.printf("STATE=%s ERR=%u OFFER=%.1f AMP=%.1f VOLT=%.0f TEMP=%.0f NTC=%u ENERGY=%.3f TMIN=%lu EN=%d MODE=%s SEL=%s\n",
                    S, (unsigned)g_err, offered, i, v, tempC_from(g_ntc_f), g_ntc_f, session_kwh(),
                    (unsigned long)session_min(), g_enabled ? 1 : 0,
                    g_mode == MODE_AUTO ? "AUTO" : "MAN", SEL_TXT[g_sel < 4 ? g_sel : 3]);
}

// ===========================================================================
//  setup / loop
// ===========================================================================
#ifdef DIAG_MODE
// ---------------------------------------------------------------------------
//  CALIBRATION / DIAGNOSTIC build (-D DIAG_MODE, [env:diag]).
//  Streams the raw CP/ADC readings so the thresholds can be measured.
//  Starts in STANDBY (+12 V, no PWM) so the CP polarity can be checked at rest.
//  Commands:
//    "OFFER <6..16>"  post the PWM for that current ; "OFFER 0" back to standby
//    "CHARGE ON/OFF"  allow the contactor (see interlock) / forbid it
//    "SCAN"           non-blocking ADC sweep, one pin per loop iteration, while
//                     everything else keeps running (also under load) -> finds
//                     the CT channel ; never PB1 (the contactor)
//    "CTPIN <name>"   pin measured in the ct= field (mean / AC-RMS; default PIN_CT)
//    "CAP <pin> <us>" 400 samples of one input into g_cap[] (read back over SWD)
//  Interlock: the contactor closes ONLY while all of these hold for
//  DELAY_STATE_MS: PWM active, plateau in the C window, -12 V half generated,
//  B seen before C, measured duty plausible, no over-temperature, timer sane. It opens at once
//  otherwise, and CHARGE ON is cleared when the EV leaves (A) or on over-temp.
//  WARNING: "CHARGE ON" energises the connector - only with a properly plugged EV.
//  Until the CP boundaries are calibrated the interlock may never be satisfied:
//  read cp_hi / cp_lo, set the boundaries in board.h, re-flash, then CHARGE ON.
// ---------------------------------------------------------------------------
static bool  diag_charge  = false;
static float diag_offer_a = 0.0f;            // 0 = standby
static int   diag_scan_idx = -1;             // -1 = idle
// PA7 (the CP drive) and PB1 (the contactor) are outputs and must never be
// sampled: an analog input there would leave them floating.
static const struct { const char *name; uint32_t pin; } DIAG_CH[] = {
  {"PA0", PA0}, {"PA1", PA1}, {"PA2", PA2}, {"PA3", PA3}, {"PA4", PA4}, {"PA5", PA5},
  {"PA6", PA6}, {"PB0", PB0}, {"PC0", PC0}, {"PC1", PC1}, {"PC2", PC2},
  {"PC3", PC3}, {"PC4", PC4}, {"PC5", PC5} };
static const int DIAG_NCH = (int)(sizeof(DIAG_CH) / sizeof(DIAG_CH[0]));
static int diag_index_of(uint32_t pin) {
  for (int k = 0; k < DIAG_NCH; k++) if (DIAG_CH[k].pin == pin) return k;
  return DIAG_NCH - 1;
}
static int diag_ct_idx = diag_index_of(PIN_CT);  // PIN_CT by default; "CTPIN <pin>" to change

// "CAP <pin> <interval_us>": 400 samples of one input at a fixed pace into
// g_cap[], read back over SWD (mdh &g_cap 400). Blocking: at most 400 x 3 ms,
// well inside the 2 s watchdog; the CP keeps running (hardware PWM).
volatile uint16_t g_cap[400];                 // kept in RAM for the SWD read-back
static void diag_capture(const char *args) {
  char name[8] = {0}; unsigned iv = 0;
  if (sscanf(args, "%7s %u", name, &iv) < 1 || iv > 3000) return;
  for (int k = 0; k < DIAG_NCH; k++) {
    if (strcmp(name, DIAG_CH[k].name)) continue;
    uint32_t pin = DIAG_CH[k].pin;
    (void)adc_read(pin);
    uint32_t t = micros(), t0 = t;
    for (uint16_t i = 0; i < 400; i++) {
      g_cap[i] = adc_read(pin);
      if (iv) { t += iv; while ((int32_t)(micros() - t) < 0) {} }
    }
    SERIAL_ESP.printf("CAP %s iv=%u total_us=%lu\n", name, iv, (unsigned long)(micros() - t0));
  }
}

// ---------------------------------------------------------------------------
//  Panel in the diag build: "UI DIAG" (raw text, default), "UI LIVE" (the
//  operating UI fed with the live readings) or "UI DEMO" (every UI screen of
//  the simulator in turn, 4 s each) - to check the UI on the real panel while
//  the diag interlock stays in charge of the contactor.
// ---------------------------------------------------------------------------
enum DiagUi : uint8_t { UI_TEXT, UI_LIVE, UI_DEMO };
static DiagUi diag_ui = UI_TEXT;
static bool   diag_ui_dirty = false;           // mode changed: clear the panel first
static const uint8_t UI_DEMO_N = 10;

static DisplayData ui_base() {
  DisplayData d = {};
  d.target_a = 6; d.volts = 230.0f; d.tempC = 22.0f; d.sel = 3;
  d.mode_auto = true; d.link_ok = true; d.enabled = true;
  d.clock_min = clock_minutes();                                       // the real clock once TIME arrives
  d.on_edf = edf_now();                                                // and the house supply once known
  d.soc = soc_now();
  return d;
}
// Same screens as tools/sim/sim_ui.cpp (docs/ui-preview.png), plus fault 10.
static DisplayData ui_demo_screen(uint8_t k) {
  DisplayData d = ui_base();
  switch (k) {
    case 0: break;                                                     // READY, plug animation
    case 1: d.state = 1; d.offered_a = 16; d.target_a = 16; d.sel = 1; d.mode_auto = false; d.elapsed_s = 12; break;
    case 2: d.state = 2; d.contactor = true; d.offered_a = 11; d.target_a = 11; d.amps = 10.8f;
            d.ct_ready = true; d.kwh = 3.42f; d.elapsed_s = 3725; break; // CHARGING (AUTO)
    case 3: d.state = 2; d.contactor = true; d.offered_a = 16; d.target_a = 16; d.sel = 1; d.mode_auto = false;
            d.kwh = 0.12f; d.elapsed_s = 95; break;                    // CHARGING (manual 16 A)
    case 4: d.state = 1; d.elapsed_s = 600; break;                     // PAUSED (solar)
    case 5: d.state = 1; d.link_ok = false; break;                     // NO LINK
    case 6: d.state = 3; d.err = 1; d.target_a = 11; break;            // E1
    case 7: d.state = 3; d.err = 3; d.tempC = 71.2f; break;            // E3 over-temperature
    case 8: d.state = 3; d.err = 10; break;                            // E10 C before B
    default: d.enabled = false; break;                                 // DISABLED
  }
  return d;
}
// Front-panel button in the UI modes, display only (the offer stays under the
// OFFER command): LIVE cycles the preset highlight 10 -> 16 -> 24 -> AUTO as the
// evse build does, DEMO jumps to the next screen. Same edge + 200 ms debounce
// as button_poll().
static bool diag_button_pressed() {
  static bool     prev = true;                 // active low
  static uint32_t t_edge = 0;
  bool now = digitalRead(PIN_BUTTON), hit = false;
  if (prev && !now && (millis() - t_edge > 200)) { t_edge = millis(); hit = true; }
  prev = now;
  return hit;
}
static void diag_ui_step(const CpSample &s, float ct_mean, uint16_t vs, uint16_t ntc, bool close, bool offer) {
  static uint32_t t_show = 0, t_demo = 0, t_conn = 0;
  static uint8_t  k = 0, ui_sel = 3;
  uint32_t now = millis();
  bool pressed = diag_button_pressed();
  if (diag_ui_dirty) { display_clear(); diag_ui_dirty = false; t_show = now - 1000; t_demo = now; k = 0; }
  if (diag_ui == UI_DEMO) {
    if (pressed || now - t_demo >= 4000) { t_demo = now; k = (uint8_t)((k + 1) % UI_DEMO_N); t_show = now - 1000; }
    if (now - t_show >= 1000) { t_show = now; display_show(ui_demo_screen(k)); }
  } else if (pressed || now - t_show >= 1000) {                        // UI_LIVE
    if (pressed) ui_sel = (uint8_t)((ui_sel + 1) % 4);
    t_show = now;
    DisplayData d = ui_base();
    d.state = (s.pos >= CP_POS_AB) ? 0 : (s.pos >= CP_POS_BC) ? 1 : (s.pos >= CP_POS_CD) ? 2 : 3;
    if (d.state == 3) d.err = ERR_STATE_DE;
    if (d.state == 0) t_conn = now;
    d.elapsed_s = (now - t_conn) / 1000;
    d.offered_a = offer ? diag_offer_a : 0.0f;
    d.target_a  = offer ? diag_offer_a : 6.0f;
    d.ct_ready  = (DIAG_CH[diag_ct_idx].pin == PIN_CT);              // provisional scale (board.h)
    d.amps      = d.ct_ready ? fmaxf(0.0f, (ct_mean - CT_ZERO_COUNT) * CT_A_PER_COUNT) : 0.0f;
    d.volts     = vs * V_PER_COUNT;
    d.tempC     = tempC_from(ntc);
    d.contactor = close;
    d.sel = ui_sel; d.mode_auto = (ui_sel == 3);                       // button: display only
    if (ui_sel < 3) d.target_a = PRESETS_A[ui_sel];
    display_show(d);
  }
  display_tick();
}

static void diag_poll() {
  static LineReader lr;
  while (read_line(SERIAL_ESP, lr)) {
    const char *buf = lr.buf;
    if      (!strcmp(buf, "CHARGE ON"))  diag_charge = true;
    else if (!strcmp(buf, "CHARGE OFF")) diag_charge = false;
    else if (!strcmp(buf, "SCAN"))       { if (diag_scan_idx < 0) diag_scan_idx = 0; }
    else if (!strncmp(buf, "OFFER ", 6)) {
      float a;
      if (!strcmp(buf + 6, "0"))                       diag_offer_a = 0.0f;
      else if (parse_amps(buf + 6, 6.0f, 16.0f, a))    diag_offer_a = a;   // bounded, NaN-proof
    }
    else if (!strncmp(buf, "CTPIN ", 6)) {
      for (int k = 0; k < DIAG_NCH; k++) if (!strcmp(buf + 6, DIAG_CH[k].name)) diag_ct_idx = k;
    }
    else if (!strncmp(buf, "CAP ", 4)) diag_capture(buf + 4);
    else if (!strncmp(buf, "TIME ", 5)) clock_command(buf + 5);       // UI LIVE / DEMO screens
    else if (!strncmp(buf, "INFO ", 5)) info_command(buf + 5);
    else if (!strcmp(buf, "UI DIAG")) { diag_ui = UI_TEXT; diag_ui_dirty = true; }
    else if (!strcmp(buf, "UI LIVE")) { diag_ui = UI_LIVE; diag_ui_dirty = true; }
    else if (!strcmp(buf, "UI DEMO")) { diag_ui = UI_DEMO; diag_ui_dirty = true; }
    else if (!strcmp(buf, "BOOTLOADER")) {
      if (g_boot_ok) bootloader_request();
      else SERIAL_ESP.printf("BOOTLOADER refused: unplug the vehicle and CHARGE OFF first\n");
    }
    else if (!strcmp(buf, "VERSION")) cp_report_boot(SERIAL_ESP);
  }
}

// One pin of the sweep per call (100 ms): min / max / mean / AC-RMS in counts.
static void diag_scan_step() {
  if (diag_scan_idx < 0) return;
  if (diag_scan_idx >= DIAG_NCH) { SERIAL_ESP.printf("SCAN done\n"); diag_scan_idx = -1; return; }
  uint32_t pin = DIAG_CH[diag_scan_idx].pin;
  uint16_t mn = 4095, mx = 0; uint32_t n = 0; double sum = 0, sumsq = 0;
  uint32_t t0 = micros();
  do {
    uint16_t v = adc_read(pin);
    if (v < mn) mn = v;
    if (v > mx) mx = v;
    sum += v; sumsq += (double)v * v; n++;
  } while ((uint32_t)(micros() - t0) < 100000);
  double mean = sum / n, rms_ac = sqrt(fmax(0.0, sumsq / n - mean * mean));
  SERIAL_ESP.printf("SCAN %s n=%lu min=%u max=%u mean=%.0f rms_ac=%.1f\n",
                    DIAG_CH[diag_scan_idx].name, (unsigned long)n, mn, mx, mean, rms_ac);
  diag_scan_idx++;
}

void setup() {
  bootloader_check();                                 // "BOOTLOADER" requested before the reset?
  pinMode(PIN_CONTACTOR, OUTPUT); contactor(false);   // OPEN first
  pinMode(PIN_BUTTON, INPUT_PULLUP);
  boot_reset_cause();
  cp_init();                                          // drive the CP (+12 V) before anything slow
  cp_set_duty(CP_DUTY_STANDBY);                       // standby until "OFFER <a>"
  analogReadResolution(12);                           // (host simulation only; the board uses adc_read)
  adc_init();
  SERIAL_ESP.begin(SERIAL_ESP_BAUD);
  cp_report_boot(SERIAL_ESP);
  display_init();                                     // raw values also on the screen
#ifndef LED_PROBE                                     // (the probe build leaves those pins free)
  led_bar_init();                                     // bar dark, its inputs no longer floating
#endif
  IWatchdog.begin(2000000);
}

void loop() {
  diag_poll();
  bool offer = g_timer_ok && diag_offer_a >= 6.0f;
  cp_set_duty(offer ? AMP_TO_DUTY(diag_offer_a) : CP_DUTY_STANDBY);
  CpSample s = cp_burst();                            // pos = high plateau, neg = negative half depth
  cp_track_sequence(s);                               // B seen before C (see there)
  g_boot_ok = (s.pos >= CP_POS_AB) && !g_contactor_closed && !diag_charge;
  uint16_t ntc = ntc_filtered();

  bool plateau_c = (s.pos < CP_POS_BC && s.pos >= CP_POS_CD);
  bool diode_ok  = (s.neg <= CP_NEG_MAX);
  bool duty_ok   = cp_pwm_active() && cp_duty_plausible(s);
  bool hot       = (ntc > NTC_ADC_PANIC);
  bool ready_now = g_timer_ok && s.n >= CP_BURST_MIN_N && plateau_c && diode_ok && duty_ok && g_seen_b && !hot;
  static uint32_t t_ready = 0;
  if (!ready_now) t_ready = 0; else if (!t_ready) t_ready = millis();
  bool ready = ready_now && t_ready && (millis() - t_ready >= DELAY_STATE_MS);
  if (hot || s.pos >= CP_POS_AB) diag_charge = false; // EV gone / too hot -> CHARGE OFF
  bool close = diag_charge && ready;
  contactor(close);

  diag_scan_step();                                   // at most 100 ms, interlock already applied
  float ct_mean = mean_counts(DIAG_CH[diag_ct_idx].pin);  // the CT stage is DC (mean follows the current)
  float ct_rms  = rms_counts(DIAG_CH[diag_ct_idx].pin);   // AC residue, for information

  uint16_t vs = vsense_raw();
  SERIAL_ESP.printf("DIAG cp_pos=%u cp_neg=%u n=%u duty=%u.%u tok=%d adc=%lu ct=%s:%.0f/%.1f v=%u ntc=%u chg=%d ready=%d offer=%.1f btn=%d adc_to=%lu\n",
                    s.pos, s.neg, s.n, s.duty / 10, s.duty % 10, g_timer_ok ? 1 : 0,
                    (unsigned long)g_adc_us, DIAG_CH[diag_ct_idx].name, ct_mean, ct_rms, vs, ntc,
                    close ? 1 : 0, ready ? 1 : 0, offer ? diag_offer_a : 0.0f, digitalRead(PIN_BUTTON),
                    (unsigned long)g_adc_timeouts);
  // Same values on the panel (16 characters per line), useful while the UART is not wired.
  char l[8][20]; const char *lp[8];
  snprintf(l[0], 17, "DIAG tok%d adc%lu", g_timer_ok ? 1 : 0, (unsigned long)g_adc_us);
  snprintf(l[1], 17, "CP+%4u CP-%4u", s.pos, s.neg);
  snprintf(l[2], 17, "n%3u duty %u.%u", s.n, s.duty / 10, s.duty % 10);
  snprintf(l[3], 17, "CT %s %.0f", DIAG_CH[diag_ct_idx].name, ct_mean);
  snprintf(l[4], 17, "V%4u NTC%4u", vs, ntc);
  snprintf(l[5], 17, "offer %.1fA", offer ? diag_offer_a : 0.0f);
  snprintf(l[6], 17, "chg%d rdy%d btn%d", close ? 1 : 0, ready ? 1 : 0, digitalRead(PIN_BUTTON));
  snprintf(l[7], 17, "C%d D%d P%d S%d", plateau_c ? 1 : 0, diode_ok ? 1 : 0, duty_ok ? 1 : 0, g_seen_b ? 1 : 0);
  for (int k = 0; k < 8; k++) lp[k] = l[k];
  if (diag_ui == UI_TEXT) {
    if (diag_ui_dirty) { display_clear(); diag_ui_dirty = false; }
    display_diag(lp, 8);
  } else {
    diag_ui_step(s, ct_mean, vs, ntc, close, offer);
  }
  delay(100);
  IWatchdog.reload();
}
#else
void setup() {
  bootloader_check();                             // "BOOTLOADER" requested before the reset?
  // --- safe states BEFORE anything else ---
  pinMode(PIN_CONTACTOR, OUTPUT);
  contactor(false);                               // contactor OPEN at boot
  pinMode(PIN_BUTTON, INPUT_PULLUP);
  boot_reset_cause();
  if (g_boot_after_iwdg) { g_had_open = true; g_last_open_ms = millis(); }   // hold the first closing

  cp_init();                                      // CP driven (+12 V, state A) before anything slow
  cp_set_duty(CP_DUTY_STANDBY);
  analogReadResolution(12);                       // (host simulation only; the board uses adc_read)
  adc_init();

  SERIAL_ESP.begin(SERIAL_ESP_BAUD);
  cp_report_boot(SERIAL_ESP);
  display_init();
#ifndef LED_PROBE                                 // (the probe build leaves those pins free)
  led_bar_init();                                 // bar dark, its inputs no longer floating
#endif

  IWatchdog.begin(2000000);                       // 2 s watchdog -> reset on hang
}

void loop() {
  button_poll();
  esp_poll();
#ifdef LED_PROBE
  probe_tick();
#endif

  CpState st = cp_state_debounced();
  g_boot_ok = (st == CP_A) && !g_contactor_closed;
  if (!g_timer_ok) { st = CP_FAULT; g_cp_err = ERR_TIMER; }   // timer guard
  ThermalLevel th = thermal_update();
  float amps = target_amps();
  if (th == THERM_THROTTLE) amps = EVSE_AMP_MIN;  // throttle to the floor first

  // New charging session when a vehicle is newly plugged (A -> B/C).
  static CpState prev_st = CP_A;
  if (prev_st == CP_A && st != CP_A) meter_reset();
  prev_st = st;

  // --- faults -> latched safe state ---
  if (st == CP_FAULT)        fault_enter(g_cp_err);
  if (th == THERM_CUT)       fault_enter(ERR_OVERTEMP);
  if (th == THERM_SENSOR)    fault_enter(ERR_NTC);

  static uint32_t pause_ms = 0;                   // graceful-open timer
  bool want = false;

  if (g_latched) {
    contactor(false);
    cp_set_duty(CP_DUTY_STANDBY);                 // withdraw the offer
    pause_ms = 0;
    if (st != CP_FAULT && th != THERM_CUT && th != THERM_SENSOR) fault_maintain(st);
  } else {
    // --- charge wanted? with anti short-cycling ---
    bool want_raw = charge_wanted();
    want = want_raw;
    // a solar pause (setpoint < 6 A) is honoured only MIN_CHARGE_MS after closing;
    // "ENABLE 0" (g_enabled false) is always immediate.
    if (g_contactor_closed && !want_raw && g_enabled &&
        (millis() - g_t_contactor) < MIN_CHARGE_MS) want = true;

    // --- state machine ---
    switch (st) {
      case CP_A:                                  // no EV
        contactor(false);
        cp_set_duty(CP_DUTY_STANDBY);             // +12 V
        pause_ms = 0;
        break;
      case CP_B:                                  // connected, not ready (or offer not yet posted)
        contactor(false);
        cp_set_duty(want ? AMP_TO_DUTY(amps) : CP_DUTY_STANDBY);
        pause_ms = 0;
        break;
      case CP_C:                                  // ready (entered under PWM with the checks passed)
        if (want) {
          cp_set_duty(AMP_TO_DUTY(amps));
          if (may_close()) contactor(true);       // else keep offering; close once the hold expires
          pause_ms = 0;
        } else {
          // graceful stop: drop the CP offer first, let the EV ramp down, then open.
          // (cp_state_raw keeps reporting C while the contactor is closed.)
          cp_set_duty(CP_DUTY_STANDBY);
          if (!pause_ms) pause_ms = millis();
          if (millis() - pause_ms > 3000) contactor(false);  // TODO: open on current-zero (CT)
        }
        break;
      default: contactor(false); cp_set_duty(CP_DUTY_STANDBY); break;
    }
  }
#ifndef LED_PROBE
  led_bar_update(st, g_contactor_closed && st == CP_C, g_latched);
#endif

  static uint32_t t_st = 0;
  if (millis() - t_st > 1000) {
    t_st = millis();
    float v = volts();
    float i = (CT_MEASURE || CT_READY) ? current_a() : 0.0f;   // no bogus current before calibration
    bool charging = g_contactor_closed && st == CP_C && want;
    meter_update(charging, v, i);
    float offered = ((st == CP_B || st == CP_C) && want) ? amps : 0.0f;
#if CT_READY
    // Over-current against the HIGHEST offer of the last 10 s, so a slow
    // ramp-down after a setpoint drop is not a fault; then welded contacts.
    static float   offer_hist[10] = {0}; static uint8_t oh = 0;
    offer_hist[oh] = g_contactor_closed ? offered : 0.0f; oh = (oh + 1) % 10;
    float ref = 0; for (uint8_t k = 0; k < 10; k++) if (offer_hist[k] > ref) ref = offer_hist[k];
    static uint32_t t_oc = 0, t_weld = 0;
    if (g_contactor_closed) {
      float lim = fmaxf(ref * 1.2f, ref + 3.0f);
      if (i > lim) { if (!t_oc) t_oc = millis(); if (millis() - t_oc >= OC_DELAY_MS) fault_enter(ERR_OVERCURRENT); }
      else t_oc = 0;
      t_weld = 0;
    } else {
      if (i > WELD_CURRENT_A) { if (!t_weld) t_weld = millis(); if (millis() - t_weld >= 2000) fault_enter(ERR_WELD); }
      else t_weld = 0;
      t_oc = 0;
    }
#endif
    // Shown and reported values are smoothed: one 20 ms window per second jumps by
    // +-1 A on a car. Protections and the energy meter above keep the
    // raw reading; with the contactor open the shown current follows the raw value
    // at once, so it drops to 0 right after a stop instead of decaying.
    static float v_show = 0.0f, i_show = 0.0f;
    v_show = (v_show == 0.0f) ? v : v_show + REPORT_SMOOTH_K * (v - v_show);
    i_show = g_contactor_closed ? i_show + REPORT_SMOOTH_K * (i - i_show) : i;
    g_led_amps = i_show;                          // LED bar animation speed
    CpState st_pub = g_latched ? CP_FAULT : st;   // F for the whole latched period
    esp_status(st_pub, v_show, i_show);
    DisplayData dd = {};
    dd.state     = (uint8_t)st_pub;
    dd.err       = (uint8_t)g_err;
    dd.offered_a = offered;
    dd.target_a  = amps;
    dd.amps      = i_show;
    dd.ct_ready  = (CT_MEASURE || CT_READY) != 0;
    dd.volts     = v_show;
    dd.tempC     = tempC_from(g_ntc_f);
    dd.kwh       = session_kwh();
    dd.elapsed_s = (st != CP_A) ? (millis() - g_session_start_ms) / 1000UL : 0;
    dd.sel       = g_sel;
    dd.mode_auto = (g_mode == MODE_AUTO);
    dd.link_ok   = (millis() - g_last_net_ms <= NET_TIMEOUT_MS);
    dd.enabled   = g_enabled;
    dd.contactor = g_contactor_closed;
    dd.clock_min = clock_minutes();
    dd.on_edf    = edf_now();
    dd.soc       = soc_now();
    display_show(dd);
  }

  display_tick();                                 // icon animation (cheap when idle)
  IWatchdog.reload();
}
#endif // DIAG_MODE
