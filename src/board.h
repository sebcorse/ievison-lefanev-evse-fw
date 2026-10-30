// ============================================================================
//  board.h - Pin map of the IEVISON / Lefanev LE-F30 charging station
//  MCU: STM32F103VET6 (12 MHz crystal). Found through SWD and measurements on
//  one station; the [?] pins are still to be confirmed.
//  Legend: [OK] confirmed  ·  [?] to be confirmed  ·  [est.] estimated
// ============================================================================
#pragma once
#include <Arduino.h>

// ---- Control Pilot ---------------------------------------------------------
//  The CP output stage follows PA7 = TIM3_CH2, not inverted (PA7 = 1 -> CP +12 V,
//  PA7 = 0 -> CP -12 V). PA6 has no effect while PA7 is driven. Never leave PA7
//  floating: the CP stage then drifts with any disturbance.
#define PIN_CP_GEN      PA7   // [OK] TIM3_CH2 hardware PWM, 1 kHz ; high = +12 V
#define CP_TIM_CHANNEL  2     // TIM3 channel of PIN_CP_GEN
#define CP_TIM_PIN      PA_7_ALT1  // PinName of TIM3_CH2 on PA7: the core lists PA_7 as
                                   // TIM1_CH1N first, so plain PA7 would hit Error_Handler()
#define CP_GEN_INVERTED 0     // 1 would select PWM mode 2 (output low during the +12 V time)
#define PIN_CP_POS      PC1   // [OK] CP positive half (ADC1_IN11): +12 V -> ~3667 ; <= 0 V -> ~0
#define PIN_CP_NEG      PC5   // [OK] CP full range, coarse (ADC1_IN15): +12 V -> ~2300, -12 V -> ~1475
                              //      (~34 counts/V): only used for the depth of the negative half
//  PC0 is not a CP sense: it carries a 50 Hz square wave (mains zero-cross
//  detector?).

// ---- Contactor (the two Hongfa relays, L+N, driven TOGETHER) ----------------
#define PIN_CONTACTOR PB1   // [OK] output, active HIGH (PB1=1 -> relays closed)
#define CONTACTOR_ON  HIGH
#define CONTACTOR_OFF LOW

// ---- Front-panel button (current selection 10/16/24 A) ---------------------
#define PIN_BUTTON    PE14  // [OK] input, active LOW (idle=1, pressed=0)

// ---- Analog measurements (ADC1) --------------------------------------------
//  Read with the direct ADC driver of main.cpp (never analogRead(), see there).
//  Idle readings, no vehicle: PA0 ~1540, PA1 ~4092, PA3 ~2, PC3 ~4,
//  PA4/PA5/PA6/PB0/PC4 ~2030-2110 (flat), PC0 = 50 Hz square, PA2 = 50 Hz
//  half-wave (peak ~2916), PC2 ~3225.
#define PIN_VSENSE    PA2   // [?] mains voltage: half-wave rectified 50 Hz (peak ~2916 at ~230 V) -> peak over 20 ms
#define PIN_NTC       PC2   // [OK] temperature: the reading FALLS when hot (~3225 at room
                            //      temperature); ntc_raw() returns 4095 - reading so higher = hotter
#define PIN_CT        PC3   // [OK] current: half-wave rectified 50 Hz from the CT stage
                            //     (~600 counts peak at 6 A); its MEAN over 20 ms follows the current

// ---- Topway LMT018DNBFWD-1 display (ILI9163C, 128x160 portrait) ---------------
//  Topway manual: 8080 8-bit parallel, controller ILI9163C (NOT ST7735S).
//  FPC: 1 BLK, 2 BLA, 3 VSS, 4 VDD, 5-6 NC, 7 /CS, 8 /RES, 9 D/C, 10 /WR, 11 /RD,
//  12..19 DB7..DB0, 20 VSS. On the main board the ribbon lands on J10, a latched
//  2x10 header numbered like a DIP (see README). Every line checked by continuity;
//  ribbon 5 (NC) reaches PB8, which has no role.
//  Backlight: BLA on 3V3, BLK on GND -> always on, no MCU control.
//    data D0..D7 : PD14 PD15 PD0 PD1 PE7 PE8 PE9 PE10
//    CS = PD7 (FSMC NE1, MCU pin 88)   RD = PD4 (NOE)   RST = PD11 (plain GPIO, pin 58)
//    D/C = PD13 = FSMC A18 (pin 60)    WR = PD5 = FSMC NWE (pin 86)
#define PIN_TFT_RST      PD11  // [OK] /RES, ribbon 8 -> pin 58 (+/-1: PD10 pin 57 / PD12 pin 59 if the panel never resets)
#define TFT_DC_ADDR_BIT  18    // [OK] D/C on FSMC A18 (PD13, pin 60); probing is +/-1 pin: 17 (PD12, pin 59) is the alternative
#define TFT_HAS_BL       0     // backlight hard-wired on; 1 + PIN_TFT_BL if a GPIO turns out to drive it
#define PIN_TFT_BL       PB8   //   (ribbon 5 is NC on the module: PB8 has no known role)
#define TFT_BL_ACTIVE    HIGH
#define TFT_CONTROLLER_ILI9163C 1   // 0 = ST7735S init sequence instead (fallback)
#define TFT_FSMC_ADDSET  3     // HCLK cycles (13.9 ns @ 72 MHz): conservative for the
#define TFT_FSMC_DATAST  4     //   ILI9163 write cycle (datasheet: see ILI9163B, tWC ~ 66 ns)
#define TFT_MADCTL       0xC8  // [OK] MY+MX, BGR (0x08 shows the picture upside down)
#define TFT_INVERT       0     // 1 if colours come out inverted
#define TFT_COL_OFFSET   0     // panel RAM offsets (ILI9163C RAM is 132x162; 128x160 panels usually 0/0)
#define TFT_ROW_OFFSET   0

// ---- Front LED bar: LED board 102-001-0175 on J11, 74HC595 + 7 segments x 2 LEDs ----
// LED board side: J1 green -> Q1 -> /OE (pin 13), white -> SER (14), yellow ->
// SRCLK (11), blue -> RCLK (12), black = 3.3 V (16), red = GND (8). Left
// floating, these MCU inputs let the bar light at random while current flows.
#define PIN_LED_RED   PE5   // [OK] green wire, Q1 base: HIGH lets the RED LEDs light
#define PIN_LED_SER   PE4   // [OK] white, 595 serial data
#define PIN_LED_SCK   PE3   // [OK] yellow, 595 shift clock (SRCLK)
#define PIN_LED_RCK   PE2   // [OK] blue, 595 latch (RCLK)
// Bicolour bar, 8 positions, 595 output k = position k from the TOP. Per output:
//   0 -> GREEN (whatever PIN_LED_RED);  1 -> RED if PIN_LED_RED is HIGH, else DARK.
// So: dark bar = all ones + PIN_LED_RED low; green on a dark background is possible,
// red only with every other position red or green. (Q1 switches the red LEDs only.)
#define LED_POSITIONS 8
// Charging animation: green filling from the top (the cable side), one position per
// step; step frequency proportional to the shown current, interpolated between these
// two step periods (tunable at run time with "LEDSPEED <ms at 6 A> <ms at 24 A>").
#define LED_STEP_MS_6A   800   // at 6 A
#define LED_STEP_MS_24A  125   // at 24 A

// ---- ESP bridge (UART, MQTT<->UART) ----------------------------------------
//  USART1 (PA9 TX / PA10 RX) via SP3232 (U15) channel 1 -> J12 (2 = TX, 3 = RX):
//  forcing PA9 low flips J12-2 from -5.7 V to +5.7 V; holding J12-3 high pulls
//  PA10 low. UART4 (PC10/PC11) only reaches U15 channel 2, which does NOT go to J12.
#define SERIAL_ESP_RX PA10
#define SERIAL_ESP_TX PA9
#define SERIAL_ESP    SerialJ12   // HardwareSerial on USART1, defined in main.cpp
#define SERIAL_ESP_BAUD 115200   // lower to 38400 if the cable run proves noisy

// ---- Earth-leakage protection = AUTONOMOUS HARDWARE (out of firmware) -------
//  Differential CT -> FM2147 (U2) -> SCR -> trip coil. Nothing to code here
//  (Fudan FM2147 datasheet: self-contained AC leakage trip).

// ============================================================================
//  EVSE constants
// ============================================================================
// CP duty cycle <-> offered current (IEC 61851 / J1772, 6..51 A linear range)
//   I[A] = duty% * 0.6   (e.g. 10 A -> 16.7 % ; 16 A -> 26.7 % ; 24 A -> 40 %)
#define CP_FREQ_HZ        1000
#define CP_DUTY_STANDBY   100.0f       // 100 % = steady +12 V = state A (no offer)
#define AMP_TO_DUTY(a)    ((a) / 0.6f) // duty percent
#define DUTY_TO_AMP(d)    ((d) * 0.6f)

#define EVSE_AMP_MIN      6            // IEC floor (duty 10 %)
#define EVSE_AMP_MAX      24           // operational cap (~5.5 kW). The hardware
                                       // supports 32 A (40 A relays, 6 mm2 cable)
                                       // but 24 A keeps a thermal / nameplate margin.
static const uint8_t PRESETS_A[] = { 10, 16, 24 };   // button presets

// CP sense, TWO channels:
//  PIN_CP_POS (PC1) reads the CP LINE, positive half only (clamped at ~0 below
//  0 V). Plateaus calibrated on a vehicle: A (nothing plugged) ~3680 | B ~2711
//  static, ~2683 under the 10 % PWM (the high phase is a ramp that has not fully
//  settled after 100 us) | C ~1795 (~+5.9 V) | D not measured (+3 V, estimated
//  ~880 from ~305 counts/V).
//  Decision BOUNDARIES at the mid-points - ordered, non-overlapping.
#define CP_POS_AB         3195        // [OK] at/above = A (no EV)
#define CP_POS_BC         2240        // [OK] at/above = B (connected) ; below = C
#define CP_POS_CD         1340        // [est.] below = D/E -> fault (D plateau not measured)
// Negative half, read on PIN_CP_NEG (PC5). PC5 does NOT follow the CP line: it
// stays at ~2250-2280 with or without a vehicle in B and reads ~2252 during the
// +V phase in C -> it sees the output stage BEFORE the 1 kOhm, not the line.
// So this check only proves that the -12 V half is generated (~1490-1525 when it
// is, ~2250 if the stage never swings negative). A vehicle WITHOUT its diode (or
// a resistive CP-PE leak) cannot be told apart on this board: no input senses the
// negative half of the line (PC1 clamps at 0 V). See the sequence check (B before
// C) in main.cpp.
#define CP_NEG_MAX        1700        // [OK as a drive check] at/below = the -12 V half is present

// State-machine debounce (as in OpenEVSE): a candidate state must persist before commit.
#define DELAY_STATE_MS    250
#define DELAY_STATE_A_MS  25

// NTC over-temperature, on the "heat" value h = ntc_raw() = 4095 - PIN_NTC
// reading (rises with heat). Beta model anchored on one measured point (31.7 degC
// on the sensor <-> h ~1008), B assumed 3435: if the real B is higher the model
// OVERestimates the heat (safe side). The divider puts the NTC on the low side
// (the reading falls when hot): R_ntc/R_fixed = raw / h.
// Thresholds below = the model's h at 70 / 65 / 85 / 80 degC.
#define NTC_REF_HEAT      1008        // [OK] h at NTC_REF_C
#define NTC_REF_C         31.7f       // degC, thermometer on the sensor
#define NTC_BETA          3435.0f     // [assumed] common 10 k NTC value; to check with a 2nd point
#define NTC_ADC_THROTTLE  2189        // ~70 degC: reduce to EVSE_AMP_MIN above this
#define NTC_ADC_PANIC     2604        // ~85 degC: hard cut above this

// ADC -> engineering units.
#define V_PER_COUNT       0.0789f     // [TODO] RMS volts per count of the half-wave PEAK (~2916 @ ~230 V)
// CT stage: on the stock firmware, U6 pin 1 reads 2.6 V DC and 0 V AC at 10 A,
// 0 V when idle -> a rectified + filtered DC level, ~0.26 V/A at the LM358
// output; PC3 sees it through a divider. Scale calibrated against a power meter
// at 10 and 16 A (residual ~1 %); idle reads 0.0 A.
#define CT_ZERO_COUNT     4           // [OK] mean ADC counts at 0 A
#define CT_A_PER_COUNT    0.0384f     // [OK] amps per count of the 20 ms mean above CT_ZERO_COUNT
#define REPORT_SMOOTH_K   0.2f        // shown / reported current and voltage: exponential smoothing
                                      // per 1 s sample (~5 s); protections and metering use raw values

// Network fallback
#define NET_TIMEOUT_MS    30000U      // no setpoint from the network -> fallback
#define NET_FALLBACK_A    EVSE_AMP_MIN
#define EDF_STALE_MS      180000U     // "INFO EDF" (screen only, sent every minute): forgotten after this

// ---- Fault latching, thermal hysteresis, cycle limits ----
#define FAULT_LATCH_MS        60000U  // minimum time spent in FAULT before any retry
#define FAULT_MAX_RETRIES     3       // after that many faults, wait for the EV to be unplugged (A)
#define SEQ_RETRY_MS          10000U  // ERR_SEQUENCE (C seen before B): short latch, unlimited retries
#define DELAY_STATE_CB_MS     50      // C -> B (the EV opened S2): open fast (IEC 61851-1: <= 100 ms)
#define NTC_ADC_THROTTLE_OFF  2038    // ~65 degC: leave the throttle below this
#define NTC_ADC_PANIC_OFF     2473    // ~80 degC: leave the cut below this (after the cooldown)
#define THERMAL_COOLDOWN_MS   60000U  // minimum time open after a thermal cut
#define NTC_MEDIAN_N          9       // median filter length (one sample every 100 ms)
#define NTC_PLAUSIBLE_MIN     100     // h < 100 = colder than ~-25 degC = open / unplugged sensor -> fault
#define NTC_PLAUSIBLE_MAX     4095    // (a shorted sensor reads hot -> the cut handles it)
#define MIN_CHARGE_MS         60000U  // anti short-cycling: a solar pause is honoured only after this time closed
#define MIN_PAUSE_MS          60000U  // ... and a resume only after this time paused ("ENABLE 0" is immediate)
#ifndef CT_MEASURE                    // (overridable with -D)
#define CT_MEASURE            1       // 1 = show the measured current, AMP and ENERGY
#endif
#ifndef CT_READY                      // (overridable with -D for the host simulation bench)
#define CT_READY              1       // CT scale calibrated: enables the over-current and
#endif                                // welded-contact rules and AMP/ENERGY
#define OC_DELAY_MS           5000U   // over-current must persist this long before latching a fault
#define WELD_CURRENT_A        2.0f    // current while the contactor is commanded open -> welded contacts
