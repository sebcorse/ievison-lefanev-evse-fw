// Host mock of the bits of Arduino / STM32duino / HAL used by src/main.cpp.
// For the simulation bench only (tools/sim) - never compiled into the firmware.
#pragma once
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <math.h>
#include <functional>
#include <string>
#include <deque>

#define HIGH 1
#define LOW 0
#define OUTPUT 1
#define INPUT 0
#define INPUT_PULLUP 2
#define INPUT_ANALOG 5

enum SimPin : uint32_t { PA0=0, PA1, PA2, PA3, PA4, PA5, PA6, PA7, PA9, PA10, PB0, PB1, PB3, PB8, PB9,
  PC0, PC1, PC2, PC3, PC4, PC5, PD6, PD11, PD12, PD13, PE2, PE3, PE4, PE5, PE14, SIM_NPINS };
#define NC ((uint32_t)0xFFFFFFFFu)
#define PA_7_ALT1 PA7          // STM32duino PinName of TIM3_CH2 on PA7 (board.h CP_TIM_PIN)

uint32_t millis();
uint32_t micros();
void delay(uint32_t ms);
void pinMode(uint32_t pin, uint32_t mode);
void digitalWrite(uint32_t pin, uint32_t val);
int  digitalRead(uint32_t pin);
uint16_t analogRead(uint32_t pin);
inline void analogReadResolution(int) {}

class Print {
public:
  virtual ~Print() {}
  virtual size_t write(const uint8_t *buf, size_t n) = 0;
  int printf(const char *fmt, ...) __attribute__((format(printf, 2, 3))) {
    char b[512]; va_list ap; va_start(ap, fmt); int r = vsnprintf(b, sizeof b, fmt, ap); va_end(ap);
    write((const uint8_t *)b, strlen(b)); return r;
  }
};
class Stream : public Print {
public:
  virtual int available() = 0;
  virtual int read() = 0;
};
class SimSerial : public Stream {
public:
  std::deque<char> rx; std::string line; std::deque<std::string> out;
  void begin(uint32_t) {}
  void flush() {}
  int available() override { return (int)rx.size(); }
  int read() override { if (rx.empty()) return -1; char c = rx.front(); rx.pop_front(); return (unsigned char)c; }
  size_t write(const uint8_t *buf, size_t n) override;
};
extern SimSerial SerialJ12;

typedef enum { TICK_FORMAT, MICROSEC_FORMAT, HERTZ_FORMAT } TimerFormat_t;
typedef enum { TICK_COMPARE_FORMAT } TimerCompareFormat_t;
#define TIMER_OUTPUT_COMPARE 0
#define TIMER_OUTPUT_COMPARE_PWM1 1
#define TIMER_OUTPUT_COMPARE_PWM2 2
typedef struct { int dummy; } TIM_TypeDef;
extern TIM_TypeDef *TIM3;
class HardwareTimer {
public:
  uint32_t psc = 1, ovf = 65536, ccr = 0;
  std::function<void()> cb_ovf, cb_cc;
  explicit HardwareTimer(TIM_TypeDef *) {}
  uint32_t getTimerClkFreq() { return 72000000u; }
  void setPrescaleFactor(uint32_t p) { psc = p; }
  uint32_t getPrescaleFactor() { return psc; }
  void setOverflow(uint32_t o, TimerFormat_t) { ovf = o; }
  uint32_t getOverflow(TimerFormat_t) { return ovf; }
  void setMode(uint32_t, int, uint32_t) {}
  void setCaptureCompare(uint32_t, uint32_t c, TimerCompareFormat_t) { ccr = c; }
  void attachInterrupt(std::function<void()> f) { cb_ovf = f; }
  void attachInterrupt(uint32_t, std::function<void()> f) { cb_cc = f; }
  void resume() {}
};

// ---- HAL clock stubs (SystemClock_Config) ----
typedef struct { uint32_t PLLState, PLLSource, PLLMUL; } RCC_PLLInitTypeDef;
typedef struct { uint32_t OscillatorType, HSEState, HSEPredivValue; RCC_PLLInitTypeDef PLL; } RCC_OscInitTypeDef;
typedef struct { uint32_t ClockType, SYSCLKSource, AHBCLKDivider, APB1CLKDivider, APB2CLKDivider; } RCC_ClkInitTypeDef;
typedef struct { uint32_t PeriphClockSelection, AdcClockSelection; } RCC_PeriphCLKInitTypeDef;
enum { HAL_OK = 0 };
#define RCC_OSCILLATORTYPE_HSE 1
#define RCC_HSE_ON 1
#define RCC_HSE_PREDIV_DIV1 0
#define RCC_PLL_ON 1
#define RCC_PLLSOURCE_HSE 1
#define RCC_PLL_MUL6 6
#define RCC_CLOCKTYPE_SYSCLK 1
#define RCC_CLOCKTYPE_HCLK 2
#define RCC_CLOCKTYPE_PCLK1 4
#define RCC_CLOCKTYPE_PCLK2 8
#define RCC_SYSCLKSOURCE_PLLCLK 2
#define RCC_SYSCLK_DIV1 0
#define RCC_HCLK_DIV2 4
#define RCC_HCLK_DIV1 0
#define FLASH_LATENCY_2 2
#define RCC_PERIPHCLK_ADC 2
#define RCC_ADCPCLK2_DIV6 2
inline int HAL_RCC_DeInit() { return HAL_OK; }
inline int HAL_RCC_OscConfig(RCC_OscInitTypeDef *) { return HAL_OK; }
inline int HAL_RCC_ClockConfig(RCC_ClkInitTypeDef *, uint32_t) { return HAL_OK; }
inline int HAL_RCCEx_PeriphCLKConfig(RCC_PeriphCLKInitTypeDef *) { return HAL_OK; }

// ---- HAL reset-cause flags (boot_reset_cause) ----
// A scenario may set sim_rcc_iwdg_flag() = 1 before setup() to simulate a
// watchdog reset; the firmware clears it like the real __HAL_RCC_CLEAR_RESET_FLAGS.
inline int &sim_rcc_iwdg_flag() { static int f = 0; return f; }
#define RCC_FLAG_IWDGRST 0x1D
#define __HAL_RCC_GET_FLAG(flag) (sim_rcc_iwdg_flag())
#define __HAL_RCC_CLEAR_RESET_FLAGS() (sim_rcc_iwdg_flag() = 0)
