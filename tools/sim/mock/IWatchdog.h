// Host mock of STM32duino's IWatchdog (tools/sim only). Records the longest
// gap between two reload() calls so a scenario can check the 2 s budget.
#pragma once
#include <stdint.h>
class IWatchdogClass {
public:
  uint64_t last_reload_us = 0, max_gap_us = 0, started = 0;
  void begin(uint32_t);
  void reload();
};
extern IWatchdogClass IWatchdog;
