# Flashing the station with an ST-Link V2 and OpenOCD

Wiring (see the main README): ST-Link `SWDIO → J9 DIO`, `SWCLK → J9 CLK`,
`GND → J9 G`, `RST → NRST lead (pad of R57)`. Leave the probe's 3.3 V output
unconnected while the station is mains-powered. Keep the upstream breaker /
relay as your own cut-off; the contactor is never commanded by anything below.

OpenOCD here is the one bundled with PlatformIO:

```sh
OCD=~/.platformio/packages/tool-openocd/bin/openocd
CFG=tools/openocd/station.cfg
```

## 1. Probe and identify (non-destructive)

```sh
$OCD -f $CFG -c "init; reset halt; stm32f1x options_read 0; shutdown"
```

Expect `IDCODE 0x10036414` (STM32F103 high density), a clean halt, and the
option bytes showing **read protection ON** (stock firmware, RDP level 1).
If OpenOCD cannot read the IDR, the probe's RST pin is probably not driving
NRST: hold the NRST lead to GND by hand while the command starts.

## 2. Remove the read protection — IRREVERSIBLE

This mass-erases the chip. The stock firmware is gone for good; no backup is
possible (that is the whole reason this project exists).

```sh
$OCD -f $CFG -c "init; reset halt; stm32f1x unlock 0; shutdown"
```

Then **power-cycle the station** (the option bytes are loaded at reset), and
read them back:

```sh
$OCD -f $CFG -c "init; reset halt; stm32f1x options_read 0; shutdown"
```

Expect read protection OFF, no write protection, `WDG_SW` = software watchdog.

## 3. Flash the diagnostic build first

```sh
pio run -e diag
$OCD -f $CFG -c "program .pio/build/diag/firmware.elf verify reset exit"
```

(`pio run -e diag -t upload` does the same through PlatformIO's own OpenOCD
invocation once the chip is unlocked.) Open the UART on J12 at 115200 and follow
the calibration procedure in the main README. Only then the `evse` build.

## 4. Later updates

This firmware leaves the SWD port enabled, so the NRST lead is no longer needed:
connect without reset (many ST-Link clones do not drive their RST pin anyway),
halt the running core, write, verify and reset by software.

```sh
$OCD -f $CFG -c "reset_config none separate" \
     -c "program .pio/build/evse/firmware.elf verify reset exit"
```

Without the probe at all: the ROM bootloader on J12, through the ESP bridge
(`tools/flash-over-bridge.sh`, see the main README).
