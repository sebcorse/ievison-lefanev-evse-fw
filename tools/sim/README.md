# Host simulation bench

Compiles `src/main.cpp` **unchanged** on the development machine against a mock
of the Arduino / STM32duino / HAL calls it uses, with a simulated clock, an EV
model (states A/B/C, S2 timing, diode, current ramp) and a CP front-end model
(first-order RC, optional outliers). Scenarios replay charging sessions and
report every contactor action, whether it broke under load, fault latching,
the watchdog budget and the UART status lines. It is the regression tool for
the state machine: run it after every change to `main.cpp`.

It does **not** model the real ADC front-end, the EV population or EMC — it
checks the logic, not the hardware. Calibration still happens on the station.

## Build

```sh
tools/sim/build.sh        # -> sim, simdiag (-DDIAG_MODE), simct (-DCT_READY=1)
```

## Run

```sh
./sim <scenario> [adc_us] [tau_us] [verbose] [noise_p] [noise_mag] ...
```

`adc_us` = duration of one ADC read (default 10, as measured on the station), `tau_us` = CP sense RC
time constant (0 = ideal), `noise_p`/`noise_mag` = probability and size of an
isolated outlier per CP sample. `INVERT=1` in the environment inverts the CP
output stage (polarity fault). Scenario 15 prints raw-state statistics instead
of a session: `./sim 15 35 15 0 0 0 6 1` = 6 A offer, EV in B, tau 15 µs.

| # | Binary | Scenario | Expected |
|---|---|---|---|
| 1 | sim | `ENABLE 0` while charging | PWM withdrawn, EV opens S2, contactor opens with I = 0 (graceful) |
| 2 | sim | solar pause (`SET 0`) then `SET 10` | pause honoured ≥ 60 s after closing, re-close ≥ 60 s after opening |
| 3 | sim | EV without diode | **closes**: this board cannot sense the diode (see the main README, *Known limitation*) |
| 4 | sim | NTC above PANIC, then cold | immediate cut (ERR 3), release after cooldown + hysteresis |
| 5 | sim | `ENABLE` toggled every 2 s | one close, one graceful open, no re-close within 60 s |
| 6 | sim | EV ends charge (opens S2) | open within ~50 ms, I = 0 |
| 7 | sim | 1 h, 1 % low outliers | no spurious opening |
| 8 | sim | no diode + 5 % high outliers | closes, as S3 (no outlier-induced fault) |
| 9 | sim | EV stays in B under PWM | never closes |
| 10 | sim | unplug while charging | open within ~30 ms |
| 11–14 | sim | outlier noise in various phases | correct detection and timing |
| 15 | sim | raw-state statistics (see above) | front-end speed limit of the burst read |
| 16 | sim | boot with EV plugged | close at ~0.9 s after PWM + checks |
| 17 | sim | boot after a watchdog reset | first close held 60 s |
| 18 | sim | no EV, static 882 Ω CP-PE leak (C window, never B) | never closes, `ERR=10` retried every 10 s |
| 19 | sim | EV still in C at boot, opens S2 after 1 s without PWM | one `ERR=10`, then closes at ~11.5 s |
| 20–24 | simdiag | diag build interlock, `OFFER`, `CHARGE ON`, `SCAN` under load | closes only through the interlock; never with `INVERT=1` |
| 25 | sim | `TIME` (screen clock) | `HH:MM[:SS]` accepted, malformed lines ignored, midnight rollover, keeps counting for 60 days without `TIME` (across the `millis()` wrap) |
| 26 | sim | `INFO EDF 0\|1` (house supply, screen only) | exact lines accepted, others ignored, forgotten after `EDF_STALE_MS` |
| 30 | simct | down-modulation 24 → 10 A, EV reacts in 4.9 s | no over-current fault |
| 31 | simct | welded contacts | shows the CT cannot see a weld with a compliant EV (known limitation) |

## UI screenshots (`simui`)

`simui` compiles `src/display.cpp` against a **virtual ILI9163C** (CASET /
RASET / RAMWR decoded into a 128×160 frame buffer) and saves one PPM per UI
state (`ui_<name>.ppm`: boot, idle animation frames, connected, charging with
and without a calibrated CT, paused, no link, faults, disabled, diag screen).
It also prints the bytes written on the bus per screen and checks that an
unchanged refresh writes nothing (field cache) and that no pixel falls outside
the panel. `ui_sheet.py` (Pillow) tiles them into `docs/ui-preview.png`.

## Interactive simulator (`simlive` + `live.py`)

The firmware running in real time on the development machine, in a browser
page, with the same EV / CP / ADC model as the scenarios and the virtual
ILI9163C for the screen:

```sh
tools/sim/build.sh
tools/sim/live.py            # operational firmware, http://127.0.0.1:8765
tools/sim/live.py --diag     # diagnostic build (OFFER / CHARGE ON / SCAN over the UART box)
```

The page shows the front panel (screen + button), the simulated world (plug /
unplug the EV, EV wants to charge or is complete, diode present or missing,
NTC counts, CP front-end time constant, simulation speed up to x60, power
cycle), the ESP bridge commands (`SET`, `MODE`, `ENABLE`, or any raw UART
line) and, live, the UART output and the events (CP PWM, S2, contactor with
"break under load" flagged). `simlive` itself is a plain line protocol on
stdin / stdout (see the header of `sim_live.cpp`), so it can also be scripted.
It is the tool to rehearse every situation on the real screen layout before
the irreversible flash; it does not replace calibration on the station.
