# Network bridge (MQTT <-> RS-232)

The station has no network. An **ESP8266** running [ESPHome](https://esphome.io/)
relays MQTT to its RS-232 port (**J12**) and back. It is a dumb relay: no
charging logic lives here, all safety and timing are in the STM32 firmware. If
the bridge goes silent, the station falls back to 6 A after 30 s (in AUTO).

WiFi does not work from inside the closed station, so the ESP sits indoors and
reaches J12 through a few metres of cable (an Ethernet cable through a cable
gland keeps the enclosure sealed).

## Wiring

```
ESP TX  D5/GPIO14, INVERTED 0/3.3 V ──────────────────────────────► J12-3  station RX (its SP3232 receiver)
ESP RX  D7/GPIO13 (UART0, swapped) ◄── MAX3232 module receiver ◄─── J12-2  station TX (RS-232, idles -5.5 V)
ESP GND ─────────────────────────────────────────────────────────── J12-1  GND          (J12-4 +12 V unused)
10 kOhm D7 -> GND ; the MAX3232 module powered from the ESP's 3V3 / GND
```

- **Transmit without a line driver.** The station's input is a genuine SP3232
  receiver: it switches around 1.5 V, tolerates ±25 V and loads the line with
  5 kOhm. An ESP output with inverted logic (idle 0 V = RS-232 mark) drives it
  directly.
- **Receive through a level shifter.** J12-2 swings ±5.5 V, which would damage
  the ESP: it goes through the receiver half of a MAX3232 module.
- **Cheap "MAX3232" modules are often counterfeit.** Their transmitter cannot
  hold its level under load (useless here, hence the direct TX); their receiver
  works with a 10 kOhm pull-down on its output. Their pads lift easily: glue the
  wires down. Power the module from the ESP's 3.3 V: the ESP8266 RX is not 5 V
  tolerant.
- **Power** the ESP from an ordinary (isolated) USB charger. The station's low
  voltage side is isolated from the mains too, so joining the grounds through
  the cable is fine. Never use a non-isolated mains device (on a Sonoff-type
  relay the logic ground sits at mains potential).
- **ESP8266 pins:** GPIO1/GPIO3 belong to the on-board USB chip and GPIO15 (D8)
  must be low at boot, hence D5/D7. A software serial cannot receive while it
  sends: test a loopback with a hardware UART only.

## ESPHome

[`evse-bridge-esp8266.yaml`](evse-bridge-esp8266.yaml) (NodeMCU; a D1 mini
works too), with a `secrets.yaml` next to it:

```yaml
wifi_ssid: "..."
wifi_password: "..."
mqtt_broker: "192.168.1.10"     # your MQTT broker
ntp_server: "192.168.1.1"       # any NTP server the ESP can reach (time of day on the screen)
timezone: "Europe/Paris"
```

`esphome run evse-bridge-esp8266.yaml`: the first time over USB (unplug J12
first), then over the air. ESPHome rather than Tasmota because the wiring needs
an inverted TX on one pin next to a non-inverted hardware RX.

## MQTT

Commands, each written to the station as one line:

| Topic | Payload | Line sent |
|---|---|---|
| `evse/set_current` | amps, e.g. `12.5` | `SET 12.5` |
| `evse/mode` | `AUTO` / `MAN` | `MODE AUTO` / `MODE MAN` |
| `evse/enable` | `1` / `0` (or `ON` / `OFF`) | `ENABLE 1` / `ENABLE 0` |
| `evse/raw` | any line | as is: `PRESET 16`, `INFO EDF 0`, `VERSION`, `BOOTLOADER`, diag commands... |
| `evse/flash` | a baud rate / `0` | flash mode on / off (below) |

States go under `tele/evse-bridge/` (MQTT discovery off: some broker ACLs only
let devices write under `tele/#`; define the Home Assistant entities from
these topics):

- `tele/evse-bridge/sensor/<name>/state` for `evse_state`, `evse_error_code`,
  `evse_offered_current`, `evse_current`, `evse_voltage`, `evse_temperature`,
  `evse_session_energy`, `evse_session_minutes`, `evse_selection`,
  `evse_raw_line` (every status line, verbatim) and `evse_bridge_wifi_signal`;
- `tele/evse-bridge/binary_sensor/evse_charging_enabled/state`;
- `tele/evse-bridge/status` (`online` / `offline`) and `tele/evse-bridge/debug`
  (the ESPHome log, every byte in and out of the UART).

The ESP also sends `TIME HH:MM:SS` to the station on the minute, from SNTP.

## Firmware update through the bridge

Publish `BOOTLOADER` on `evse/raw`: with no vehicle and the contactor open, the
STM32 reboots into its ROM bootloader on J12. Then publish the baud rate on
`evse/flash`: both UARTs switch to 8E1 and TCP port 6638 relays them (status
parsing and `evse/*` commands pause); `0` brings the bridge back to normal, and
it does so by itself after 10 min without traffic.

[`../tools/stm32_bridge_flash.py`](../tools/stm32_bridge_flash.py) speaks the
ROM bootloader protocol (ST AN3155) over that port: it erases only the pages it
needs, reads every block back, retries, reconnects. 
[`../tools/flash-over-bridge.sh`](../tools/flash-over-bridge.sh) runs the whole
sequence (it needs ssh access to the MQTT broker host and, in
`tools/local.env`, `ESP=` the bridge's address and `BROKER_SSH=`).

- Send `BOOTLOADER` and switch the bridge at once: a stray command reaching the
  ROM before its first sync byte becomes its baud-rate byte and leaves it deaf
  until a power cycle. The script does it in one go.
- The bridge serves one TCP client and drops the previous one: open no other
  connection to port 6638 during a flash.
- If a flash fails half-way, the STM32 stays in its bootloader (CP at +12 V,
  contactor open): run the flasher again, do not power-cycle the station.

## Driving it from an energy manager

- Publish the setpoint (6-24 A) on `evse/set_current` and `AUTO` on
  `evse/mode`, and re-send them periodically (every ~10 s): the station boots
  in AUTO with nothing persisted and falls back to 6 A after 30 s of silence.
- A setpoint below 6 A pauses the charge, honoured only after 60 s of charging
  (anti short-cycling); `ENABLE 0` stops at once. Changing the setpoint between
  6 and 24 A only changes the PWM: no relay action.
- Follow OpenEVSE's "divert" logic (`ESP32_WiFi_V4.x/src/divert.cpp`): smooth
  the available current (20 s when it rises, 600 s when it falls), start at
  6.5 A, and stop at 6 A only after 10 minutes of charge.
