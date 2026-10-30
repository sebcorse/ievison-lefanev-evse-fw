#!/bin/bash
# flash-over-bridge.sh - update the station firmware WITHOUT the SWD probe,
# through the ESP bridge and the STM32 ROM bootloader on J12 (see README,
# "Firmware update without the probe").
#
#   tools/flash-over-bridge.sh [firmware.bin] [baud]
#
# Defaults: .pio/build/evse/firmware.bin, 19200 baud (115200 not tried with
# the Python flasher yet). Needs here: python3 and ssh access to the MQTT
# broker host (the MQTT commands run there, on its localhost listener).
# Settings, from the environment or tools/local.env (not tracked by git):
#   ESP=<address of the ESP bridge>   BROKER_SSH=<user@host of the MQTT broker>
# The flashing itself is tools/stm32_bridge_flash.py (AN3155 over TCP, every
# block read back, retries): stm32flash through socat stumbled on the WiFi
# latency, the Python flasher writes 56 KB in ~130 s with no retry.
#
# The station must be idle: no vehicle (state A), contactor open, otherwise
# the firmware answers "BOOTLOADER refused". If anything fails half-way the
# STM32 stays in its ROM bootloader (CP held at +12 V, contactor open): just
# run tools/stm32_bridge_flash.py again while the bridge is in flash mode.
set -euo pipefail
FW=${1:-.pio/build/evse/firmware.bin}
BAUD=${2:-19200}
HERE=$(cd "$(dirname "$0")" && pwd)
if [ -f "$HERE/local.env" ]; then . "$HERE/local.env"; fi
ESP=${ESP:?set ESP to the address of the ESP bridge (environment or tools/local.env)}
BROKER_SSH=${BROKER_SSH:?set BROKER_SSH to user@host of the MQTT broker (environment or tools/local.env)}

pub() { ssh -n -o ConnectTimeout=6 "$BROKER_SSH" "mosquitto_pub -h 127.0.0.1 -t '$1' -m '$2'" 2>/dev/null; }
raw_line() {  # print the first station line matching $1 within a few seconds
  ssh -n -o ConnectTimeout=6 "$BROKER_SSH" \
    "timeout 5 mosquitto_sub -h 127.0.0.1 -t tele/evse-bridge/sensor/evse_raw_line/state | grep -m1 '$1'" 2>/dev/null || true
}

[ -f "$FW" ] || { echo "no firmware file $FW"; exit 1; }
echo "== firmware $FW ($(wc -c < "$FW") bytes), $BAUD baud 8E1 via $ESP"

echo "== 1-2. BOOTLOADER, then the bridge in flash mode at once"
# An energy manager typically re-sends evse/enable + evse/set_current every few
# seconds. One of them reaching the station after it has entered its ROM bootloader,
# but before the bridge is in flash mode, becomes the bootloader's baud-rate byte and
# leaves it deaf until a power cycle. So wait for one of those commands (15 s at most),
# then send BOOTLOADER and switch the bridge in the same session: the window shrinks
# to a fraction of a second, well ahead of the next one.
reply=$(ssh -o ConnectTimeout=6 "$BROKER_SSH" "BAUD=$BAUD bash -s" 2>/dev/null <<'EOS'
timeout 15 mosquitto_sub -h 127.0.0.1 -t evse/enable -C 1 >/dev/null 2>&1
f=$(mktemp)
timeout 5 mosquitto_sub -h 127.0.0.1 -t tele/evse-bridge/sensor/evse_raw_line/state -R 2>/dev/null \
  | grep -m1 BOOTLOADER > "$f" 2>/dev/null &
sleep 0.3
mosquitto_pub -h 127.0.0.1 -t evse/raw -m BOOTLOADER
for i in $(seq 50); do [ -s "$f" ] && break; sleep 0.1; done
r=$(cat "$f"); rm -f "$f"
if echo "$r" | grep -q rebooting; then mosquitto_pub -h 127.0.0.1 -t evse/flash -m "$BAUD"; fi
echo "$r"
EOS
)
echo "   station: ${reply:-<no answer>}"
case "$reply" in *rebooting*) ;; *) echo "   not accepted, stopping"; exit 1;; esac
sleep 2                                         # let the bridge open its TCP port

echo "== 3. flash (tools/stm32_bridge_flash.py)"
# An ESP that restarts during the flash comes back in normal mode with its port
# closed: the flasher then runs this to get flash mode back.
REARM="ssh -n -o ConnectTimeout=6 $BROKER_SSH \"mosquitto_pub -h 127.0.0.1 -t evse/flash -m $BAUD\""
rc=0
python3 "$(dirname "$0")/stm32_bridge_flash.py" "$FW" --host "$ESP" --rearm "$REARM" || rc=$?

if [ "$rc" -ne 0 ]; then
  # Keep the bridge in flash mode: back in normal mode it would forward the energy
  # manager's commands to a bootloader that may hold a half-written image.
  echo "== flash FAILED (exit $rc). The bridge STAYS in flash mode (auto-off after 10 min idle)."
  echo "   Run again within 10 min: python3 $(dirname "$0")/stm32_bridge_flash.py $FW --host $ESP --rearm '$REARM'"
  echo "   Meanwhile open no other connection to port 6638: the bridge would drop the flasher."
  echo "   Do NOT power-cycle the station if blocks were written: the image may be incomplete."
  exit "$rc"
fi
echo "== 4. bridge back to normal"
pub evse/flash 0; sleep 2

echo "== 5. running version"
( sleep 1; pub evse/raw VERSION ) &
v=$(raw_line BOOT); wait
echo "   ${v:-<no answer>}"
