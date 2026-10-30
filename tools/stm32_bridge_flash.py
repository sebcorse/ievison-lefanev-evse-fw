#!/usr/bin/env python3
"""stm32_bridge_flash.py - flash the station's STM32 through the ESP bridge.

Speaks the STM32 ROM bootloader protocol (ST AN3155) over the bridge's TCP
port (flash mode, see esp-bridge/components/tcp_uart_bridge). Written because
stm32flash through socat stumbled on the WiFi latency ("unexpected reply"),
whereas the link itself is clean (30 x 256 random bytes written to RAM and read
back without a single error at 19200 baud).

    stm32_bridge_flash.py firmware.bin [--host BRIDGE] [--port 6638] [--rearm CMD]

BRIDGE (the ESP bridge's address) defaults to $ESP.

Erases only the pages the image needs, writes 256-byte blocks, reads every
block back, retries a block (after resynchronising the bootloader) up to 8
times, then jumps to the new firmware ("go"). Survives a dropped link: it
reconnects, unsticks a bootloader left in the middle of a frame, and runs CMD
(which must put the bridge back in flash mode) if the bridge restarted and
closed its port. Standard library only. The bridge serves ONE client and drops
the previous one: never open another connection to its port during a flash.
"""
import argparse
import os
import socket
import subprocess
import sys
import time

ACK, NACK = 0x79, 0x1F
BASE = 0x08000000
PAGE = 2048                                    # STM32F103 high density
GET_REPLY_PREFIX = bytes([ACK, 0x0B])          # GET: ACK, N=11, version, commands, ACK


class Bootloader:
    def __init__(self, host, port, rearm=None):
        self.host, self.port, self.s = host, port, None
        self.rearm = rearm                      # shell command: bridge back in flash mode
        self.buf = b""                          # bytes received beyond what was asked
        self.connect()

    def connect(self, tries=30):
        """(Re)open the TCP link. The bridge stays in flash mode while we are away,
        so the bootloader keeps its state and nothing else reaches it (back in
        normal mode after a Wi-Fi drop, the bridge would forward the energy manager's
        commands, and a stray byte makes the bootloader deaf)."""
        for k in range(tries):
            try:
                if self.s is not None:
                    self.s.close()
                self.s = socket.create_connection((self.host, self.port), timeout=5)
                self.s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                self.buf = b""
                time.sleep(0.3)
                self.drain()
                return
            except OSError as e:
                print(f"\n  bridge unreachable ({e}), retry {k + 1}/{tries}")
                if isinstance(e, ConnectionRefusedError) and self.rearm and k % 5 == 0:
                    # Port closed while the ESP answers: it restarted and left flash
                    # mode. Ask again, so that the port reopens and the energy
                    # manager's commands stop reaching the ROM.
                    print("  bridge port closed: asking for flash mode again")
                    try:
                        subprocess.run(self.rearm, shell=True, timeout=20)
                    except (OSError, subprocess.SubprocessError) as e2:
                        print(f"  rearm command failed ({e2})")
                time.sleep(2)
        raise RuntimeError("bridge unreachable")

    def drain(self):
        self.buf = b""
        self.s.settimeout(0.05)
        try:
            while self.s.recv(1024):
                pass
        except (socket.timeout, BlockingIOError):
            pass

    def xfer(self, tx, n, wait=2.0):
        """Send tx, return exactly n reply bytes (fewer on timeout); extra bytes are kept."""
        if tx:
            self.s.sendall(bytes(tx))
        t0 = time.time()
        self.s.settimeout(0.05)
        while len(self.buf) < n and time.time() - t0 < wait:
            try:
                d = self.s.recv(1024)
                if d:
                    self.buf += d
            except socket.timeout:
                pass
        r, self.buf = self.buf[:n], self.buf[n:]
        return r

    def sync(self, single=8, burst_bytes=640):
        """Send 0x7F until the bootloader answers, and NOTHING else before that.

        The ROM measures its baud rate on the first byte it receives: if that 0x7F
        is lost (bridge not yet serving the new client...) and the next byte is a
        command, it locks onto a wrong rate and stays deaf until the next reset.
        ACK = synchronised now, NACK = already synchronised by an earlier 0x7F.

        A bootloader left in the middle of a frame by a dropped link swallows these
        bytes as frame data and stays silent: then keep sending 0x7F, 8 at a time,
        until the frame is complete and it answers (64 bytes were needed once).
        Whatever the cut frame does
        with them is harmless: the image is erased and written again afterwards."""
        for _ in range(single):
            r = self.xfer([0x7F], 1, 0.6)
            if r[:1] in (bytes([ACK]), bytes([NACK])):
                return True
            self.drain()
        for _ in range(burst_bytes // 8):
            r = self.xfer([0x7F] * 8, 1, 0.08)
            if r[:1] in (bytes([ACK]), bytes([NACK])):
                time.sleep(0.3)
                self.drain()                    # the NACKs of the remaining pairs
                return True
        return False

    def resync(self):
        """Bring the command stream back to a command boundary (GET until a clean reply)."""
        for _ in range(12):
            self.drain()
            r = self.xfer([0x00, 0xFF], 15, 0.8)
            if r.startswith(GET_REPLY_PREFIX) and len(r) == 15 and r[-1] == ACK:
                return r
            if not r:                           # silent: still inside a cut frame
                self.sync(single=0)
            else:
                self.xfer([0x00], 1, 0.3)
        raise RuntimeError("bootloader does not answer GET (is the bridge in flash mode?)")

    def cmd(self, c):
        return self.xfer([c, c ^ 0xFF], 1) == bytes([ACK])

    @staticmethod
    def addr(a):
        b = a.to_bytes(4, "big")
        return list(b) + [b[0] ^ b[1] ^ b[2] ^ b[3]]

    def get_id(self):
        if not self.cmd(0x02):
            return None
        r = self.xfer([], 4)
        return (r[1] << 8 | r[2]) if len(r) == 4 and r[0] == 1 and r[3] == ACK else None

    def erase_pages(self, pages):
        if not self.cmd(0x43):
            return False
        frame = [len(pages) - 1] + list(pages)
        cs = 0
        for x in frame:
            cs ^= x
        return self.xfer(frame + [cs], 1, 30.0) == bytes([ACK])

    def write(self, a, data):
        if not self.cmd(0x31):
            return False
        if self.xfer(self.addr(a), 1) != bytes([ACK]):
            return False
        frame = [len(data) - 1] + list(data)
        cs = 0
        for x in frame:
            cs ^= x
        return self.xfer(frame + [cs], 1, 5.0) == bytes([ACK])

    def read(self, a, n):
        if not self.cmd(0x11):
            return None
        if self.xfer(self.addr(a), 1) != bytes([ACK]):
            return None
        r = self.xfer([n - 1, (n - 1) ^ 0xFF], n + 1, 5.0)
        return r[1:] if len(r) == n + 1 and r[0] == ACK else None

    def go(self, a):
        return self.cmd(0x21) and self.xfer(self.addr(a), 1) == bytes([ACK])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("firmware")
    ap.add_argument("--host", default=os.environ.get("ESP"),
                    help="address of the ESP bridge (default: $ESP)")
    ap.add_argument("--port", type=int, default=6638)
    ap.add_argument("--no-go", action="store_true", help="stay in the bootloader after writing")
    ap.add_argument("--rearm", help="shell command putting the bridge back in flash mode, "
                                    "run when its port refuses connections")
    a = ap.parse_args()
    if not a.host:
        ap.error("give --host, or set ESP to the address of the ESP bridge")

    img = open(a.firmware, "rb").read()
    img += b"\xFF" * (-len(img) % 4)              # writes in whole words
    blocks = [(BASE + o, img[o:o + 256]) for o in range(0, len(img), 256)]
    pages = list(range((len(img) + PAGE - 1) // PAGE))
    print(f"image {len(img)} bytes, {len(blocks)} blocks, pages 0..{pages[-1]}")

    bl = Bootloader(a.host, a.port, a.rearm)
    if not bl.sync():
        sys.exit("no answer to 0x7F: the bootloader is not listening (or is locked on a wrong baud "
                 "rate by an earlier stray byte). If an earlier attempt had started writing, do NOT "
                 "power-cycle (the image is incomplete): recover with the ST-Link. Otherwise "
                 "power-cycle the station and start again.")
    print("sync: ok")
    g = bl.resync()
    print(f"bootloader v{g[2] >> 4}.{g[2] & 15}, commands {g[3:-1].hex(' ')}")
    pid = bl.get_id()
    print(f"device id: 0x{pid:04x}" if pid else "device id: ?")
    if pid != 0x0414:
        sys.exit("unexpected device id, stopping (expected 0x0414, STM32F10x high density)")

    t0 = time.time()
    for attempt in range(3):
        if bl.erase_pages(pages):
            break
        print("erase failed, resync and retry")
        bl.resync()
    else:
        sys.exit("erase failed")
    print(f"erased {len(pages)} pages in {time.time() - t0:.1f} s")

    retries = reconnects = 0
    for i, (addr, data) in enumerate(blocks):
        for attempt in range(8):
            try:
                if bl.write(addr, data) and bl.read(addr, len(data)) == data:
                    break
                retries += 1
                print(f"\n  block 0x{addr:08x}: retry {attempt + 1}")
                bl.resync()
            except (OSError, RuntimeError) as e:     # link dropped: reconnect, same block again
                reconnects += 1
                print(f"\n  link lost at block 0x{addr:08x} ({e}): reconnecting")
                try:
                    bl.connect()
                    if not bl.sync():                     # 0x7F first, never a command
                        raise RuntimeError("no answer to 0x7F")
                    bl.resync()
                except (OSError, RuntimeError) as e2:     # dropped again while resynchronising
                    print(f"  still no bootloader ({e2})")
        else:
            sys.exit(f"\nblock 0x{addr:08x} failed 8 times, stopping. The bootloader may still be "
                     "waiting: keep the bridge in flash mode and run this again. Do NOT power-cycle.")
        print(f"\r  {i + 1}/{len(blocks)} blocks written and verified", end="", flush=True)
    print(f"\nwritten and verified in {time.time() - t0:.0f} s, {retries} retries, {reconnects} reconnections")

    if a.no_go:
        print("staying in the bootloader (--no-go)")
    else:
        print("go 0x08000000:", "ok" if bl.go(BASE) else "no ACK (power-cycle the station)")


if __name__ == "__main__":
    main()
