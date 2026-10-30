#!/usr/bin/env python3
"""Browser front-end for the interactive simulator (tools/sim/simlive).

    tools/sim/build.sh && tools/sim/live.py [--diag] [--port 8765]

then open http://127.0.0.1:8765 : the front panel (screen + button), the
simulated EV, the NTC, the ESP/MQTT commands and the UART log, in real time
(or faster). Needs Pillow for the PNG frames.
"""
import argparse, collections, io, json, os, subprocess, sys, threading, time
from http.server import ThreadingHTTPServer, BaseHTTPRequestHandler
from PIL import Image

HERE = os.path.dirname(os.path.abspath(__file__))

class Sim:
    def __init__(self, binary):
        self.binary = binary
        self.lock = threading.Lock()
        self.frame_png = None; self.frame_seq = 0
        self.state = {}; self.fw_line = ""
        self.uart = collections.deque(maxlen=400); self.log = collections.deque(maxlen=400)
        self.proc = None; self.start()

    def start(self):
        with self.lock:
            if self.proc: self.proc.kill()
            self.proc = subprocess.Popen([self.binary], stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                         text=True, bufsize=1, cwd=HERE)
            self.uart.clear(); self.log.clear(); self.state = {}; self.fw_line = ""
            self.log.append("[live] started %s" % os.path.basename(self.binary))
        threading.Thread(target=self.reader, args=(self.proc,), daemon=True).start()

    def reader(self, proc):
        for line in proc.stdout:
            line = line.rstrip("\n")
            if line.startswith("FRAME "):
                raw = bytes.fromhex(line[6:])
                img = Image.frombytes("RGB", (128, 160), raw)
                buf = io.BytesIO(); img.save(buf, "PNG")
                with self.lock: self.frame_png = buf.getvalue(); self.frame_seq += 1
            elif line.startswith("STATE "):
                try: st = json.loads(line[6:])
                except ValueError: continue
                with self.lock: self.state = st
            elif line.startswith("UART "):
                with self.lock:
                    self.uart.append(line[5:])
                    if line[5:].startswith("STATE=") or line[5:].startswith("DIAG "): self.fw_line = line[5:]
            elif line.startswith("LOG "):
                with self.lock: self.log.append(line[4:])
        with self.lock: self.log.append("[live] simulator exited")

    def send(self, cmd):
        if cmd.strip() == "reboot": self.start(); return
        with self.lock:
            if self.proc and self.proc.poll() is None:
                self.proc.stdin.write(cmd.strip() + "\n"); self.proc.stdin.flush()

    def snapshot(self):
        with self.lock:
            return {"state": self.state, "fw": self.fw_line, "frame_seq": self.frame_seq,
                    "uart": list(self.uart)[-60:], "log": list(self.log)[-80:]}

PAGE = r"""<!doctype html><html><head><meta charset="utf-8"><title>evse-fw live simulator</title>
<style>
 body{font-family:system-ui,sans-serif;background:#1e1e1e;color:#ddd;margin:0;padding:12px}
 .grid{display:grid;grid-template-columns:420px 360px 1fr;gap:16px}
 .panel{background:#2a2a2a;border-radius:8px;padding:12px}
 h3{margin:0 0 8px 0;font-size:14px;color:#9cf;text-transform:uppercase;letter-spacing:.5px}
 #screen{width:384px;height:480px;image-rendering:pixelated;background:#000;border:6px solid #333;border-radius:10px;display:block}
 button{background:#444;color:#eee;border:0;border-radius:6px;padding:8px 12px;margin:3px;cursor:pointer;font-size:13px}
 button:hover{background:#555} button.on{background:#2a7} button.red{background:#a33}
 input[type=number],input[type=text]{width:70px;background:#111;color:#eee;border:1px solid #555;border-radius:4px;padding:6px}
 input[type=text].wide{width:220px}
 pre{background:#111;color:#bbb;font-size:11px;height:260px;overflow:auto;padding:8px;margin:0;border-radius:6px;white-space:pre-wrap}
 .kv{font-family:monospace;font-size:12px;line-height:1.5}
 .row{margin:6px 0}
 .big{font-size:16px;padding:14px 20px;background:#355}
</style></head><body>
<div class="grid">
 <div class="panel"><h3>Front panel</h3>
  <img id="screen" src="/frame.png">
  <div class="row"><button class="big" onclick="cmd('btn')">&#9679; Front button (10 &rarr; 16 &rarr; 24 &rarr; AUTO)</button></div>
  <div class="kv" id="fw"></div>
 </div>
 <div class="panel"><h3>Simulated world</h3>
  <div class="row"><b>EV</b>: <button id="b_plug" onclick="cmd('plug')">Plug</button><button id="b_unplug" onclick="cmd('unplug')">Unplug</button>
   <button id="b_ready1" onclick="cmd('ready 1')">Wants charge</button><button id="b_ready0" onclick="cmd('ready 0')">Charge complete</button></div>
  <div class="row"><b>Diode</b>: <button id="b_d1" onclick="cmd('diode 1')">OK</button><button id="b_d0" onclick="cmd('diode 0')">Missing</button></div>
  <div class="row"><b>NTC</b> counts: <input type="number" id="ntc" value="0" min="0" max="4095" step="50" onchange="cmd('ntc '+this.value)">
   <button onclick="setntc(1500)">cold</button><button onclick="setntc(3050)">throttle</button><button onclick="setntc(3400)">panic</button></div>
  <div class="row"><b>CP tau</b> &micro;s: <input type="number" id="tau" value="10" min="0" max="60" step="5" onchange="cmd('tau '+this.value)"></div>
  <div class="row"><b>Speed</b>: <button onclick="cmd('speed 0')">pause</button><button onclick="cmd('speed 1')">x1</button>
   <button onclick="cmd('speed 10')">x10</button><button onclick="cmd('speed 60')">x60</button>
   <button class="red" onclick="cmd('reboot')">Power cycle</button></div>
  <h3 style="margin-top:14px">ESP bridge (MQTT &rarr; UART)</h3>
  <div class="row">SET <input type="number" id="set" value="11" min="0" max="32" step="0.5"> A
   <button onclick="cmd('uart SET '+document.getElementById('set').value)">send</button></div>
  <div class="row"><button onclick="cmd('uart MODE AUTO')">MODE AUTO</button><button onclick="cmd('uart MODE MAN')">MODE MAN</button>
   <button onclick="cmd('uart ENABLE 1')">ENABLE 1</button><button onclick="cmd('uart ENABLE 0')">ENABLE 0</button></div>
  <div class="row">UART line: <input type="text" class="wide" id="raw" placeholder="OFFER 16 / CHARGE ON / SCAN" onkeydown="if(event.key=='Enter'){cmd('uart '+this.value);this.value=''}"></div>
  <div class="kv" id="st"></div>
 </div>
 <div class="panel"><h3>UART from the firmware</h3><pre id="uart"></pre>
  <h3 style="margin-top:12px">Events</h3><pre id="log"></pre></div>
</div>
<script>
let seq=-1;
function cmd(c){fetch('/cmd',{method:'POST',body:c});}
function setntc(v){document.getElementById('ntc').value=v;cmd('ntc '+v);}
function on(id,flag){document.getElementById(id).className=flag?'on':'';}
async function poll(){
  try{
    const r=await fetch('/state'); const j=await r.json(); const s=j.state;
    if(j.frame_seq!==seq){seq=j.frame_seq;document.getElementById('screen').src='/frame.png?'+seq;}
    if(s && s.t!==undefined){
      on('b_plug',s.plugged);on('b_unplug',!s.plugged);on('b_ready1',s.wants);on('b_ready0',!s.wants);on('b_d1',s.diode);on('b_d0',!s.diode);
      document.getElementById('st').innerHTML=
        `sim time ${s.t.toFixed(1)} s &nbsp; speed x${s.speed}<br>`+
        `EV: ${s.plugged?(s.S2?'C (S2 closed)':'B (connected)'):'A (none)'} &nbsp; I_ev = ${s.I} A<br>`+
        `CP: ${s.pwm?('PWM '+s.duty+' % = '+s.offer+' A'):'standby +12 V'}<br>`+
        `contactor: <b style="color:${s.contactor?'#4f4':'#f84'}">${s.contactor?'CLOSED':'open'}</b> &nbsp; NTC ${s.ntc}<br>`+
        `firmware: sel ${['10 A','16 A','24 A','AUTO'][s.sel]} / ${s.mode} / enabled ${s.enabled} / setpoint ${s.setpoint} A / latched ${s.latched} err ${s.err}`;
    }
    document.getElementById('fw').textContent=j.fw;
    document.getElementById('uart').textContent=j.uart.join('\n');
    document.getElementById('log').textContent=j.log.join('\n');
    for(const id of ['uart','log']){const e=document.getElementById(id);e.scrollTop=e.scrollHeight;}
  }catch(e){}
  setTimeout(poll,150);
}
poll();
</script></body></html>"""

def make_handler(sim):
    class H(BaseHTTPRequestHandler):
        def log_message(self, *a): pass
        def _send(self, code, ctype, body):
            self.send_response(code); self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body))); self.send_header("Cache-Control", "no-store")
            self.end_headers(); self.wfile.write(body)
        def do_GET(self):
            p = self.path.split("?")[0]
            if p == "/":            self._send(200, "text/html; charset=utf-8", PAGE.encode())
            elif p == "/state":     self._send(200, "application/json", json.dumps(sim.snapshot()).encode())
            elif p == "/frame.png":
                png = sim.frame_png
                if png is None:
                    buf = io.BytesIO(); Image.new("RGB", (128, 160)).save(buf, "PNG"); png = buf.getvalue()
                self._send(200, "image/png", png)
            else: self._send(404, "text/plain", b"not found")
        def do_POST(self):
            if self.path == "/cmd":
                n = int(self.headers.get("Content-Length", "0"))
                sim.send(self.rfile.read(n).decode()); self._send(200, "text/plain", b"ok")
            else: self._send(404, "text/plain", b"not found")
    return H

def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--diag", action="store_true", help="run the diagnostic build (simlive_diag)")
    ap.add_argument("--port", type=int, default=8765)
    a = ap.parse_args()
    binary = os.path.join(HERE, "simlive_diag" if a.diag else "simlive")
    if not os.path.exists(binary): sys.exit(f"{binary} missing: run tools/sim/build.sh first")
    sim = Sim(binary)
    srv = ThreadingHTTPServer(("127.0.0.1", a.port), make_handler(sim))
    print(f"evse-fw live simulator ({os.path.basename(binary)}): http://127.0.0.1:{a.port}  (Ctrl-C to stop)")
    try: srv.serve_forever()
    except KeyboardInterrupt: pass
    finally: sim.send("quit")

if __name__ == "__main__": main()
