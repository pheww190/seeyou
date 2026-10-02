#!/usr/bin/env python3
"""
csi_radar.py — PC side of the ESP32 CSI radio head.

Reads the raw binary stream the ESP32 pushes over USB serial, parses CSI /
scan / hello records, sanitises phase, computes sensing metrics and renders a
live 3D view (pygame + OpenGL).

  python3 csi_radar.py --port /dev/ttyACM0 --baud 921600   # live hardware
  python3 csi_radar.py --sim                               # synthetic data, no board
  python3 csi_radar.py --selftest                          # headless pipeline check

Deps:  pip install numpy pyserial pygame PyOpenGL

Honest scope reminder: one ESP32 = one antenna = no angle-of-arrival. This tool
visualises the *channel* (amplitude/phase/motion/range-of-dominant-path). It is
NOT a geometric 3D image of the room; that needs synthetic aperture (move the
node) or an array of nodes (tomography). The per-node pipeline here is the same
one every node in such an array would run.
"""

import argparse
import glob
import math
import os
import struct
import sys
import time

import numpy as np

# OpenGL is imported at module level (guarded) — `from X import *` is illegal
# inside a function, and we want --selftest to work without PyOpenGL installed.
try:
    from OpenGL.GL import *                      # noqa: F401,F403
    HAVE_GL = True
except Exception:                                # pragma: no cover
    HAVE_GL = False


def _perspective(fovy_deg, aspect, near, far):
    """gluPerspective replacement — avoids a GLU dependency."""
    top = near * math.tan(math.radians(fovy_deg) / 2.0)
    bottom = -top
    right = top * aspect
    left = -right
    glFrustum(left, right, bottom, top, near, far)


def _look_at(eye, center, up):
    """gluLookAt replacement — loads the view matrix directly."""
    eye = np.asarray(eye, np.float64)
    center = np.asarray(center, np.float64)
    up = np.asarray(up, np.float64)
    f = center - eye; f /= np.linalg.norm(f)
    s = np.cross(f, up); s /= np.linalg.norm(s)
    u = np.cross(s, f)
    m = np.array([
        s[0], u[0], -f[0], 0.0,
        s[1], u[1], -f[1], 0.0,
        s[2], u[2], -f[2], 0.0,
        -np.dot(s, eye), -np.dot(u, eye), np.dot(f, eye), 1.0,
    ], np.float32)
    glLoadMatrixf(m)

# ----------------------------------------------------------------------------
# wire format (must match esp32_csi_stream.ino)
# ----------------------------------------------------------------------------
MAGIC = b"CSI1"
T_CSI, T_SCAN, T_HELLO = 1, 2, 3
COMMON = 10          # magic(4) type(1) node(1) seq(4)


class Parser:
    """Incremental parser for the ESP32 binary stream."""

    def __init__(self):
        self.buf = bytearray()

    def feed(self, data: bytes):
        self.buf += data
        out = []
        while True:
            i = self.buf.find(MAGIC)
            if i < 0:
                if len(self.buf) > 3:
                    self.buf = self.buf[-3:]
                break
            if i:
                del self.buf[:i]
            if len(self.buf) < COMMON:
                break
            typ = self.buf[4]
            node = self.buf[5]
            seq = struct.unpack_from("<I", self.buf, 6)[0]

            if typ == T_CSI:
                if len(self.buf) < 21:
                    break
                rssi = struct.unpack_from("<b", self.buf, 10)[0]
                ch, sec = self.buf[11], self.buf[12]
                ln = struct.unpack_from("<H", self.buf, 13)[0]
                mac = bytes(self.buf[15:21])
                if len(self.buf) < 21 + ln:
                    break
                payload = bytes(self.buf[21:21 + ln])
                out.append(dict(type="csi", node=node, seq=seq, rssi=rssi,
                                ch=ch, sec=sec, mac=mac, data=payload))
                del self.buf[:21 + ln]

            elif typ == T_HELLO:
                if len(self.buf) < 14:
                    break
                ch = self.buf[10]
                nsc = struct.unpack_from("<H", self.buf, 11)[0]
                fl = self.buf[13]
                if len(self.buf) < 14 + fl:
                    break
                fw = bytes(self.buf[14:14 + fl]).decode("ascii", "replace")
                out.append(dict(type="hello", node=node, ch=ch, nsc=nsc, fw=fw))
                del self.buf[:14 + fl]

            elif typ == T_SCAN:
                if len(self.buf) < 11:
                    break
                cnt = self.buf[10]
                p, aps, ok = 11, [], True
                for _ in range(cnt):
                    if len(self.buf) < p + 9:
                        ok = False
                        break
                    a_rssi = struct.unpack_from("<b", self.buf, p)[0]
                    a_ch = self.buf[p + 1]
                    bssid = bytes(self.buf[p + 2:p + 8])
                    sl = self.buf[p + 8]
                    if len(self.buf) < p + 9 + sl:
                        ok = False
                        break
                    ssid = bytes(self.buf[p + 9:p + 9 + sl]).decode("utf-8", "replace")
                    aps.append(dict(rssi=a_rssi, ch=a_ch, bssid=bssid, ssid=ssid))
                    p += 9 + sl
                if not ok:
                    break
                out.append(dict(type="scan", node=node, aps=aps))
                del self.buf[:p]

            else:                                  # unknown → skip the magic
                del self.buf[:4]
        return out


# ----------------------------------------------------------------------------
# signal processing
# ----------------------------------------------------------------------------
def csi_to_iq(data: bytes) -> np.ndarray:
    a = np.frombuffer(data, dtype=np.int8).astype(np.float32)
    n = len(a) // 2
    return a[0:2 * n:2] + 1j * a[1:2 * n:2]


def sanitize_phase(ph: np.ndarray) -> np.ndarray:
    """Remove the CFO/SFO/PDD linear ramp so multipath structure shows."""
    ph = np.unwrap(ph)
    k = np.arange(len(ph), dtype=np.float32)
    A = np.vstack([k, np.ones_like(k)]).T
    coef, *_ = np.linalg.lstsq(A, ph, rcond=None)
    return ph - (coef[0] * k + coef[1])


def phase_slope_range(iq: np.ndarray, bw_hz: float = 20e6) -> float:
    """Coarse range of the dominant path from the phase-vs-subcarrier slope."""
    ph = np.unwrap(np.angle(iq))
    k = np.arange(len(ph))
    slope = np.polyfit(k, ph, 1)[0]           # rad per subcarrier
    df = bw_hz / max(1, len(iq))              # subcarrier spacing
    tau = slope / (2 * math.pi * df)          # group delay (s)
    return max(0.0, tau * 3e8)                # metres (free-space, optimistic)


class CsiState:
    """Rolling history of CSI for one node."""

    def __init__(self, nsc=64, hist=180):
        self.nsc = nsc
        self.hist = hist
        self.amp = np.zeros((hist, nsc), np.float32)
        self.phase = np.zeros((hist, nsc), np.float32)
        self.rssi = np.zeros(hist, np.float32)
        self.count = 0
        self.motion = 0.0
        self.range_m = 0.0
        self.last_iq = None
        self.prev_by_mac = {}       # mac -> last amplitude vector (per-source motion)
        self.sources = set()        # macs heard

    def push(self, iq: np.ndarray, rssi: int, mac: bytes = b""):
        n = min(len(iq), self.nsc)
        if len(iq) != self.nsc:                 # keep width fixed
            pad = np.zeros(self.nsc, np.complex64)
            pad[:n] = iq[:n]
            iq = pad
        a = np.abs(iq).astype(np.float32)
        ph = sanitize_phase(np.angle(iq[:n]))

        self.amp = np.roll(self.amp, -1, axis=0)
        self.phase = np.roll(self.phase, -1, axis=0)
        self.rssi = np.roll(self.rssi, -1)
        self.amp[-1, :n] = a[:n]
        self.phase[-1, :n] = ph
        self.rssi[-1] = rssi

        # motion is measured per source (per transmitter MAC) and averaged, so a
        # change anywhere in the airspace — any AP or device — shows up.
        key = bytes(mac)
        self.sources.add(key)
        prev = self.prev_by_mac.get(key)
        if prev is not None and len(prev) == n:
            d = float(np.mean(np.abs(a[:n] - prev)))
            self.motion = 0.9 * self.motion + 0.1 * d
        self.prev_by_mac[key] = a[:n].copy()

        self.last_iq = iq[:n].copy()
        self.range_m = phase_slope_range(iq[:n])
        self.count += 1


# ----------------------------------------------------------------------------
# synthetic CSI (for --sim / --selftest)
# ----------------------------------------------------------------------------
def synth_iq(t: float, nsc: int, rng: np.random.Generator) -> np.ndarray:
    k = np.arange(nsc)
    base = 0.6 + 0.35 * np.sin(0.25 * k) + 0.20 * np.cos(0.60 * k + 1.0)
    moving = 0.30 * np.sin(0.9 * k + 0.15 * t) * (0.5 + 0.5 * np.sin(0.02 * t))
    mag = np.clip(base + moving, 0.05, 1.5)
    ph = 0.35 * k + 2.0 * np.sin(0.3 * k - 0.05 * t)
    iq = mag * np.exp(1j * ph) * 80.0                    # scale into int8 range
    iq *= np.exp(1j * rng.uniform(-math.pi, math.pi))    # random per-packet phase
    iq += rng.normal(0, 4.0, nsc) + 1j * rng.normal(0, 4.0, nsc)
    return iq


def colormap_lut() -> np.ndarray:
    """256x3 uint8 lookup table: blue -> cyan -> green -> yellow -> red."""
    stops = np.array([0.0, 0.3, 0.6, 0.8, 1.0])
    cols = np.array([[13, 26, 102], [0, 179, 230], [26, 230, 77],
                     [242, 217, 26], [255, 51, 38]], float)
    x = np.linspace(0, 1, 256)
    lut = np.stack([np.interp(x, stops, cols[:, c]) for c in range(3)], axis=1)
    return lut.astype(np.uint8)


def pack_csi(node, seq, rssi, ch, sec, mac, iq) -> bytes:
    i = np.clip(iq.real, -127, 127).astype(np.int8)
    q = np.clip(iq.imag, -127, 127).astype(np.int8)
    payload = np.empty(len(i) * 2, np.int8)
    payload[0::2] = i
    payload[1::2] = q
    b = bytearray(MAGIC)
    b += struct.pack("<BB", T_CSI, node)
    b += struct.pack("<I", seq)
    b += struct.pack("<b", rssi)
    b += struct.pack("<BB", ch, sec)
    b += struct.pack("<H", len(payload))
    b += mac
    b += payload.tobytes()
    return bytes(b)


# ----------------------------------------------------------------------------
# sources
# ----------------------------------------------------------------------------
DEFAULT_PORT = "/dev/ttyUSB0"


def resolve_port(requested: str) -> str:
    """Pick the ESP32's serial device.

    An existing path wins. The default port (or 'auto') that isn't present falls
    back to the first /dev/ttyUSB*, then /dev/ttyACM*. An explicitly named path
    that is missing is honoured as-is, so you get a clear error rather than a
    silent switch to a different device.
    """
    requested = requested or DEFAULT_PORT
    if os.path.exists(requested):
        return requested
    if requested in ("auto", DEFAULT_PORT):
        cands = sorted(glob.glob("/dev/ttyUSB*")) + sorted(glob.glob("/dev/ttyACM*"))
        if cands:
            return cands[0]
        return DEFAULT_PORT
    return requested


def source_serial(port, baud):
    import serial
    try:
        ser = serial.Serial(port, baud, timeout=0.05)
    except serial.SerialException as e:
        sys.stderr.write(
            f"cannot open {port}: {e}\n"
            "  - attached into WSL?   usbipd list ; usbipd attach --wsl --busid <ID>\n"
            f"  - permission?          sudo chmod 666 {port}\n"
            "  - different device?    try --port /dev/ttyACM0\n")
        return
    sys.stderr.write(f"reading {port} @ {baud} baud\n")
    while True:
        chunk = ser.read(4096)
        if chunk:
            yield chunk


def source_sim(fps=60, nsc=64, seed=1):
    rng = np.random.default_rng(seed)
    t = 0.0
    seq = 0
    # several fake transmitters on their own channels, to mimic channel hopping
    macs = [bytes([0xde, 0xad, 0xbe, 0xef, 0x00, i]) for i in range(4)]
    chans = [1, 6, 11, 1]
    while True:
        t += 1.0 / fps
        for _ in range(3):                    # a few frames per tick
            seq += 1
            i = seq % len(macs)
            rssi = int(-52 + 6 * math.sin(0.05 * t + i) + rng.normal(0, 1.5))
            iq = synth_iq(t + i * 3.0, nsc, rng)
            yield pack_csi(1, seq, rssi, chans[i], 0, macs[i], iq)
        time.sleep(1.0 / fps)


# ----------------------------------------------------------------------------
# headless self-test
# ----------------------------------------------------------------------------
def run_selftest():
    rng = np.random.default_rng(7)
    parser = Parser()
    state = CsiState(nsc=64, hist=120)
    got = 0
    for t in range(400):
        iq = synth_iq(t * 0.02, 64, rng)
        frame = pack_csi(1, t, -50, 6, 0, b"\xaa\xbb\xcc\xdd\xee\xff", iq)
        for rec in parser.feed(frame):
            if rec["type"] == "csi":
                state.push(csi_to_iq(rec["data"]), rec["rssi"], rec.get("mac", b""))
                got += 1
    # also check the parser resyncs across garbage and split reads
    parser2 = Parser()
    f_a = pack_csi(1, 1, -60, 6, 0, b"\x01" * 6, synth_iq(1, 64, rng))
    f_b = pack_csi(1, 2, -60, 6, 0, b"\x01" * 6, synth_iq(2, 64, rng))
    assert len(parser2.feed(b"\x00\x11garbage" + f_a)) == 1, "resync failed"
    assert len(parser2.feed(f_b[:10])) == 0, "premature parse on partial header"
    assert len(parser2.feed(f_b[10:])) == 1, "split read failed"

    amp_mean = float(state.amp[-1].mean())
    mot = state.motion
    rng_m = state.range_m
    assert got == 400, f"parsed {got}/400"
    assert np.isfinite(amp_mean) and amp_mean > 0, "bad amplitude"
    assert state.phase.shape == (120, 64), "phase buffer shape"
    assert np.all(np.isfinite(state.amp)), "non-finite amplitude"

    print("SELFTEST OK")
    print(f"  frames parsed : {got}")
    print(f"  mean |H|      : {amp_mean:.3f}")
    print(f"  motion metric : {mot:.4f}")
    print(f"  range (dom.)  : {rng_m:.2f} m")
    print(f"  rssi last     : {state.rssi[-1]:.0f} dBm")
    return 0


# ----------------------------------------------------------------------------
# GUI  (pygame + OpenGL)
# ----------------------------------------------------------------------------
def run_gui(args):
    import pygame
    from pygame.locals import (DOUBLEBUF, OPENGL, QUIT, KEYDOWN, K_ESCAPE,
                               K_SPACE, RESIZABLE)

    if not HAVE_GL:
        sys.stderr.write("PyOpenGL is required for the display. "
                         "Install it:  pip install PyOpenGL PyOpenGL_accelerate\n")
        return 2

    pygame.init()
    W, H = 1280, 720
    try:
        pygame.display.set_mode((W, H), DOUBLEBUF | OPENGL | RESIZABLE)
    except pygame.error as e:
        sys.stderr.write(
            f"cannot open a window: {e}\n"
            "  WSL needs WSLg (Windows 11) or an X server with DISPLAY set.\n"
            "  headless check:  python3 csi_radar.py --selftest\n")
        return 2
    pygame.display.set_caption("ESP32 CSI Radar")

    font = pygame.font.SysFont("monospace", 14)
    big = pygame.font.SysFont("monospace", 18, bold=True)

    glEnable(GL_DEPTH_TEST)
    glEnable(GL_BLEND)
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)

    state = CsiState(nsc=args.nsc, hist=180)
    parser = Parser()
    node_ch = {"ch": "?", "fw": "?"}
    aps = []

    if args.sim:
        src = iter(source_sim())
    else:
        args.port = resolve_port(args.port)
        src = iter(source_serial(args.port, args.baud))

    heat_tex = glGenTextures(1)
    LUT = colormap_lut()
    text_cache = {}
    angle = 0.0
    auto = True
    paused = False
    last = time.time()
    fps = 0.0

    def text_tex(s, color=(200, 230, 255), f=None):
        key = (s, color)
        if key in text_cache:
            return text_cache[key]
        surf = (f or font).render(s, True, color)
        w, h = surf.get_size()
        data = pygame.image.tostring(surf, "RGBA", True)
        tid = glGenTextures(1)
        glBindTexture(GL_TEXTURE_2D, tid)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR)
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, w, h, 0, GL_RGBA, GL_UNSIGNED_BYTE, data)
        text_cache[key] = (tid, w, h)
        return tid, w, h

    def draw_text(s, x, y, color=(200, 230, 255), f=None):
        tid, w, h = text_tex(s, color, f)
        glEnable(GL_TEXTURE_2D)
        glBindTexture(GL_TEXTURE_2D, tid)
        glColor4f(1, 1, 1, 1)
        glBegin(GL_QUADS)
        glTexCoord2f(0, 0); glVertex2f(x, y)
        glTexCoord2f(1, 0); glVertex2f(x + w, y)
        glTexCoord2f(1, 1); glVertex2f(x + w, y + h)
        glTexCoord2f(0, 1); glVertex2f(x, y + h)
        glEnd()
        glDisable(GL_TEXTURE_2D)

    def amp_color(v):
        # blue → cyan → green → yellow → red
        v = max(0.0, min(1.0, v))
        stops = [(0.0, (0.05, 0.1, 0.4)), (0.3, (0.0, 0.7, 0.9)),
                 (0.6, (0.1, 0.9, 0.3)), (0.8, (0.95, 0.85, 0.1)), (1.0, (1.0, 0.2, 0.15))]
        for i in range(len(stops) - 1):
            a, ca = stops[i]; b, cb = stops[i + 1]
            if v <= b:
                t = (v - a) / (b - a + 1e-9)
                return tuple(ca[j] + (cb[j] - ca[j]) * t for j in range(3))
        return stops[-1][1]

    running = True
    frames = 0
    while running:
        for ev in pygame.event.get():
            if ev.type == QUIT:
                running = False
            elif ev.type == KEYDOWN:
                if ev.key == K_ESCAPE:
                    running = False
                elif ev.key == K_SPACE:
                    auto = not auto

        # --- pull + parse data ---
        if not paused:
            try:
                chunk = next(src)
                for rec in parser.feed(chunk):
                    if rec["type"] == "csi":
                        state.push(csi_to_iq(rec["data"]), rec["rssi"], rec.get("mac", b""))
                    elif rec["type"] == "hello":
                        node_ch["ch"] = rec["ch"]; node_ch["fw"] = rec["fw"]
                    elif rec["type"] == "scan":
                        aps = sorted(rec["aps"], key=lambda a: -a["rssi"])
            except StopIteration:
                running = False

        now = time.time()
        dt = now - last; last = now
        fps = 0.9 * fps + 0.1 * (1.0 / dt if dt > 0 else 0)
        if auto:
            angle += dt * 22.0

        # ------------------- 3D scene -------------------
        glViewport(0, 0, W, H)
        glClearColor(0.02, 0.03, 0.06, 1.0)
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

        glMatrixMode(GL_PROJECTION); glLoadIdentity()
        _perspective(45.0, W / H, 0.5, 400.0)
        glMatrixMode(GL_MODELVIEW); glLoadIdentity()
        _look_at((0, 62, 168), (0, 6, 0), (0, 1, 0))
        glRotatef(angle, 0, 1, 0)
        glTranslatef(-32, 0, 0)          # centre the subcarrier axis

        # amplitude waterfall surface: x=subcarrier, z=time(depth), y=amplitude
        rows = 48
        step = max(1, state.hist // rows)
        sub = state.amp[::step]
        R, C = sub.shape
        vmax = float(sub.max()) or 1.0
        for i in range(R - 1):
            glBegin(GL_QUADS)
            for j in range(C - 1):
                for (di, dj) in ((0, 0), (0, 1), (1, 1), (1, 0)):
                    v = sub[i + di, j + dj] / vmax
                    r, g, b = amp_color(v)
                    glColor3f(r, g, b)
                    glVertex3f(j + dj, v * 20.0, (i + di) * 1.5)
            glEnd()

        # baseline grid
        glColor4f(0.2, 0.4, 0.7, 0.5)
        glBegin(GL_LINES)
        for j in range(0, C, 8):
            glVertex3f(j, 0, 0); glVertex3f(j, 0, (R - 1) * 1.2)
        glEnd()

        # ------------------- 2D overlay -------------------
        glMatrixMode(GL_PROJECTION); glLoadIdentity()
        glOrtho(0, W, 0, H, -1, 1)
        glMatrixMode(GL_MODELVIEW); glLoadIdentity()
        glDisable(GL_DEPTH_TEST)

        # amplitude heatmap texture, right panel
        hm = state.amp
        vmax = float(hm.max()) or 1.0
        norm = np.clip(hm / vmax, 0, 1)
        img = LUT[(norm * 255).astype(np.uint8)]
        # uploaded as-is (row 0 = v 0 = bottom edge): oldest at bottom, newest on top
        glEnable(GL_TEXTURE_2D)
        glBindTexture(GL_TEXTURE_2D, heat_tex)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR)
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, img.shape[1], img.shape[0], 0,
                     GL_RGB, GL_UNSIGNED_BYTE, img.tobytes())
        px, py, pw, ph = W - 300, H - 320, 260, 180
        glColor4f(1, 1, 1, 1)
        glBegin(GL_QUADS)
        glTexCoord2f(0, 0); glVertex2f(px, py)
        glTexCoord2f(1, 0); glVertex2f(px + pw, py)
        glTexCoord2f(1, 1); glVertex2f(px + pw, py + ph)
        glTexCoord2f(0, 1); glVertex2f(px, py + ph)
        glEnd()
        glDisable(GL_TEXTURE_2D)

        # motion bar
        mv = min(1.0, state.motion * 4.0)
        glColor3f(0.15, 0.15, 0.2)
        glBegin(GL_QUADS)
        glVertex2f(px, py - 34); glVertex2f(px + pw, py - 34)
        glVertex2f(px + pw, py - 18); glVertex2f(px, py - 18)
        glEnd()
        r, g, b = amp_color(mv)
        glColor3f(r, g, b)
        glBegin(GL_QUADS)
        glVertex2f(px, py - 34); glVertex2f(px + pw * mv, py - 34)
        glVertex2f(px + pw * mv, py - 18); glVertex2f(px, py - 18)
        glEnd()

        # text HUD
        draw_text("ESP32 CSI RADAR", 20, H - 34, (90, 220, 255), big)
        draw_text(f"mode  : {'SIM' if args.sim else 'LIVE'}  {'PAUSED' if paused else ''}", 20, H - 70)
        draw_text(f"frames: {state.count}    fps {fps:4.0f}", 20, H - 88)
        draw_text(f"rssi  : {state.rssi[-1]:6.1f} dBm   ch {node_ch['ch']}", 20, H - 106)
        draw_text(f"motion: {state.motion:6.3f}   sources: {len(state.sources)}", 20, H - 124)
        draw_text(f"range : {state.range_m:6.2f} m  (dominant path, coarse)", 20, H - 142)
        draw_text(f"fw    : {node_ch['fw']}", 20, H - 160)
        draw_text("motion", px, py - 56)
        draw_text("amplitude waterfall  (subcarrier x time)", px, py + ph + 10)
        draw_text("space: pause spin   esc: quit", 20, 20)
        for n, ap in enumerate(aps[:8]):
            draw_text(f"AP {ap['rssi']:4d}dBm ch{ap['ch']:<2d} {ap['ssid'][:18]}",
                      20, 20 + 20 * (n + 1), (150, 200, 160))

        pygame.display.flip()
        glEnable(GL_DEPTH_TEST)

        frames += 1
        if args.frames and frames >= args.frames:
            if args.shot:
                data = glReadPixels(0, 0, W, H, GL_RGB, GL_UNSIGNED_BYTE)
                img = np.frombuffer(data, np.uint8).reshape(H, W, 3)[::-1]
                try:
                    from PIL import Image
                    Image.fromarray(img).save(args.shot)
                    print(f"saved {args.shot}")
                except Exception as e:                       # pragma: no cover
                    print(f"screenshot failed: {e}")
            running = False

    pygame.quit()
    return 0


# ----------------------------------------------------------------------------
def run_headless(args):
    """Read the stream and print stats — no window. Good for WSL / CI / a quick check."""
    try:
        sys.stdout.reconfigure(line_buffering=True)
    except Exception:
        pass
    parser = Parser()
    state = CsiState(nsc=args.nsc, hist=180)
    if args.sim:
        src = iter(source_sim())
        where = "sim"
    else:
        args.port = resolve_port(args.port)
        src = iter(source_serial(args.port, args.baud))
        where = args.port

    print(f"listening on {where} @ {args.baud} baud   (Ctrl-C to stop)")
    t0 = time.time()
    frames = 0
    last = t0
    try:
        for chunk in src:
            for rec in parser.feed(chunk):
                if rec["type"] == "csi":
                    state.push(csi_to_iq(rec["data"]), rec["rssi"], rec.get("mac", b""))
                    frames += 1
                elif rec["type"] == "hello":
                    print(f"  hello: fw={rec['fw']} ch={rec['ch']} nsc={rec['nsc']}")
                elif rec["type"] == "scan":
                    print(f"  scan: {len(rec['aps'])} APs")
            now = time.time()
            if now - last >= 1.0:
                fps = frames / max(1e-6, now - t0)
                print(f"frames={frames:<8d} fps={fps:6.1f}  rssi={state.rssi[-1]:6.1f} dBm  "
                      f"sources={len(state.sources):<3d} motion={state.motion:6.3f}  "
                      f"range={state.range_m:5.2f} m")
                last = now
    except KeyboardInterrupt:
        print("\nstopped")
    return 0


def main():
    ap = argparse.ArgumentParser(description="ESP32 CSI radar (PC side)")
    ap.add_argument("--port", default="/dev/ttyUSB0",
                    help="serial device (default /dev/ttyUSB0; 'auto' = first ttyUSB/ttyACM)")
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--nsc", type=int, default=64, help="subcarriers to display")
    ap.add_argument("--sim", action="store_true", help="use synthetic data (no board)")
    ap.add_argument("--selftest", action="store_true", help="headless pipeline check")
    ap.add_argument("--frames", type=int, default=0, help="render N frames then exit (0 = forever)")
    ap.add_argument("--shot", default="", help="save a PNG of the last frame")
    ap.add_argument("--headless", action="store_true", help="read + print stats, no window")
    args = ap.parse_args()
    if args.selftest:
        return run_selftest()
    if args.headless:
        return run_headless(args)
    return run_gui(args)


if __name__ == "__main__":
    sys.exit(main())
