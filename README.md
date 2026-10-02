# seeyou

A thin **ESP32** (ESP-IDF) captures raw Wi-Fi **CSI** — Channel State
Information, i.e. per-subcarrier amplitude *and* phase — and streams it out of
the console UART as compact binary. A **Python app** on the host parses it,
sanitises the phase, computes sensing metrics and renders a live 3D view
(pygame + OpenGL).

```
 ┌────────────┐   raw binary over UART (921600)   ┌──────────────────────────────┐
 │   ESP32    │  CSI: RSSI, MAC, int8 I/Q pairs   │  host: csi_radar.py          │
 │ radio head │ ────────────────────────────────► │  parse → sanitise phase →    │
 │ (ESP-IDF)  │                                   │  sensing → pygame+OpenGL 3D  │
 └────────────┘                                   └──────────────────────────────┘
```

## Layout

```
seeyou/
├── CMakeLists.txt              ESP-IDF project root
├── sdkconfig.defaults          console 921600, quiet logs
├── main/
│   ├── CMakeLists.txt
│   ├── Kconfig.projbuild       menuconfig → "CSI radio head"
│   └── csi_head.c              firmware
└── csi_radar.py                host: parse + sensing + 3D display
```

## Radio modes

CSI is produced only for frames the radio actually **receives**, so the ESP32
needs (a) a channel to sit on and (b) traffic to hear. Two ways to get that:

- **Associate** (`CSI_ASSOCIATE=y`, default) — join a 2.4 GHz AP. The channel is
  set for you and there is a guaranteed stream (beacons + gateway pings). This is
  why the firmware asks for an SSID/password. The gateway-ping traffic generator
  lives here too.
- **Listen-only** (`CSI_ASSOCIATE=n`) — **no credentials, no association.** The
  radio parks on `CSI_LISTEN_CHANNEL` and captures whatever ambient frames
  appear on it (mostly AP beacons). Handy if you don't want the node on your
  network. Expect a lighter, burstier stream than the associated mode.

Either way the ESP32 is not a network client in the usual sense — nothing is
sent anywhere except the raw CSI out of the UART.

## Firmware — build & flash

```bash
idf.py set-target esp32            # or esp32s3 / esp32c3
idf.py menuconfig                  # -> "CSI radio head"
idf.py build flash monitor
```

| option | meaning |
|--------|---------|
| `CSI_ASSOCIATE` | on = join an AP; off = listen-only, no credentials |
| `CSI_WIFI_SSID` / `CSI_WIFI_PASSWORD` | only when associating |
| `CSI_LISTEN_CHANNEL` | channel to park on in listen-only mode |
| `CSI_NODE_ID` | stamped into every frame; unique per board |
| `CSI_TRAFFIC_GEN` | UDP pings to the gateway so CSI keeps flowing (associate mode) |
| `CSI_ENABLE_SCAN` / `CSI_SCAN_PERIOD_MS` | periodic AP-scan frames |

The console UART runs at **921600** (`sdkconfig.defaults`); open the host at the
same baud.

## Host — run (WSL)

```bash
# get the serial port into WSL first (usbipd-win on the Windows side):
#   usbipd list ; usbipd bind --busid <ID> ; usbipd attach --wsl --busid <ID>

pip install numpy pyserial pygame PyOpenGL
python3 csi_radar.py                 # defaults to /dev/ttyUSB0
python3 csi_radar.py --port auto     # first ttyUSB* / ttyACM*
python3 csi_radar.py --sim           # synthetic data, no board
python3 csi_radar.py --selftest      # headless pipeline check
```

The device is auto-detected: the default `/dev/ttyUSB0` (or `--port auto`) falls
back to the first `/dev/ttyUSB*`, then `/dev/ttyACM*` if it isn't present. An
explicitly named port that's missing is used as-is, so you get a clear error
rather than a silent switch. If it can't open the port it prints the `usbipd` /
permission fixes. The window needs **WSLg** (Windows 11) or an X server with
`DISPLAY` set.

Keys: `space` pause the spin, `esc` quit. Flags: `--baud`, `--nsc`, `--frames N`, `--shot out.png`.

## Wire format (little-endian)

Common header: `magic[4]='CSI1'` `type(u8)` `node(u8)` `seq(u32)`

| type | name  | body |
|------|-------|------|
| 1 | CSI   | `rssi(i8) ch(u8) sec(u8) len(u16) mac[6]` + `len` bytes of int8 I/Q |
| 2 | SCAN  | `count(u8)` then per AP: `rssi(i8) ch(u8) bssid[6] slen(u8) ssid[slen]` |
| 3 | HELLO | `ch(u8) nsc(u16) fwlen(u8) fw[fwlen]` |

## What the display shows

- **3D waterfall** — amplitude `|H[k]|` across subcarriers (x) and time (depth).
- **Heatmap panel** — the same amplitude as a 2D image.
- **Motion** — how fast the channel is changing.
- **Range** — coarse distance to the dominant path (phase-slope).
- **AP list** — nearby access points from the periodic scan.

## Honest scope

This is a **channel** instrument, not a camera. One ESP32 = one antenna = **no
angle-of-arrival**, so you get amplitude, phase, motion and a *coarse* range to
the dominant path — not a direction. Raw phase is corrupted per packet by
CFO/SFO/packet-detection-delay and is sanitised on the host before display.

Real imaging needs spatial diversity: **synthetic aperture** (move/rotate the
node and fuse frames) or an **array of nodes** (≥3–4) for RF tomography. Frames
already carry `node` + `bssid`, so adding nodes is additive on the host side —
flash the same firmware with a different `CSI_NODE_ID`.

## Notes

- CSI is channel-locked: you only see the channel you are tuned to.
- Scans briefly leave promiscuous mode; that is why the period is large.
- Logs share the UART — kept at WARN — and the host resyncs on the `CSI1` magic.
