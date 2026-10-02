# seeyou

A thin **ESP32** (ESP-IDF) captures raw Wi-Fi **CSI** — Channel State
Information, i.e. per-subcarrier amplitude *and* phase — and streams it out of
the console UART as compact binary. A **Python app** on the host parses it,
sanitises the phase, and renders a live 3D view (pygame + OpenGL).

**No access point required.** By default the firmware does not associate with
anything — it listens to the ambient radio traffic around it.

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

CSI is produced only for frames the radio **receives**, so the ESP32 needs a
channel to sit on and transmitters to hear. It never connects to anything by
default:

- **Listen-only** (`CSI_ASSOCIATE=n`, the default) — no credentials, no
  association. The radio **hops across channels** (`CSI_HOP`, default `1,6,11`)
  and captures frames from every transmitter it can hear: every nearby AP
  beacons several times a second, and those beacons are the signals we sense.
  This is "use all available signals".
- **Associate** (`CSI_ASSOCIATE=y`) — join a 2.4 GHz AP. Optional: it guarantees
  a steady stream and sets the channel for you, at the cost of needing
  credentials and sitting on one channel.

## Firmware — build & flash

```bash
idf.py set-target esp32            # or esp32s3 / esp32c3
idf.py menuconfig                  # -> "CSI radio head"
idf.py build flash monitor
```

| option | meaning |
|--------|---------|
| `CSI_ASSOCIATE` | off (default) = listen-only; on = join an AP |
| `CSI_HOP` / `CSI_HOP_CHANNELS` / `CSI_HOP_DWELL_MS` | channel hopping (listen-only) |
| `CSI_LISTEN_CHANNEL` | fixed channel, when hopping is off |
| `CSI_WIFI_SSID` / `CSI_WIFI_PASSWORD` | only when associating |
| `CSI_NODE_ID` | stamped into every frame; unique per board |
| `CSI_TRAFFIC_GEN` | UDP pings to the gateway (associate mode) |
| `CSI_ENABLE_SCAN` / `CSI_SCAN_PERIOD_MS` | periodic AP-scan frames |

**One required option.** Wi-Fi CSI is **off by default** in ESP-IDF, so the build
needs `CONFIG_ESP_WIFI_CSI_ENABLED=y` (menuconfig → Component config → Wi-Fi →
*WiFi CSI (Channel State Information)*). It is already set in
`sdkconfig.defaults`, so a plain `idf.py build` picks it up. On IDF 4.x the symbol
was `CONFIG_ESP32_WIFI_CSI_ENABLED`.

**Targets.** `esp32` (reference), `esp32s3`, `esp32c3` all support CSI; `esp32c2`
does not (`SOC_WIFI_CSI_SUPPORT=n`). CSI also requires **promiscuous mode**, which
the firmware enables.

The console UART runs at **921600** (`sdkconfig.defaults`); open the host at the
same baud.

## Host — run (WSL)

```bash
# get the serial port into WSL first (usbipd-win on the Windows side):
#   usbipd list ; usbipd bind --busid <ID> ; usbipd attach --wsl --busid <ID>

pip install numpy pyserial pygame PyOpenGL
python3 csi_radar.py                 # defaults to /dev/ttyUSB0
python3 csi_radar.py --port auto     # first ttyUSB* / ttyACM*
python3 csi_radar.py --headless      # live stats to the terminal, no window
python3 csi_radar.py --sim           # synthetic data, no board
python3 csi_radar.py --selftest      # offline pipeline check
```

The default `/dev/ttyUSB0` (or `--port auto`) falls back to the first
`/dev/ttyUSB*`, then `/dev/ttyACM*` if it isn't present. An explicitly named port
that's missing is used as-is, so you get a clear error rather than a silent
switch. If it can't open the port it prints the `usbipd` / permission fixes. The
window needs **WSLg** (Windows 11) or an X server with `DISPLAY` set.

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
- **Heatmap panel** — the same amplitude as a 2D image (oldest at the bottom, newest on top).
- **Motion** — measured **per transmitter MAC** and averaged, so a change from
  *any* source in the airspace shows up. This is the movement detector.
- **Sources** — how many distinct transmitters have been heard.
- **Range** — coarse distance to the dominant path (phase-slope).
- **AP list** — nearby access points from the periodic scan.

## Honest scope

This is a **channel** instrument, not a camera. One ESP32 = one antenna = **no
angle-of-arrival**, so you get amplitude, phase, motion and a *coarse* range to
the dominant path — not a direction. Raw phase is corrupted per packet by
CFO/SFO/packet-detection-delay and is sanitised on the host before display.

Movement is the thing a single node does genuinely well: perturb the multipath
and the per-source motion metric jumps. It cannot say *where* or *who*, and it
cannot detect a still person. Real imaging needs spatial diversity: **synthetic
aperture** (move/rotate the node) or an **array of nodes** (≥3–4) for RF
tomography. Frames already carry `node` + `bssid`, so adding nodes is additive
on the host side — flash the same firmware with a different `CSI_NODE_ID`.

## Notes

- CSI is channel-locked: hopping is how we cover more than one channel. Expect
  burstier per-source updates while hopping than on a single fixed channel.
- Scans briefly leave promiscuous mode; the hop task pauses during a scan.
- Logs share the UART — kept at WARN — and the host resyncs on the `CSI1` magic.

## Troubleshooting

- **`esp_wifi_set_csi_config` returns `ESP_FAIL`, board aborts at boot.** Wi-Fi
  CSI is disabled in `sdkconfig`. Enable `CONFIG_ESP_WIFI_CSI_ENABLED` in
  menuconfig, or delete `sdkconfig` so `sdkconfig.defaults` re-applies — it is
  only loaded when `sdkconfig` does not already exist. The firmware now fails at
  *build* time with a clear `#error` if the option is off.
- **`Detected size(4096k) larger than the size in the binary image header(2048k)`.**
  Set Flash size to 4 MB (Serial flasher config → Flash size).
- **Non-generic flash chip warning.** Optional: enable the matching
  `SPI_FLASH_SUPPORT_*_CHIP` in menuconfig.
- **`ESP_ROM_ELF_DIR` not defined / gdbinit warning.** Harmless — it only affects
  ROM symbols in GDB. Source the IDF export script to silence it.
- **Console stays at 115200 despite `CONFIG_ESP_CONSOLE_UART_BAUDRATE=921600`.**
  The baud option is only honoured when `CONFIG_ESP_CONSOLE_UART_CUSTOM=y`. Both
  lines are in `sdkconfig.defaults`; if you build from an old `sdkconfig`, delete
  it so they re-apply. Run the host at whatever the monitor reports
  (`idf.py monitor` prints the baud in its banner).
- **`idf.py monitor` crashes with `KeyError: 0`.** Expected — the CSI stream is
  binary and the monitor tries to parse it as log lines. Don't use the monitor
  for this project; use `csi_radar.py`. Close the monitor (`Ctrl+]`) before
  starting the host, it holds the port.
