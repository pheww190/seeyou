/*
 * ============================================================================
 *  ESP32 CSI RADIO HEAD  —  raw Wi-Fi Channel State Information over USB serial
 * ============================================================================
 *
 *  The ESP32 does ONE job: capture raw radio data and push it out of the USB
 *  port as compact binary. No HTTP, no HTML, no compute. All parsing, phase
 *  sanitisation, sensing and display happen on the PC side (csi_radar.py).
 *
 *  WHAT IT SENDS
 *    CSI frames  — for every received 802.11 frame: RSSI, channel, source MAC
 *                  and the raw per-subcarrier CSI buffer (int8 I/Q pairs).
 *    SCAN frames — periodically, a list of nearby APs (RSSI / channel / BSSID).
 *    HELLO frame — once at boot.
 *
 *  WIRE FORMAT (little-endian).  Every record starts with the 4-byte magic 'CSI1'
 *    Common header (10 bytes):
 *      magic[4] type(u8) node_id(u8) seq(u32)
 *    CSI   (type=1): rssi(i8) ch(u8) sec(u8) len(u16) mac[6]  +  len bytes I/Q
 *    SCAN  (type=2): count(u8)  then per AP: rssi(i8) ch(u8) bssid[6] slen(u8) ssid[slen]
 *    HELLO (type=3): ch(u8) nsc(u16) fwlen(u8) fw[fwlen]
 *
 *  NOTES / HONEST CAVEATS
 *    - CSI is only produced for frames the radio receives. We connect to an AP
 *      and (optionally) generate traffic so there is a steady stream.
 *    - CSI is channel-locked: you only "see" the channel you are tuned to.
 *    - Promiscuous mode + a connected STA coexist, but a scan briefly leaves
 *      promiscuous (that is why scans are infrequent).
 *    - Absolute phase is corrupted per packet by CFO/SFO/PDD. The PC side
 *      sanitises it. Do not trust raw phase.
 *
 *  BUILD: Arduino IDE / arduino-cli, ESP32 core 2.0.5+ (IDF 4.4) or 3.x (IDF 5.x).
 * ============================================================================
 */

#include <WiFi.h>
#include <WiFiUdp.h>
#include "esp_wifi.h"

// ------------------------------- configure ---------------------------------
static const char*    WIFI_SSID   = "YOUR_WIFI_NAME";      // 2.4 GHz network
static const char*    WIFI_PASS   = "YOUR_WIFI_PASSWORD";
static const uint8_t  NODE_ID     = 1;                     // give each node a unique id
static const uint32_t SERIAL_BAUD = 921600;
static const bool     TRAFFIC_GEN = true;                  // elicit CSI-bearing replies
static const bool     ENABLE_SCAN = true;                  // periodic AP scan frames
static const uint32_t SCAN_PERIOD_MS = 30000;
// ---------------------------------------------------------------------------

#define MAGIC0 'C'
#define MAGIC1 'S'
#define MAGIC2 'I'
#define MAGIC3 '1'
#define T_CSI   1
#define T_SCAN  2
#define T_HELLO 3

#define RB_N     8       // ISR ring depth (frames)
#define MAX_CSI  256     // max CSI bytes (40 MHz HT = 256)

struct CsiFrame {
  int16_t len;
  int8_t  rssi;
  uint8_t ch;
  uint8_t sec;
  uint8_t mac[6];
  int8_t  data[MAX_CSI];
};

static CsiFrame rb[RB_N];
static volatile int      rb_head = 0, rb_tail = 0;
static volatile uint32_t g_seq = 0;
static volatile uint32_t g_dropped = 0;

// ------------------------- ISR: copy CSI into the ring ----------------------
// Never Serial.write() inside an ISR — copy out, stream from loop().
void IRAM_ATTR csi_cb(void* ctx, wifi_csi_info_t* info) {
  if (!info || !info->buf) return;
  int next = (rb_head + 1) % RB_N;
  if (next == rb_tail) { g_dropped++; return; }          // ring full → drop oldest-new

  CsiFrame& f = rb[rb_head];
  int len = info->len;
  if (len > MAX_CSI) len = MAX_CSI;
  memcpy(f.data, info->buf, len);
  f.len  = (int16_t)len;
  f.rssi = (int8_t)info->rx_ctrl.rssi;
  f.ch   = (uint8_t)info->rx_ctrl.channel;
  f.sec  = (uint8_t)info->rx_ctrl.secondary_channel;
  memcpy(f.mac, info->mac, 6);
  rb_head = next;
}

// ------------------------------ writers ------------------------------------
static inline void w_u8(uint8_t v)  { Serial.write(v); }
static inline void w_u16(uint16_t v){ Serial.write((uint8_t)(v & 0xFF)); Serial.write((uint8_t)(v >> 8)); }
static inline void w_u32(uint32_t v){ for (int i = 0; i < 4; i++) Serial.write((uint8_t)((v >> (8 * i)) & 0xFF)); }

static void sendHeader(uint8_t type, uint32_t seq) {
  Serial.write(MAGIC0); Serial.write(MAGIC1); Serial.write(MAGIC2); Serial.write(MAGIC3);
  w_u8(type); w_u8(NODE_ID); w_u32(seq);
}

static void sendHello() {
  sendHeader(T_HELLO, g_seq++);
  w_u8((uint8_t)WiFi.channel());
  w_u16((uint16_t)64);                     // nominal subcarriers (20 MHz)
  const char* fw = "csi-head-1.0";
  uint8_t n = (uint8_t)strlen(fw);
  w_u8(n); Serial.write((const uint8_t*)fw, n);
}

static void sendCsi(CsiFrame& f) {
  sendHeader(T_CSI, g_seq++);
  w_u8((uint8_t)f.rssi);
  w_u8(f.ch); w_u8(f.sec);
  w_u16((uint16_t)f.len);
  Serial.write(f.mac, 6);
  Serial.write((const uint8_t*)f.data, f.len);
}

static void sendScan() {
  esp_wifi_set_promiscuous(false);         // scanning needs the radio to itself
  int n = WiFi.scanNetworks(false, true);
  if (n < 0) n = 0;
  sendHeader(T_SCAN, g_seq++);
  w_u8((uint8_t)(n < 255 ? n : 255));
  for (int i = 0; i < n && i < 255; i++) {
    w_u8((uint8_t)(int8_t)WiFi.RSSI(i));
    w_u8((uint8_t)WiFi.channel(i));
    uint8_t* b = WiFi.BSSID(i);
    Serial.write(b, 6);
    String s = WiFi.SSID(i);
    uint8_t sl = (uint8_t)(s.length() > 32 ? 32 : s.length());
    w_u8(sl); Serial.write((const uint8_t*)s.c_str(), sl);
  }
  WiFi.scanDelete();
  esp_wifi_set_promiscuous(true);
}

// ------------------------------- setup -------------------------------------
void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(300);

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 20000) delay(300);

  // capture every frame on the tuned channel
  wifi_promiscuous_filter_t filt;
  filt.filter_mask = WIFI_PROMIS_FILTER_MASK_ALL;
  esp_wifi_set_promiscuous_filter(&filt);
  esp_wifi_set_promiscuous(true);

  // CSI config + callback
  wifi_csi_config_t cfg;
  memset(&cfg, 0, sizeof(cfg));
  cfg.lltf_en          = true;
  cfg.htltf_en         = true;
  cfg.stbc_htltf2_en   = true;
  cfg.ltf_merge_en     = true;
  cfg.channel_filter_en = false;
  cfg.manu_scale       = false;
  esp_wifi_set_csi_config(&cfg);
  esp_wifi_set_csi_rx_cb(&csi_cb, NULL);
  esp_wifi_set_csi(true);

  sendHello();
}

// -------------------------------- loop -------------------------------------
static WiFiUDP udp;

void loop() {
  // stream whatever the ISR captured
  while (rb_tail != rb_head) {
    sendCsi(rb[rb_tail]);
    rb_tail = (rb_tail + 1) % RB_N;
  }

  // generate downlink traffic so CSI keeps flowing
  if (TRAFFIC_GEN) {
    static uint32_t lastTx = 0;
    if (millis() - lastTx > 10) {
      lastTx = millis();
      udp.beginPacket(WiFi.gatewayIP(), 9);      // discard port → elicits a reply
      udp.write((const uint8_t*)"csi", 3);
      udp.endPacket();
    }
  }

  // occasional AP scan
  if (ENABLE_SCAN) {
    static uint32_t lastScan = 0;
    if (millis() - lastScan > SCAN_PERIOD_MS) {
      lastScan = millis();
      sendScan();
    }
  }
}
