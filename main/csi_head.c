/*
 * ============================================================================
 *  ESP32 CSI RADIO HEAD  —  ESP-IDF
 * ============================================================================
 *
 *  A thin radio head: capture raw Wi-Fi Channel State Information and push it
 *  out of the console UART as compact binary. No networking stack of our own,
 *  no HTML, no compute — the host (csi_radar.py) parses, sanitises and renders.
 *
 *  RADIO MODES (menuconfig -> "CSI radio head"):
 *    - Listen-only (CSI_ASSOCIATE=n, the default): no credentials, no
 *      association. The radio hops across channels (CSI_HOP) and captures
 *      ambient frames from every transmitter it can hear — all nearby APs.
 *    - Associate (CSI_ASSOCIATE=y): join a 2.4 GHz AP for a guaranteed steady
 *      stream. Optional; not needed for sensing.
 *
 *  WIRE FORMAT (little-endian). Every record begins with the 4-byte magic 'CSI1'
 *    Common header (10 bytes):
 *      magic[4] type(u8) node_id(u8) seq(u32)
 *    CSI   (type=1): rssi(i8) ch(u8) sec(u8) len(u16) mac[6]  +  len bytes I/Q
 *    SCAN  (type=2): count(u8)  then per AP: rssi(i8) ch(u8) bssid[6] slen(u8) ssid[slen]
 *    HELLO (type=3): ch(u8) nsc(u16) fwlen(u8) fw[fwlen]
 *
 *  BUILD (terminal):
 *      idf.py set-target esp32          # or esp32s3 / esp32c3 ...
 *      idf.py menuconfig                # -> "CSI radio head"
 *      idf.py build flash monitor
 *
 *  The host reads the same UART at CONFIG_ESP_CONSOLE_UART_BAUDRATE (921600).
 *
 *  NOTES
 *    - CSI is channel-locked; hopping is how we cover more than one channel.
 *    - Logs share this UART; they are kept quiet and the host resyncs on magic.
 *    - Absolute phase is corrupted per packet (CFO/SFO/PDD); sanitised on the host.
 * ============================================================================
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "nvs_flash.h"

#include "lwip/sockets.h"
#include "lwip/inet.h"

static const char *TAG = "csi_head";

#define MAGIC0 'C'
#define MAGIC1 'S'
#define MAGIC2 'I'
#define MAGIC3 '1'
#define T_CSI   1
#define T_SCAN  2
#define T_HELLO 3

#define RB_N    8        /* ISR ring depth (frames)            */
#define MAX_CSI 256      /* max CSI bytes (40 MHz HT = 256)    */
#define MAX_AP  32       /* max APs per scan frame             */
#define TXBUF   1024     /* staging buffer for the UART writes */
#define MAX_HOP 32       /* max channels in the hop list       */

/* ---------------------------- capture ring -------------------------------- */
typedef struct {
    int16_t len;
    int8_t  rssi;
    uint8_t ch;
    uint8_t sec;
    uint8_t mac[6];
    int8_t  data[MAX_CSI];
} csi_frame_t;

static csi_frame_t       s_rb[RB_N];
static volatile int      s_head = 0, s_tail = 0;
static volatile uint32_t s_seq = 0;
static volatile uint32_t s_dropped = 0;
static volatile bool     s_scanning = false;   /* pause hopping during a scan */

/* Copy CSI in the ISR; never block or write to the UART from here. */
static void IRAM_ATTR csi_rx_cb(void *ctx, wifi_csi_info_t *info)
{
    if (!info || !info->buf) {
        return;
    }
    int next = (s_head + 1) % RB_N;
    if (next == s_tail) {            /* ring full → drop */
        s_dropped++;
        return;
    }
    csi_frame_t *f = &s_rb[s_head];
    int len = info->len;
    if (len > MAX_CSI) {
        len = MAX_CSI;
    }
    memcpy(f->data, info->buf, len);
    f->len  = (int16_t)len;
    f->rssi = (int8_t)info->rx_ctrl.rssi;
    f->ch   = (uint8_t)info->rx_ctrl.channel;
    f->sec  = (uint8_t)info->rx_ctrl.secondary_channel;
    memcpy(f->mac, info->mac, 6);
    s_head = next;
}

/* --------------------------- binary writer -------------------------------- */
static uint8_t s_tx[TXBUF];
static size_t  s_txn = 0;

static inline void tx_flush(void)            { if (s_txn) { fwrite(s_tx, 1, s_txn, stdout); s_txn = 0; } }
static inline void tx_need(size_t n)         { if (s_txn + n > TXBUF) { tx_flush(); } }
static inline void tx_u8(uint8_t v)          { s_tx[s_txn++] = v; }
static inline void tx_u16(uint16_t v)        { s_tx[s_txn++] = (uint8_t)v; s_tx[s_txn++] = (uint8_t)(v >> 8); }
static inline void tx_u32(uint32_t v)        { for (int i = 0; i < 4; i++) { s_tx[s_txn++] = (uint8_t)(v >> (8 * i)); } }
static inline void tx_bytes(const void *p, size_t n) { memcpy(s_tx + s_txn, p, n); s_txn += n; }

static void tx_header(uint8_t type, uint32_t seq)
{
    tx_need(10);
    tx_u8(MAGIC0); tx_u8(MAGIC1); tx_u8(MAGIC2); tx_u8(MAGIC3);
    tx_u8(type); tx_u8(CONFIG_CSI_NODE_ID); tx_u32(seq);
}

static void send_hello(void)
{
    uint8_t prim = 0;
    wifi_second_chan_t sec = WIFI_SECOND_CHAN_NONE;
    esp_wifi_get_channel(&prim, &sec);

    tx_header(T_HELLO, s_seq++);
    tx_u8(prim);
    tx_u16(64);                       /* nominal subcarriers (20 MHz) */
    const char *fw = "csi-idf-1.2";
    uint8_t n = (uint8_t)strlen(fw);
    tx_u8(n);
    tx_bytes(fw, n);
    tx_flush();
}

static void send_csi(csi_frame_t *f)
{
    tx_need(21 + f->len);
    tx_header(T_CSI, s_seq++);
    tx_u8((uint8_t)f->rssi);
    tx_u8(f->ch);
    tx_u8(f->sec);
    tx_u16((uint16_t)f->len);
    tx_bytes(f->mac, 6);
    tx_bytes(f->data, f->len);
}

/* ------------------------------- tasks ------------------------------------ */
static void stream_task(void *arg)
{
    while (1) {
        while (s_tail != s_head) {
            send_csi(&s_rb[s_tail]);
            s_tail = (s_tail + 1) % RB_N;
        }
        tx_flush();
        vTaskDelay(1);               /* 1 ms tick */
    }
}

#if CONFIG_CSI_TRAFFIC_GEN
static void traffic_task(void *arg)
{
    int s = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (s < 0) {
        ESP_LOGW(TAG, "udp socket failed");
        vTaskDelete(NULL);
        return;
    }
    esp_netif_ip_info_t ip;
    if (esp_netif_get_ip_info(esp_netif_get_handle_from_ifkey("WIFI_STA_DEF"), &ip) != ESP_OK) {
        ESP_LOGW(TAG, "no ip info");
        close(s);
        vTaskDelete(NULL);
        return;
    }
    struct sockaddr_in dst;
    memset(&dst, 0, sizeof(dst));
    dst.sin_family = AF_INET;
    dst.sin_port = htons(9);         /* discard port → elicits an ICMP reply */
    dst.sin_addr.s_addr = ip.gw.addr;

    while (1) {
        sendto(s, "csi", 3, 0, (struct sockaddr *)&dst, sizeof(dst));
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
#endif

/* ------------------------- channel hopping (listen-only) ------------------ */
#if !CONFIG_CSI_ASSOCIATE && CONFIG_CSI_HOP
static uint8_t s_hop[MAX_HOP];
static int     s_hop_n = 0;

/* Parse "1,6,11" or "1-13" (or a mix) into s_hop[]. */
static void parse_channels(void)
{
    const char *s = CONFIG_CSI_HOP_CHANNELS;
    while (*s && s_hop_n < MAX_HOP) {
        while (*s == ' ' || *s == ',') {
            s++;
        }
        if (!*s) {
            break;
        }
        if (*s >= '0' && *s <= '9') {
            int a = 0;
            while (*s >= '0' && *s <= '9') {
                a = a * 10 + (*s - '0');
                s++;
            }
            if (*s == '-') {
                s++;
                int b = 0;
                while (*s >= '0' && *s <= '9') {
                    b = b * 10 + (*s - '0');
                    s++;
                }
                for (int c = a; c <= b && s_hop_n < MAX_HOP; c++) {
                    s_hop[s_hop_n++] = (uint8_t)c;
                }
            } else if (a > 0 && a <= 14) {
                s_hop[s_hop_n++] = (uint8_t)a;
            }
        } else {
            s++;
        }
    }
    if (s_hop_n == 0) {              /* sane fallback */
        s_hop[s_hop_n++] = 1;
        s_hop[s_hop_n++] = 6;
        s_hop[s_hop_n++] = 11;
    }
}

static void hop_task(void *arg)
{
    int i = 0;
    while (1) {
        if (!s_scanning) {
            esp_wifi_set_channel(s_hop[i], WIFI_SECOND_CHAN_NONE);
            i = (i + 1) % s_hop_n;
        }
        vTaskDelay(pdMS_TO_TICKS(CONFIG_CSI_HOP_DWELL_MS));
    }
}
#endif

#if CONFIG_CSI_ENABLE_SCAN
static void scan_task(void *arg)
{
    static wifi_ap_record_t recs[MAX_AP];
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(CONFIG_CSI_SCAN_PERIOD_MS));

        s_scanning = true;                     /* hold the hop task still */
        esp_wifi_set_promiscuous(false);       /* scanning needs the radio */

        wifi_scan_config_t sc;
        memset(&sc, 0, sizeof(sc));
        sc.show_hidden = true;

        if (esp_wifi_scan_start(&sc, true) == ESP_OK) {
            uint16_t got = MAX_AP;
            memset(recs, 0, sizeof(recs));
            if (esp_wifi_scan_get_ap_records(&got, recs) == ESP_OK) {
                tx_header(T_SCAN, s_seq++);
                tx_u8((uint8_t)(got > 255 ? 255 : got));
                for (int i = 0; i < got && i < 255; i++) {
                    tx_need(9 + 32);
                    tx_u8((uint8_t)(int8_t)recs[i].rssi);
                    tx_u8((uint8_t)recs[i].primary);
                    tx_bytes(recs[i].bssid, 6);
                    uint8_t sl = (uint8_t)strnlen((const char *)recs[i].ssid, 32);
                    tx_u8(sl);
                    tx_bytes(recs[i].ssid, sl);
                }
                tx_flush();
            }
        }
        esp_wifi_set_promiscuous(true);
        s_scanning = false;
    }
}
#endif

/* ------------------------------ events ------------------------------------ */
static EventGroupHandle_t s_eg;
#define WIFI_CONNECTED_BIT BIT0

static void wifi_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
#if CONFIG_CSI_ASSOCIATE
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGW(TAG, "disconnected, reconnecting");
        esp_wifi_connect();
    } else
#endif
    if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *e = (ip_event_got_ip_t *)data;
        ESP_LOGW(TAG, "got ip " IPSTR, IP2STR(&e->ip_info.ip));
        xEventGroupSetBits(s_eg, WIFI_CONNECTED_BIT);
    }
}

/* ------------------------------- app -------------------------------------- */
void app_main(void)
{
    setvbuf(stdout, NULL, _IONBF, 0);       /* binary-safe, unbuffered */
    esp_log_level_set("*", ESP_LOG_WARN);

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t ic = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&ic));

    s_eg = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event, NULL, NULL));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));

#if CONFIG_CSI_ASSOCIATE
    wifi_config_t wc;
    memset(&wc, 0, sizeof(wc));
    strncpy((char *)wc.sta.ssid, CONFIG_CSI_WIFI_SSID, sizeof(wc.sta.ssid) - 1);
    strncpy((char *)wc.sta.password, CONFIG_CSI_WIFI_PASSWORD, sizeof(wc.sta.password) - 1);
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wc));
#endif

    ESP_ERROR_CHECK(esp_wifi_start());

#if CONFIG_CSI_ASSOCIATE
    ESP_LOGW(TAG, "associating with %s ...", CONFIG_CSI_WIFI_SSID);
    xEventGroupWaitBits(s_eg, WIFI_CONNECTED_BIT, pdFALSE, pdTRUE, portMAX_DELAY);
#elif CONFIG_CSI_HOP
    parse_channels();
    esp_wifi_set_channel(s_hop[0], WIFI_SECOND_CHAN_NONE);
    ESP_LOGW(TAG, "listen-only, hopping %d channel(s)", s_hop_n);
#else
    ESP_ERROR_CHECK(esp_wifi_set_channel(CONFIG_CSI_LISTEN_CHANNEL, WIFI_SECOND_CHAN_NONE));
    ESP_LOGW(TAG, "listen-only on channel %d", CONFIG_CSI_LISTEN_CHANNEL);
#endif

    /* capture every frame on the tuned channel */
    wifi_promiscuous_filter_t filt;
    filt.filter_mask = WIFI_PROMIS_FILTER_MASK_ALL;
    ESP_ERROR_CHECK(esp_wifi_set_promiscuous_filter(&filt));
    ESP_ERROR_CHECK(esp_wifi_set_promiscuous(true));

    /* CSI config + callback */
    wifi_csi_config_t cc;
    memset(&cc, 0, sizeof(cc));
    cc.lltf_en           = true;
    cc.htltf_en          = true;
    cc.stbc_htltf2_en    = true;
    cc.ltf_merge_en      = true;
    cc.channel_filter_en = false;
    cc.manu_scale        = false;
    ESP_ERROR_CHECK(esp_wifi_set_csi_config(&cc));
    ESP_ERROR_CHECK(esp_wifi_set_csi_rx_cb(&csi_rx_cb, NULL));
    ESP_ERROR_CHECK(esp_wifi_set_csi(true));

    send_hello();
    ESP_LOGW(TAG, "node %d streaming CSI", CONFIG_CSI_NODE_ID);

    xTaskCreate(stream_task, "stream", 4096, NULL, 5, NULL);
#if CONFIG_CSI_TRAFFIC_GEN
    xTaskCreate(traffic_task, "traffic", 4096, NULL, 4, NULL);
#endif
#if !CONFIG_CSI_ASSOCIATE && CONFIG_CSI_HOP
    xTaskCreate(hop_task, "hop", 3072, NULL, 3, NULL);
#endif
#if CONFIG_CSI_ENABLE_SCAN
    xTaskCreate(scan_task, "scan", 4096, NULL, 2, NULL);
#endif
}
