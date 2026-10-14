// XIAO ESP32S3 Sense camera/telemetry board for the home rover.
//
// Port 80:  /             bench page (live view + telemetry)
//           /capture.jpg  single JPEG frame
//           /ws           WebSocket, pushes JSON telemetry every TELEMETRY_PERIOD_MS
// Port 81:  /stream       MJPEG (multipart/x-mixed-replace); each part carries
//                         X-Timestamp-Us (capture time, uptime clock) and X-Frame-Seq
//
// All timestamps are microseconds of device uptime (esp_timer), the same clock as
// telemetry "t_us", so the Mac can compute frame age. "epoch_ms" is added once SNTP syncs.
// This board never drives the rover.

#include <stdio.h>
#include <string.h>
#include <sys/time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_netif_sntp.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "nvs_flash.h"
#include "esp_http_server.h"
#include "esp_camera.h"
#include "mdns.h"

#if __has_include("secrets.h")
#include "secrets.h"
#else
#error "Copy main/secrets.h.example to main/secrets.h and set WIFI_SSID / WIFI_PASS"
#endif

#define HOSTNAME             "rover-cam"
#define TELEMETRY_PERIOD_MS  200
#define STREAM_BOUNDARY      "frame"

static const char *TAG = "xiao-cam";

static volatile int64_t s_last_frame_us = 0;
static volatile uint32_t s_frame_seq = 0;
static volatile uint32_t s_telemetry_seq = 0;
static volatile bool s_wifi_up = false;

// ---------- Camera (XIAO ESP32S3 Sense pinout) ----------

static esp_err_t camera_init(void)
{
    camera_config_t cfg = {
        .pin_pwdn = -1,
        .pin_reset = -1,
        .pin_xclk = 10,
        .pin_sccb_sda = 40,
        .pin_sccb_scl = 39,
        .pin_d7 = 48,
        .pin_d6 = 11,
        .pin_d5 = 12,
        .pin_d4 = 14,
        .pin_d3 = 16,
        .pin_d2 = 18,
        .pin_d1 = 17,
        .pin_d0 = 15,
        .pin_vsync = 38,
        .pin_href = 47,
        .pin_pclk = 13,
        .xclk_freq_hz = 20000000,
        .ledc_timer = LEDC_TIMER_0,
        .ledc_channel = LEDC_CHANNEL_0,
        .pixel_format = PIXFORMAT_JPEG,
        .frame_size = FRAMESIZE_VGA,
        .jpeg_quality = 12,
        .fb_count = 2,
        .fb_location = CAMERA_FB_IN_PSRAM,
        .grab_mode = CAMERA_GRAB_LATEST,
    };
    esp_err_t err = esp_camera_init(&cfg);
    if (err == ESP_OK) {
        sensor_t *s = esp_camera_sensor_get();
        ESP_LOGI(TAG, "camera ok, sensor PID 0x%04x", s ? s->id.PID : 0);
    }
    return err;
}

static int64_t fb_timestamp_us(const camera_fb_t *fb)
{
    return (int64_t)fb->timestamp.tv_sec * 1000000LL + fb->timestamp.tv_usec;
}

// ---------- Wi-Fi ----------

static void wifi_event_handler(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        s_wifi_up = false;
        ESP_LOGW(TAG, "wifi disconnected, retrying");
        vTaskDelay(pdMS_TO_TICKS(1000));
        esp_wifi_connect();
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *ev = data;
        s_wifi_up = true;
        ESP_LOGI(TAG, "got ip " IPSTR "  ->  http://" IPSTR "/  or  http://" HOSTNAME ".local/",
                 IP2STR(&ev->ip_info.ip), IP2STR(&ev->ip_info.ip));
    }
}

static void wifi_init(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_t *netif = esp_netif_create_default_wifi_sta();
    esp_netif_set_hostname(netif, HOSTNAME);

    wifi_init_config_t init = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&init));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event_handler, NULL));

    wifi_config_t cfg = { 0 };
    strlcpy((char *)cfg.sta.ssid, WIFI_SSID, sizeof(cfg.sta.ssid));
    strlcpy((char *)cfg.sta.password, WIFI_PASS, sizeof(cfg.sta.password));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &cfg));
    ESP_ERROR_CHECK(esp_wifi_start());
    esp_wifi_set_ps(WIFI_PS_NONE);  // lower, steadier latency for video
}

static void services_init(void)
{
    esp_sntp_config_t sntp = ESP_NETIF_SNTP_DEFAULT_CONFIG("pool.ntp.org");
    esp_netif_sntp_init(&sntp);

    if (mdns_init() == ESP_OK) {
        mdns_hostname_set(HOSTNAME);
        mdns_service_add(NULL, "_http", "_tcp", 80, NULL, 0);
    }
}

// ---------- HTTP: stream server (port 81) ----------

static esp_err_t stream_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "multipart/x-mixed-replace;boundary=" STREAM_BOUNDARY);
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

    char part[192];
    esp_err_t res = ESP_OK;
    while (res == ESP_OK) {
        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) {
            ESP_LOGE(TAG, "frame capture failed");
            return ESP_FAIL;
        }
        int64_t ts = fb_timestamp_us(fb);
        uint32_t seq = ++s_frame_seq;
        s_last_frame_us = ts;

        int n = snprintf(part, sizeof(part),
                         "\r\n--" STREAM_BOUNDARY "\r\n"
                         "Content-Type: image/jpeg\r\n"
                         "Content-Length: %u\r\n"
                         "X-Timestamp-Us: %lld\r\n"
                         "X-Frame-Seq: %lu\r\n\r\n",
                         (unsigned)fb->len, (long long)ts, (unsigned long)seq);
        res = httpd_resp_send_chunk(req, part, n);
        if (res == ESP_OK) {
            res = httpd_resp_send_chunk(req, (const char *)fb->buf, fb->len);
        }
        esp_camera_fb_return(fb);
    }
    return res;
}

// ---------- HTTP: control server (port 80) ----------

static const char INDEX_HTML[] =
    "<!doctype html><meta name=viewport content='width=device-width'>"
    "<title>rover-cam</title>"
    "<body style='font:14px monospace;background:#111;color:#ddd;margin:16px'>"
    "<img id=v style='max-width:100%;display:block'>"
    "<pre id=t>connecting...</pre>"
    "<script>"
    "v.src=location.protocol+'//'+location.hostname+':81/stream';"
    "const w=new WebSocket('ws://'+location.host+'/ws');"
    "w.onmessage=e=>t.textContent=JSON.stringify(JSON.parse(e.data),null,1);"
    "w.onclose=()=>t.textContent='telemetry closed';"
    "</script>";

static esp_err_t index_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, INDEX_HTML, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t capture_handler(httpd_req_t *req)
{
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    char ts[24], seq[12];
    snprintf(ts, sizeof(ts), "%lld", (long long)fb_timestamp_us(fb));
    snprintf(seq, sizeof(seq), "%lu", (unsigned long)++s_frame_seq);
    s_last_frame_us = fb_timestamp_us(fb);

    httpd_resp_set_type(req, "image/jpeg");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "X-Timestamp-Us", ts);
    httpd_resp_set_hdr(req, "X-Frame-Seq", seq);
    esp_err_t res = httpd_resp_send(req, (const char *)fb->buf, fb->len);
    esp_camera_fb_return(fb);
    return res;
}

static esp_err_t ws_handler(httpd_req_t *req)
{
    if (req->method == HTTP_GET) {
        return ESP_OK;  // handshake
    }
    // Inbound messages are read and ignored; this board accepts no commands.
    httpd_ws_frame_t frame = { 0 };
    esp_err_t res = httpd_ws_recv_frame(req, &frame, 0);
    if (res != ESP_OK || frame.len == 0) {
        return res;
    }
    uint8_t *buf = malloc(frame.len);
    if (!buf) {
        return ESP_ERR_NO_MEM;
    }
    frame.payload = buf;
    res = httpd_ws_recv_frame(req, &frame, frame.len);
    free(buf);
    return res;
}

// Runs in the httpd task (via httpd_queue_work), so async sends are safe.
static void send_telemetry(void *arg)
{
    httpd_handle_t srv = arg;
    int fds[CONFIG_LWIP_MAX_SOCKETS];
    size_t n_fds = CONFIG_LWIP_MAX_SOCKETS;
    if (httpd_get_client_list(srv, &n_fds, fds) != ESP_OK) {
        return;
    }

    int64_t now_us = esp_timer_get_time();
    int64_t last = s_last_frame_us;
    wifi_ap_record_t ap;
    int rssi = (s_wifi_up && esp_wifi_sta_get_ap_info(&ap) == ESP_OK) ? ap.rssi : 0;

    char epoch[24] = "null";
    struct timeval tv;
    gettimeofday(&tv, NULL);
    if (tv.tv_sec > 1700000000) {  // SNTP synced
        snprintf(epoch, sizeof(epoch), "%lld", (long long)tv.tv_sec * 1000 + tv.tv_usec / 1000);
    }

    char msg[320];
    int len = snprintf(msg, sizeof(msg),
                       "{\"type\":\"telemetry\",\"seq\":%lu,\"t_us\":%lld,\"epoch_ms\":%s,"
                       "\"rssi\":%d,\"frame_seq\":%lu,\"last_frame_age_ms\":%lld,"
                       "\"heap_free\":%u,\"psram_free\":%u}",
                       (unsigned long)++s_telemetry_seq, (long long)now_us, epoch, rssi,
                       (unsigned long)s_frame_seq, last ? (long long)((now_us - last) / 1000) : -1LL,
                       (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
                       (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));

    httpd_ws_frame_t frame = {
        .final = true,
        .type = HTTPD_WS_TYPE_TEXT,
        .payload = (uint8_t *)msg,
        .len = len,
    };
    for (size_t i = 0; i < n_fds; i++) {
        if (httpd_ws_get_fd_info(srv, fds[i]) == HTTPD_WS_CLIENT_WEBSOCKET) {
            httpd_ws_send_frame_async(srv, fds[i], &frame);
        }
    }
}

static void telemetry_task(void *arg)
{
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(TELEMETRY_PERIOD_MS));
        httpd_queue_work((httpd_handle_t)arg, send_telemetry, arg);
    }
}

static void http_init(void)
{
    httpd_handle_t ctrl = NULL;
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.server_port = 80;
    cfg.ctrl_port = 32768;
    ESP_ERROR_CHECK(httpd_start(&ctrl, &cfg));

    const httpd_uri_t uris[] = {
        { .uri = "/", .method = HTTP_GET, .handler = index_handler },
        { .uri = "/capture.jpg", .method = HTTP_GET, .handler = capture_handler },
        { .uri = "/ws", .method = HTTP_GET, .handler = ws_handler, .is_websocket = true },
    };
    for (size_t i = 0; i < sizeof(uris) / sizeof(uris[0]); i++) {
        httpd_register_uri_handler(ctrl, &uris[i]);
    }
    xTaskCreate(telemetry_task, "telemetry", 4096, ctrl, 5, NULL);

    // Separate server so a long-lived stream never blocks control/telemetry requests.
    httpd_handle_t stream = NULL;
    httpd_config_t scfg = HTTPD_DEFAULT_CONFIG();
    scfg.server_port = 81;
    scfg.ctrl_port = 32769;
    scfg.max_open_sockets = 3;
    ESP_ERROR_CHECK(httpd_start(&stream, &scfg));
    const httpd_uri_t stream_uri = { .uri = "/stream", .method = HTTP_GET, .handler = stream_handler };
    httpd_register_uri_handler(stream, &stream_uri);
}

void app_main(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    if (camera_init() != ESP_OK) {
        ESP_LOGE(TAG, "camera init failed - check the Sense camera board and ribbon");
    }
    wifi_init();
    services_init();
    http_init();
}
