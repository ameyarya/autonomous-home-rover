// XIAO ESP32S3 Sense camera/telemetry board for the home rover.
//
// Port 80:  /             bench page (live view + telemetry)
//           /capture.jpg  single JPEG frame
//           /ws           WebSocket, pushes JSON telemetry every TELEMETRY_PERIOD_MS
//           /config       GET shows settings; GET /config?key=val&... changes and persists them
//           /ota          POST firmware image (X-OTA-Token header), reboots into it
//           /reboot       POST (X-OTA-Token header)
// Port 81:  /stream       MJPEG (multipart/x-mixed-replace); each part carries
//                         X-Timestamp-Us (capture time, uptime clock) and X-Frame-Seq
//
// All timestamps are microseconds of device uptime (esp_timer), the same clock as
// telemetry "t_us", so the Mac can compute frame age. "epoch_ms" is added once SNTP syncs.
// This board never drives the rover.

#include <stdio.h>
#include <string.h>
#include <sys/param.h>
#include <sys/time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_app_desc.h"
#include "esp_event.h"
#include "esp_heap_caps.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_netif_sntp.h"
#include "esp_ota_ops.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "mdns.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "cam.h"
#include "mic.h"

#if __has_include("secrets.h")
#include "secrets.h"
#else
#error "Copy main/secrets.h.example to main/secrets.h and set WIFI_SSID / WIFI_PASS / OTA_TOKEN"
#endif
#ifndef OTA_TOKEN
#error "Add OTA_TOKEN to main/secrets.h (see secrets.h.example)"
#endif

#define HOSTNAME             "rover-cam"
#define TELEMETRY_PERIOD_MS  200
#define STREAM_BOUNDARY      "frame"
#define LED_GPIO             GPIO_NUM_21  // user LED, active low
#define STREAMING_WINDOW_US  1000000
#define OTA_VERIFY_TIMEOUT_S 60

static const char *TAG = "xiao-cam";

static volatile bool s_wifi_up = false;
static volatile uint32_t s_telemetry_seq = 0;
static bool s_led_enabled = true;
static const char *s_reset_reason = "?";

// ---------- Health ----------

static const char *reset_reason_str(esp_reset_reason_t r)
{
    switch (r) {
    case ESP_RST_POWERON: return "poweron";
    case ESP_RST_EXT: return "external";
    case ESP_RST_SW: return "software";
    case ESP_RST_PANIC: return "panic";
    case ESP_RST_INT_WDT: return "int_wdt";
    case ESP_RST_TASK_WDT: return "task_wdt";
    case ESP_RST_WDT: return "wdt";
    case ESP_RST_DEEPSLEEP: return "deepsleep";
    case ESP_RST_BROWNOUT: return "brownout";
    case ESP_RST_USB: return "usb";
    case ESP_RST_JTAG: return "jtag";
    default: return "other";
    }
}

static bool streaming_recently(void)
{
    int64_t last = rcam_last_frame_us();
    return last && esp_timer_get_time() - last < STREAMING_WINDOW_US;
}

// LED: fast blink = no Wi-Fi, solid = connected, slow blink = frames being served.
static void led_task(void *arg)
{
    gpio_reset_pin(LED_GPIO);
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);
    bool on = false;
    for (;;) {
        int period_ms;
        if (!s_led_enabled) {
            on = false;
            period_ms = 500;
        } else if (!s_wifi_up) {
            on = !on;
            period_ms = 100;
        } else if (streaming_recently()) {
            on = !on;
            period_ms = 500;
        } else {
            on = true;
            period_ms = 200;
        }
        gpio_set_level(LED_GPIO, on ? 0 : 1);
        vTaskDelay(pdMS_TO_TICKS(period_ms));
    }
}

// After an OTA update the new image boots "pending verify". Keep it only once it
// has joined Wi-Fi with a working camera; otherwise roll back to the previous image.
static void ota_verify_task(void *arg)
{
    esp_ota_img_states_t state;
    const esp_partition_t *running = esp_ota_get_running_partition();
    if (esp_ota_get_state_partition(running, &state) != ESP_OK || state != ESP_OTA_IMG_PENDING_VERIFY) {
        vTaskDelete(NULL);
        return;
    }
    for (int i = 0; i < OTA_VERIFY_TIMEOUT_S; i++) {
        if (s_wifi_up && rcam_ok()) {
            esp_ota_mark_app_valid_cancel_rollback();
            ESP_LOGI(TAG, "new firmware verified");
            vTaskDelete(NULL);
            return;
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    ESP_LOGE(TAG, "new firmware failed verification, rolling back");
    esp_ota_mark_app_invalid_rollback_and_reboot();
}

// ---------- Persistent board settings (camera settings live in cam.c) ----------

static void board_settings_load(void)
{
    nvs_handle_t h;
    uint8_t led = 1;
    if (nvs_open("board", NVS_READONLY, &h) == ESP_OK) {
        nvs_get_u8(h, "led", &led);
        nvs_close(h);
    }
    s_led_enabled = led;
}

static esp_err_t board_set_led(const char *val)
{
    if (strcmp(val, "0") != 0 && strcmp(val, "1") != 0) {
        return ESP_ERR_INVALID_ARG;
    }
    s_led_enabled = val[0] == '1';
    nvs_handle_t h;
    esp_err_t err = nvs_open("board", NVS_READWRITE, &h);
    if (err == ESP_OK) {
        err = nvs_set_u8(h, "led", s_led_enabled);
        if (err == ESP_OK) {
            err = nvs_commit(h);
        }
        nvs_close(h);
    }
    return err;
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
        uint32_t seq;
        camera_fb_t *fb = rcam_grab(&seq);
        if (!fb) {
            ESP_LOGE(TAG, "frame capture failed");
            return ESP_FAIL;
        }
        int n = snprintf(part, sizeof(part),
                         "\r\n--" STREAM_BOUNDARY "\r\n"
                         "Content-Type: image/jpeg\r\n"
                         "Content-Length: %u\r\n"
                         "X-Timestamp-Us: %lld\r\n"
                         "X-Frame-Seq: %lu\r\n\r\n",
                         (unsigned)fb->len, (long long)rcam_fb_timestamp_us(fb), (unsigned long)seq);
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
    uint32_t seq;
    camera_fb_t *fb = rcam_grab(&seq);
    if (!fb) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    char ts[24], seq_str[12];
    snprintf(ts, sizeof(ts), "%lld", (long long)rcam_fb_timestamp_us(fb));
    snprintf(seq_str, sizeof(seq_str), "%lu", (unsigned long)seq);

    httpd_resp_set_type(req, "image/jpeg");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "X-Timestamp-Us", ts);
    httpd_resp_set_hdr(req, "X-Frame-Seq", seq_str);
    esp_err_t res = httpd_resp_send(req, (const char *)fb->buf, fb->len);
    esp_camera_fb_return(fb);
    return res;
}

static esp_err_t config_handler(httpd_req_t *req)
{
    char query[256], val[16], msg[96];
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK) {
        for (int i = 0; RCAM_SETTING_KEYS[i]; i++) {
            const char *key = RCAM_SETTING_KEYS[i];
            if (httpd_query_key_value(query, key, val, sizeof(val)) == ESP_OK && rcam_set(key, val) != ESP_OK) {
                snprintf(msg, sizeof(msg), "invalid %s=%s", key, val);
                return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, msg);
            }
        }
        if (httpd_query_key_value(query, "led", val, sizeof(val)) == ESP_OK && board_set_led(val) != ESP_OK) {
            return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "invalid led (0|1)");
        }
    }

    char body[320];
    int n = snprintf(body, sizeof(body), "{");
    n += rcam_settings_json(body + n, sizeof(body) - n);
    snprintf(body + n, sizeof(body) - n, ",\"led\":%d}\n", s_led_enabled);
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, body);
}

static bool token_ok(httpd_req_t *req)
{
    char tok[80];
    if (httpd_req_get_hdr_value_str(req, "X-OTA-Token", tok, sizeof(tok)) == ESP_OK && strcmp(tok, OTA_TOKEN) == 0) {
        return true;
    }
    httpd_resp_send_err(req, HTTPD_403_FORBIDDEN, "missing or wrong X-OTA-Token");
    return false;
}

static void restart_soon(void)
{
    vTaskDelay(pdMS_TO_TICKS(500));
    esp_restart();
}

static esp_err_t ota_handler(httpd_req_t *req)
{
    if (!token_ok(req)) {
        return ESP_FAIL;
    }
    const esp_partition_t *target = esp_ota_get_next_update_partition(NULL);
    esp_ota_handle_t ota;
    esp_err_t err = esp_ota_begin(target, OTA_WITH_SEQUENTIAL_WRITES, &ota);
    if (err != ESP_OK) {
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, esp_err_to_name(err));
    }
    ESP_LOGI(TAG, "ota: receiving %d bytes into %s", req->content_len, target->label);

    char *buf = malloc(4096);
    int remaining = req->content_len;
    while (buf && remaining > 0) {
        int n = httpd_req_recv(req, buf, MIN(remaining, 4096));
        if (n == HTTPD_SOCK_ERR_TIMEOUT) {
            continue;
        }
        if (n <= 0 || esp_ota_write(ota, buf, n) != ESP_OK) {
            break;
        }
        remaining -= n;
    }
    free(buf);
    if (remaining != 0) {
        esp_ota_abort(ota);
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "upload failed");
    }
    err = esp_ota_end(ota);  // validates the image
    if (err == ESP_OK) {
        err = esp_ota_set_boot_partition(target);
    }
    if (err != ESP_OK) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, esp_err_to_name(err));
    }
    ESP_LOGI(TAG, "ota: ok, rebooting into %s", target->label);
    httpd_resp_sendstr(req, "ok, rebooting\n");
    restart_soon();
    return ESP_OK;
}

static esp_err_t reboot_handler(httpd_req_t *req)
{
    if (!token_ok(req)) {
        return ESP_FAIL;
    }
    httpd_resp_sendstr(req, "rebooting\n");
    restart_soon();
    return ESP_OK;
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
    static uint32_t prev_frame_seq = 0;
    static int64_t prev_t_us = 0;

    httpd_handle_t srv = arg;
    int fds[CONFIG_LWIP_MAX_SOCKETS];
    size_t n_fds = CONFIG_LWIP_MAX_SOCKETS;
    if (httpd_get_client_list(srv, &n_fds, fds) != ESP_OK) {
        return;
    }

    int64_t now_us = esp_timer_get_time();
    uint32_t frame_seq = rcam_frame_seq();
    float fps = prev_t_us ? (frame_seq - prev_frame_seq) * 1e6f / (now_us - prev_t_us) : 0;
    prev_frame_seq = frame_seq;
    prev_t_us = now_us;

    int64_t last = rcam_last_frame_us();
    wifi_ap_record_t ap;
    int rssi = (s_wifi_up && esp_wifi_sta_get_ap_info(&ap) == ESP_OK) ? ap.rssi : 0;

    char epoch[24] = "null";
    struct timeval tv;
    gettimeofday(&tv, NULL);
    if (tv.tv_sec > 1700000000) {  // SNTP synced
        snprintf(epoch, sizeof(epoch), "%lld", (long long)tv.tv_sec * 1000 + tv.tv_usec / 1000);
    }

    char mic[48] = "\"mic_dbfs\":null,\"mic_peak_dbfs\":null";
    if (mic_ok()) {
        snprintf(mic, sizeof(mic), "\"mic_dbfs\":%.1f,\"mic_peak_dbfs\":%.1f", mic_dbfs(), mic_peak_dbfs());
    }

    char msg[512];
    int len = snprintf(msg, sizeof(msg),
                       "{\"type\":\"telemetry\",\"seq\":%lu,\"t_us\":%lld,\"epoch_ms\":%s,"
                       "\"fw\":\"%s\",\"reset_reason\":\"%s\",\"rssi\":%d,"
                       "\"cam_ok\":%s,\"frame_seq\":%lu,\"fps\":%.1f,\"last_frame_age_ms\":%lld,%s,"
                       "\"heap_free\":%u,\"psram_free\":%u}",
                       (unsigned long)++s_telemetry_seq, (long long)now_us, epoch,
                       esp_app_get_description()->version, s_reset_reason, rssi,
                       rcam_ok() ? "true" : "false", (unsigned long)frame_seq, fps,
                       last ? (long long)((now_us - last) / 1000) : -1LL, mic,
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
    cfg.recv_wait_timeout = 10;
    ESP_ERROR_CHECK(httpd_start(&ctrl, &cfg));

    const httpd_uri_t uris[] = {
        { .uri = "/", .method = HTTP_GET, .handler = index_handler },
        { .uri = "/capture.jpg", .method = HTTP_GET, .handler = capture_handler },
        { .uri = "/config", .method = HTTP_GET, .handler = config_handler },
        { .uri = "/ota", .method = HTTP_POST, .handler = ota_handler },
        { .uri = "/reboot", .method = HTTP_POST, .handler = reboot_handler },
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
    s_reset_reason = reset_reason_str(esp_reset_reason());
    ESP_LOGI(TAG, "fw %s, reset reason %s", esp_app_get_description()->version, s_reset_reason);

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
    board_settings_load();
    xTaskCreate(led_task, "led", 4096, NULL, 2, NULL);

    if (rcam_init() != ESP_OK) {
        ESP_LOGE(TAG, "camera init failed - check the Sense camera board and ribbon");
    }
    if (mic_init() != ESP_OK) {
        ESP_LOGE(TAG, "mic init failed");
    }
    wifi_init();
    services_init();
    http_init();
    xTaskCreate(ota_verify_task, "ota_verify", 4096, NULL, 3, NULL);
}
