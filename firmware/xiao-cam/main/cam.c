#include "cam.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "esp_log.h"
#include "esp_system.h"
#include "nvs.h"

#define MAX_CONSECUTIVE_FAILS 5
#define NVS_NS                "cam"
#define NVS_KEY               "settings"
#define SETTINGS_VERSION      1

static const char *TAG = "cam";

typedef struct {
    uint8_t version;
    uint8_t framesize;
    int8_t quality;
    int8_t vflip;
    int8_t hmirror;
    int8_t brightness;
    int8_t contrast;
    int8_t saturation;
    int8_t ae_level;
} cam_settings_t;

static const cam_settings_t DEFAULTS = {
    .version = SETTINGS_VERSION,
    .framesize = FRAMESIZE_VGA,
    .quality = 12,
};

static const struct {
    const char *name;
    framesize_t size;
} FRAMESIZES[] = {
    { "qvga", FRAMESIZE_QVGA },
    { "vga", FRAMESIZE_VGA },
    { "svga", FRAMESIZE_SVGA },
    { "xga", FRAMESIZE_XGA },
    { "hd", FRAMESIZE_HD },
};
#define N_FRAMESIZES (sizeof(FRAMESIZES) / sizeof(FRAMESIZES[0]))

const char *const RCAM_SETTING_KEYS[] = {
    "framesize", "quality", "vflip", "hmirror", "brightness", "contrast", "saturation", "ae_level", NULL,
};

static cam_settings_t s_settings;
static bool s_ok = false;
static volatile uint32_t s_seq = 0;
static volatile int64_t s_last_frame_us = 0;
static volatile int s_fails = 0;

static const char *framesize_name(uint8_t fs)
{
    for (size_t i = 0; i < N_FRAMESIZES; i++) {
        if (FRAMESIZES[i].size == fs) {
            return FRAMESIZES[i].name;
        }
    }
    return "?";
}

static void settings_load(void)
{
    s_settings = DEFAULTS;
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READONLY, &h) != ESP_OK) {
        return;
    }
    cam_settings_t saved;
    size_t len = sizeof(saved);
    if (nvs_get_blob(h, NVS_KEY, &saved, &len) == ESP_OK && len == sizeof(saved) &&
        saved.version == SETTINGS_VERSION) {
        s_settings = saved;
    }
    nvs_close(h);
}

static esp_err_t settings_save(void)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NS, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        return err;
    }
    err = nvs_set_blob(h, NVS_KEY, &s_settings, sizeof(s_settings));
    if (err == ESP_OK) {
        err = nvs_commit(h);
    }
    nvs_close(h);
    return err;
}

static void settings_apply(void)
{
    sensor_t *s = esp_camera_sensor_get();
    if (!s) {
        return;
    }
    s->set_framesize(s, (framesize_t)s_settings.framesize);
    s->set_quality(s, s_settings.quality);
    s->set_vflip(s, s_settings.vflip);
    s->set_hmirror(s, s_settings.hmirror);
    s->set_brightness(s, s_settings.brightness);
    s->set_contrast(s, s_settings.contrast);
    s->set_saturation(s, s_settings.saturation);
    s->set_ae_level(s, s_settings.ae_level);
}

esp_err_t rcam_init(void)
{
    settings_load();

    // XIAO ESP32S3 Sense pinout. Frame buffers are sized for the largest
    // allowed framesize (HD) so it can be changed at runtime.
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
        .frame_size = FRAMESIZE_HD,
        .jpeg_quality = 10,
        .fb_count = 2,
        .fb_location = CAMERA_FB_IN_PSRAM,
        .grab_mode = CAMERA_GRAB_LATEST,
    };
    esp_err_t err = esp_camera_init(&cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "init failed: %s", esp_err_to_name(err));
        return err;
    }
    settings_apply();
    s_ok = true;
    sensor_t *s = esp_camera_sensor_get();
    ESP_LOGI(TAG, "ok, sensor PID 0x%04x, %s q%d", s ? s->id.PID : 0,
             framesize_name(s_settings.framesize), s_settings.quality);
    return ESP_OK;
}

bool rcam_ok(void)
{
    return s_ok;
}

int64_t rcam_fb_timestamp_us(const camera_fb_t *fb)
{
    return (int64_t)fb->timestamp.tv_sec * 1000000LL + fb->timestamp.tv_usec;
}

camera_fb_t *rcam_grab(uint32_t *seq_out)
{
    if (!s_ok) {
        return NULL;
    }
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
        if (++s_fails >= MAX_CONSECUTIVE_FAILS) {
            ESP_LOGE(TAG, "%d consecutive capture failures, restarting", s_fails);
            esp_restart();
        }
        return NULL;
    }
    s_fails = 0;
    s_last_frame_us = rcam_fb_timestamp_us(fb);
    uint32_t seq = ++s_seq;
    if (seq_out) {
        *seq_out = seq;
    }
    return fb;
}

uint32_t rcam_frame_seq(void)
{
    return s_seq;
}

int64_t rcam_last_frame_us(void)
{
    return s_last_frame_us;
}

static bool parse_int(const char *val, int lo, int hi, int8_t *out)
{
    char *end;
    long v = strtol(val, &end, 10);
    if (*val == '\0' || *end != '\0' || v < lo || v > hi) {
        return false;
    }
    *out = (int8_t)v;
    return true;
}

esp_err_t rcam_set(const char *key, const char *val)
{
    cam_settings_t next = s_settings;
    bool valid;

    if (strcmp(key, "framesize") == 0) {
        valid = false;
        for (size_t i = 0; i < N_FRAMESIZES; i++) {
            if (strcmp(val, FRAMESIZES[i].name) == 0) {
                next.framesize = FRAMESIZES[i].size;
                valid = true;
            }
        }
    } else if (strcmp(key, "quality") == 0) {
        valid = parse_int(val, 4, 63, &next.quality);
    } else if (strcmp(key, "vflip") == 0) {
        valid = parse_int(val, 0, 1, &next.vflip);
    } else if (strcmp(key, "hmirror") == 0) {
        valid = parse_int(val, 0, 1, &next.hmirror);
    } else if (strcmp(key, "brightness") == 0) {
        valid = parse_int(val, -2, 2, &next.brightness);
    } else if (strcmp(key, "contrast") == 0) {
        valid = parse_int(val, -2, 2, &next.contrast);
    } else if (strcmp(key, "saturation") == 0) {
        valid = parse_int(val, -2, 2, &next.saturation);
    } else if (strcmp(key, "ae_level") == 0) {
        valid = parse_int(val, -2, 2, &next.ae_level);
    } else {
        return ESP_ERR_NOT_FOUND;
    }
    if (!valid) {
        return ESP_ERR_INVALID_ARG;
    }

    s_settings = next;
    settings_apply();
    ESP_LOGI(TAG, "set %s=%s", key, val);
    return settings_save();
}

int rcam_settings_json(char *buf, size_t len)
{
    return snprintf(buf, len,
                    "\"framesize\":\"%s\",\"quality\":%d,\"vflip\":%d,\"hmirror\":%d,"
                    "\"brightness\":%d,\"contrast\":%d,\"saturation\":%d,\"ae_level\":%d",
                    framesize_name(s_settings.framesize), s_settings.quality, s_settings.vflip,
                    s_settings.hmirror, s_settings.brightness, s_settings.contrast,
                    s_settings.saturation, s_settings.ae_level);
}
