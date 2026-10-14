#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"
#include "esp_camera.h"

// Initialise the sensor and apply the settings saved in NVS.
esp_err_t rcam_init(void);
bool rcam_ok(void);

// Grab a frame, tracking sequence number and capture time. Restarts the board
// after repeated consecutive failures (camera hang). Return the frame with
// esp_camera_fb_return().
camera_fb_t *rcam_grab(uint32_t *seq_out);
int64_t rcam_fb_timestamp_us(const camera_fb_t *fb);
uint32_t rcam_frame_seq(void);
int64_t rcam_last_frame_us(void);

// Settings: framesize (qvga|vga|svga|xga|hd), quality (4-63, lower is better),
// vflip/hmirror (0|1), brightness/contrast/saturation/ae_level (-2..2).
// rcam_set applies the value and persists it.
esp_err_t rcam_set(const char *key, const char *val);
int rcam_settings_json(char *buf, size_t len);
extern const char *const RCAM_SETTING_KEYS[];
