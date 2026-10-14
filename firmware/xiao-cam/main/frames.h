#pragma once

#include <stddef.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"

// One capture task grabs each camera frame once and shares it with every viewer
// (MJPEG clients, /capture.jpg). Frames are reference-counted copies in PSRAM.

typedef struct {
    uint8_t *buf;
    size_t len;
    int64_t timestamp_us;  // capture time, device uptime clock
    uint32_t seq;
    int refs;
} shared_frame_t;

void frames_start(void);

// Register/unregister a viewer; the capture task only runs while viewers exist.
void frames_viewer_add(void);
void frames_viewer_remove(void);

// Wait for a frame newer than after_seq. Returns NULL on timeout. Release with frames_release().
shared_frame_t *frames_acquire(uint32_t after_seq, TickType_t timeout);
void frames_release(shared_frame_t *frame);
