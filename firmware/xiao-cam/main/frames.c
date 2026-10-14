#include "frames.h"

#include <string.h>
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "cam.h"

static const char *TAG = "frames";

static SemaphoreHandle_t s_lock;
static shared_frame_t *s_latest;
static volatile int s_viewers;

static void release_locked(shared_frame_t *frame)
{
    if (frame && --frame->refs == 0) {
        heap_caps_free(frame->buf);
        heap_caps_free(frame);
    }
}

void frames_release(shared_frame_t *frame)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    release_locked(frame);
    xSemaphoreGive(s_lock);
}

shared_frame_t *frames_acquire(uint32_t after_seq, TickType_t timeout)
{
    TickType_t start = xTaskGetTickCount();
    for (;;) {
        xSemaphoreTake(s_lock, portMAX_DELAY);
        shared_frame_t *frame = s_latest;
        if (frame && frame->seq > after_seq) {
            frame->refs++;
            xSemaphoreGive(s_lock);
            return frame;
        }
        xSemaphoreGive(s_lock);
        if (xTaskGetTickCount() - start >= timeout) {
            return NULL;
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

void frames_viewer_add(void)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_viewers++;
    xSemaphoreGive(s_lock);
}

void frames_viewer_remove(void)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_viewers--;
    xSemaphoreGive(s_lock);
}

static void capture_task(void *arg)
{
    for (;;) {
        if (s_viewers <= 0 || !rcam_ok()) {
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }
        uint32_t seq;
        camera_fb_t *fb = rcam_grab(&seq);  // restarts the board after repeated failures
        if (!fb) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        shared_frame_t *frame = heap_caps_malloc(sizeof(*frame), MALLOC_CAP_SPIRAM);
        uint8_t *buf = heap_caps_malloc(fb->len, MALLOC_CAP_SPIRAM);
        if (!frame || !buf) {
            heap_caps_free(frame);
            heap_caps_free(buf);
            esp_camera_fb_return(fb);
            ESP_LOGW(TAG, "out of PSRAM for frame copy");
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }
        memcpy(buf, fb->buf, fb->len);
        *frame = (shared_frame_t){
            .buf = buf,
            .len = fb->len,
            .timestamp_us = rcam_fb_timestamp_us(fb),
            .seq = seq,
            .refs = 1,  // held by s_latest
        };
        esp_camera_fb_return(fb);

        xSemaphoreTake(s_lock, portMAX_DELAY);
        shared_frame_t *old = s_latest;
        s_latest = frame;
        release_locked(old);
        xSemaphoreGive(s_lock);
    }
}

void frames_start(void)
{
    s_lock = xSemaphoreCreateMutex();
    xTaskCreatePinnedToCore(capture_task, "capture", 4096, NULL, 6, NULL, 1);
}
