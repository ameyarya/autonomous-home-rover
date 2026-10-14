#include "mic.h"

#include <math.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2s_pdm.h"
#include "esp_log.h"

#define SAMPLE_RATE    16000
#define WINDOW_SAMPLES (SAMPLE_RATE / 10)  // 100 ms
#define FLOOR_DBFS     -96.0f

static const char *TAG = "mic";

static i2s_chan_handle_t s_rx;
static bool s_ok = false;
static volatile float s_dbfs = FLOOR_DBFS;
static volatile float s_peak_dbfs = FLOOR_DBFS;

static float to_dbfs(float amplitude)
{
    return amplitude < 1.0f ? FLOOR_DBFS : 20.0f * log10f(amplitude / 32768.0f);
}

static void mic_task(void *arg)
{
    int16_t *buf = malloc(WINDOW_SAMPLES * sizeof(int16_t));
    if (!buf) {
        vTaskDelete(NULL);
        return;
    }
    for (;;) {
        size_t got = 0;
        if (i2s_channel_read(s_rx, buf, WINDOW_SAMPLES * sizeof(int16_t), &got, portMAX_DELAY) != ESP_OK) {
            continue;
        }
        size_t n = got / sizeof(int16_t);
        if (n == 0) {
            continue;
        }
        // Remove DC offset before measuring.
        int64_t sum = 0;
        for (size_t i = 0; i < n; i++) {
            sum += buf[i];
        }
        float mean = (float)sum / n;
        float sq = 0, peak = 0;
        for (size_t i = 0; i < n; i++) {
            float v = buf[i] - mean;
            sq += v * v;
            if (fabsf(v) > peak) {
                peak = fabsf(v);
            }
        }
        s_dbfs = to_dbfs(sqrtf(sq / n));
        s_peak_dbfs = to_dbfs(peak);
    }
}

esp_err_t mic_init(void)
{
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    esp_err_t err = i2s_new_channel(&chan_cfg, NULL, &s_rx);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "channel: %s", esp_err_to_name(err));
        return err;
    }
    i2s_pdm_rx_config_t pdm_cfg = {
        .clk_cfg = I2S_PDM_RX_CLK_DEFAULT_CONFIG(SAMPLE_RATE),
        .slot_cfg = I2S_PDM_RX_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO),
        .gpio_cfg = {
            .clk = GPIO_NUM_42,
            .din = GPIO_NUM_41,
        },
    };
    err = i2s_channel_init_pdm_rx_mode(s_rx, &pdm_cfg);
    if (err == ESP_OK) {
        err = i2s_channel_enable(s_rx);
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "pdm rx: %s", esp_err_to_name(err));
        return err;
    }
    xTaskCreate(mic_task, "mic", 4096, NULL, 4, NULL);
    s_ok = true;
    ESP_LOGI(TAG, "ok, %d Hz PDM", SAMPLE_RATE);
    return ESP_OK;
}

bool mic_ok(void)
{
    return s_ok;
}

float mic_dbfs(void)
{
    return s_dbfs;
}

float mic_peak_dbfs(void)
{
    return s_peak_dbfs;
}
