#pragma once

#include <stdbool.h>
#include "esp_err.h"

// Onboard PDM microphone (XIAO Sense: CLK GPIO42, DATA GPIO41), 16 kHz mono.
// A background task keeps the level of the latest 100 ms window.
esp_err_t mic_init(void);
bool mic_ok(void);
float mic_dbfs(void);       // RMS level, dBFS (-96 = silence floor)
float mic_peak_dbfs(void);  // peak sample level in the same window
