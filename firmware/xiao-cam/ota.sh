#!/bin/sh
# Build and push firmware to the XIAO over Wi-Fi. Usage: ./ota.sh [host]
set -e
cd "$(dirname "$0")"
HOST="${1:-rover-cam.local}"
TOKEN=$(sed -n 's/^#define OTA_TOKEN "\(.*\)"/\1/p' main/secrets.h)
[ -n "$TOKEN" ] || { echo "OTA_TOKEN missing from main/secrets.h" >&2; exit 1; }
. "${IDF_PATH:-$HOME/esp-idf}/export.sh" >/dev/null
idf.py build
curl --fail-with-body -sS -H "X-OTA-Token: $TOKEN" --data-binary @build/xiao-cam.bin "http://$HOST/ota"
