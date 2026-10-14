# xiao-cam

Camera and telemetry firmware for the Seeed Studio XIAO ESP32S3 Sense on the rover. ESP-IDF v5.4.4.
The board only streams video and telemetry to the Mac over Wi-Fi; it never drives the rover.

## Endpoints

| URL | What |
| --- | --- |
| `http://rover-cam.local/` | Bench page: live view + telemetry |
| `http://rover-cam.local:81/stream` | MJPEG (`multipart/x-mixed-replace`). Each part has `X-Timestamp-Us` (capture time) and `X-Frame-Seq` headers |
| `http://rover-cam.local/capture.jpg` | Single JPEG, same headers |
| `ws://rover-cam.local/ws` | JSON telemetry pushed every 200 ms |

Telemetry message:

```json
{"type":"telemetry","seq":184,"t_us":37247278,"epoch_ms":1791998901664,"rssi":-43,
 "frame_seq":62,"last_frame_age_ms":6855,"heap_free":209887,"psram_free":8261776}
```

`t_us` and `X-Timestamp-Us` are device uptime in microseconds (same clock), so frame age on the
Mac is `latest t_us - X-Timestamp-Us` plus link delay. `epoch_ms` is `null` until SNTP syncs.
`last_frame_age_ms` only advances while someone is pulling frames.

## Build and flash

```sh
cp main/secrets.h.example main/secrets.h   # set your 2.4 GHz SSID/password; gitignored
. ~/esp-idf/export.sh
idf.py set-target esp32s3                  # first time only
idf.py -p /dev/cu.usbmodem* build flash monitor
```

If the board doesn't show up as a serial port, hold BOOT while plugging in USB.

## Bench results (2026-10-14, USB power, desk)

- Camera detected as OV3660 at 0x3c; 8 MB octal PSRAM passes memory test.
- Wi-Fi STA joins in ~3 s, RSSI -43 dBm; mDNS `rover-cam.local` resolves.
- VGA MJPEG: 60 frames in 4 s (~15 fps), ~8.5 KB/frame at quality 12.
- Telemetry interval measured at 200.0 ms; SNTP synced.

Not yet verified: microphone, behaviour on rover battery power, and resets under motor load.
