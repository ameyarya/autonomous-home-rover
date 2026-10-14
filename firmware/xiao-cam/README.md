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
| `http://rover-cam.local/config` | Current settings as JSON; add `?key=value&...` to change them (saved across reboots) |
| `POST /ota` | Upload a firmware image (needs `X-OTA-Token`); use `./ota.sh` |
| `POST /reboot` | Reboot (needs `X-OTA-Token`) |

Telemetry message:

```json
{"type":"telemetry","seq":44,"t_us":9310809,"epoch_ms":1792000670117,"fw":"9bbb2f5",
 "reset_reason":"software","rssi":-47,"cam_ok":true,"frame_seq":0,"fps":0.0,
 "last_frame_age_ms":-1,"mic_dbfs":-62.9,"mic_peak_dbfs":-53.0,
 "heap_free":192727,"psram_free":8015476}
```

- `t_us` and `X-Timestamp-Us` are device uptime in microseconds (same clock), so frame age on the
  Mac is `latest t_us - X-Timestamp-Us` plus link delay. `epoch_ms` is `null` until SNTP syncs.
- `frame_seq`, `fps` and `last_frame_age_ms` only move while someone is pulling frames.
- `fw` is the `git describe` of the commit the image was built from (`-dirty` = uncommitted changes).
- `reset_reason`: `poweron`, `software` (reboot/OTA), `usb`, `panic`, `brownout` (supply sagged),
  `task_wdt`/`int_wdt`/`wdt` (hang), `external`, `other`.
- `mic_dbfs` / `mic_peak_dbfs`: RMS and peak level of the last 100 ms, dBFS. Quiet room ≈ -65.

## Settings

| Key | Values | Default |
| --- | --- | --- |
| `framesize` | `qvga` 320×240, `vga` 640×480, `svga` 800×600, `xga` 1024×768, `hd` 1280×720 | `vga` |
| `quality` | 4–63 (lower = better, bigger frames) | 12 |
| `vflip`, `hmirror` | 0 / 1 (set once the camera is mounted) | 0 |
| `brightness`, `contrast`, `saturation`, `ae_level` | -2 … 2 | 0 |
| `led` | 0 / 1 (turn off if it glares into the lens) | 1 |

Example: `curl 'http://rover-cam.local/config?vflip=1&hmirror=1&framesize=svga'`. Bad values return 400.

## Health and recovery

- LED (GPIO21): fast blink = no Wi-Fi, solid = connected, slow blink = frames being served.
- 5 capture failures in a row restart the board (camera hang).
- After an OTA update the new image must join Wi-Fi with a working camera within 60 s,
  otherwise the bootloader rolls back to the previous image.

## Build and flash

First time, over USB (writes the partition table):

```sh
cp main/secrets.h.example main/secrets.h   # SSID, password, OTA token; gitignored
. ~/esp-idf/export.sh
idf.py set-target esp32s3
idf.py -p /dev/cu.usbmodem* erase-flash flash monitor
```

After that, over Wi-Fi from this directory:

```sh
./ota.sh                    # or ./ota.sh 192.168.1.194 if mDNS is slow
```

If USB flashing fails to connect, hold BOOT while plugging in USB.

## Bench results

2026-10-14, desk, USB power then iPhone Air battery pack:

- Camera detected as OV3660 at 0x3c; 8 MB octal PSRAM passes memory test.
- Wi-Fi STA RSSI -42 to -47 dBm; mDNS `rover-cam.local` resolves. On boot it usually needs two
  retries (~9 s) before getting an IP.
- VGA MJPEG: 60 frames in 4 s (~15 fps), ~8.5 KB/frame at quality 12. Telemetry interval 200.0 ms.
- Mic: quiet room -62 to -70 dBFS RMS; Mac speaker playback raised it to about -54 dBFS RMS,
  peaks -44 dBFS.
- Settings persisted across 3 consecutive reboots (back online in 4–5 s each).
- OTA: build + upload + reboot in 18 s; wrong token → 403, non-image upload rejected; image
  passed self-check and stayed on `ota_1` after a hard reset.
- Fixed during bring-up: a 2 KB LED task stack overflowed into NVS memory and boot-looped the
  board once settings existed; stacks are now 4 KB.

Not yet verified: running on the rover chassis and behaviour while the motors run.
