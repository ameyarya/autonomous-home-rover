# M5StickS3 heading stick

An M5StickS3 (ESP32-S3, BMI270 gyro and accelerometer, battery, screen) mounted on the rover.
It works out which way the rover faces and streams that to the Mac over Wi-Fi. The drive server
uses it for heading hold on straight moves and for turning by angle.

- **Heading** is yaw about the gravity vector, so the stick can be mounted at any angle as long as
  it doesn't move. It counts past 360°, and + means turned left.
- **Calibration:** at power-on, keep it still for about a second (the screen says "keep still...").
  It re-learns the gyro bias every time the rover stands still, so drift stays small.
- **Bumps:** jolts over 0.6 g are counted.
- **Screen:** IP address, a compass needle, the heading, still/moving, and battery.
  Button A (the big front button) zeroes the heading.

## Protocol (UDP port 8773, `rover-imu.local`)

| From | Message | Meaning |
|---|---|---|
| Mac | `sub` | Stream to me for the next 3 s. The Mac resends it every second. |
| Mac | `zero` | The current heading becomes 0. |
| Stick | `{"seq","yaw","rate","still","cal","bumps","bat","up"}` | About 50 per second. Degrees, °/s, battery %, uptime in ms. |

The Mac side is [drive/imu.py](../../drive/imu.py).

## Build and flash (USB)

```sh
cp main/secrets.h.example main/secrets.h     # Wi-Fi SSID and password; gitignored
. ~/Documents/Projects/esp-idf/export.sh      # ESP-IDF v5.4 (see the xiao-cam README for the manual env)
idf.py set-target esp32s3 build
idf.py -p /dev/cu.usbmodem<stick> flash
```

It uses M5Unified and mDNS from the ESP component registry. Arduino isn't needed.

## Verified 2026-11-03

- Joins Wi-Fi as `rover-imu.local`, calibrates while still, and streams about 46 packets per second
  to the Mac (137 in 3 s).
- The drive server reports it online and calibrated, at 0.0°, battery 100%.
- Screen redraw is double-buffered, which fixed a flicker in the first build.

Not yet verified: rotating it to check the angle reads true, and driving with heading hold.

A full 8 MB backup of the factory firmware is kept locally in `backups/` (not in git). To restore
it: `esptool.py --chip esp32s3 -p <port> write_flash 0 backups/sticks3-factory-8MB.bin`.
