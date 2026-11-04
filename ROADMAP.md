# Autonomy roadmap

Working list, one item at a time, simplest first (details and reasoning: [PLAN.md](PLAN.md#autonomy-roadmap-2026-11-03-replaces-the-dog-companion-goal)).

- [x] 1. **Measured moves** — "forward N cm", "turn X°" from timed drives; distance/position counter (estimate). *Done 2026-11-03: forward 25 cm/s, reverse 43 cm/s, full-left radius 132.5 cm (2 s arc: 70 cm forward, 20 cm left → 32°); turns are timed from the arc run itself. Accuracy to check in use (forward speed looked low next to the arc run's ~37 cm/s). Turns need ~2 m of arc for 90°.*
- [ ] 2. **Snapshot task v0** — "take a picture of my dog" when he's in view: find, centre, approach, save the best frame to a gallery.
- [ ] 3. **Gyro heading**: the M5StickS3 on the rover streams heading over Wi-Fi; turns by angle, heading hold on straight moves. *Priority raised 2026-11-03: the front wheels have play, so the rover drifts randomly on straight runs. Centre trim only fixes the steady part; heading-hold steering fixes the rest. Built 2026-11-03 ([firmware/m5stick-imu](firmware/m5stick-imu/README.md), `drive/imu.py`, Heading card on the dashboard). Mounted, and the angle reads true by hand. First road run: the 1 m straight held its average heading but zigzagged ±4° and ended 7° off. Short full-lock taps still felt zig-zaggy: a single steer key kept the wheels turned for 400 ms. Rover app 0.9 adds fine steering from back-to-back keys: `W W` centres the wheels now, `W W A/D` steers at 30% of full lock. Heading hold uses these and only goes to full lock beyond 10° off, and every straight move finishes straight. The 90° turn reached +62° and then got stuck; a turn now stops if the heading stays unchanged for 1 s. To do: rerun both.*
- [ ] 4. **Front distance** — ToF sensor (VL53L1X, ~US$8); stop before obstacles, wall following.
- [ ] 5. **Wheel odometry** — printed slotted disc + IR interrupter (~US$3); real distance travelled.
- [ ] 6. **Dead-reckoning pose** — gyro + odometry → x, y, heading; live trail on the dashboard.
- [ ] 7. **Landmarks** — printed AprilTags at known spots correct the drift: the rover knows where it is.
- [ ] 8. **Apartment map** — floor plan as a grid with named rooms; rover shown live on it.
- [ ] 9. **Go to a place** — "go to the kitchen": path planning with distance-sensor safety; return home.
- [ ] 10. **Tasks on the map** — "find the dog and take a picture": visit rooms, look, photograph, come back.
- [ ] 11. **Builds the map itself** — exploration; camera-only visual SLAM as an experiment.
- [ ] 12. **Docking and unattended runs** — return to a charger, scheduled tasks.
- [ ] 13. **Full stack** — 2D LiDAR, Raspberry Pi onboard, ROS 2 + slam_toolbox + Nav2, Tailscale.

Later add-on: the owner's WaveSight home-sensing project as a room-level position hint (see PLAN.md).
