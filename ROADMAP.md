# Autonomy roadmap

Working list, one item at a time, simplest first (details and reasoning: [PLAN.md](PLAN.md#autonomy-roadmap-2026-11-03-replaces-the-dog-companion-goal)).

- [ ] 1. **Measured moves** — "forward N cm", "turn X°" from timed drives calibrated with a tape measure; distance-travelled counter (estimate). *In progress 2026-11-03.*
- [ ] 2. **Snapshot task v0** — "take a picture of my dog" when he's in view: find, centre, approach, save the best frame to a gallery.
- [ ] 3. **Gyro heading** — IMU (~US$3) on the XIAO's I2C; turns by angle, straight driving.
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
