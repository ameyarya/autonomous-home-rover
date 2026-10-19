# Simulation results

Closed-loop runs of the real `mac/dog_behavior.py` + `mac/drive_client.py` against the simulated rover
and dog (`sim/run_scenarios.py`, 6 seeds per scenario, 2026-10-16, rerun 2026-10-18 after the fixes). Rover numbers are estimates until
measured on the assembled chassis: 0.30 m/s at SPEED 900, 20° front-wheel angle, 157 mm wheelbase,
camera 13 cm off the floor with a 42° vertical field of view. A medium dog (~45 cm at the shoulder).

Scan (arc + reverse pair): 19.1° per pair, 19 pairs / ~57.0 s for 360°, footprint 0.37 m. Search limit: 75.0 s.

| Scenario | Runs | Dog contacts | Wall/furniture bumps | Min gap to dog (m, worst) | Pulses (median) | In view while following | Scenario check |
| --- | --- | --- | --- | --- | --- | --- | --- |
| walk_then_sleep | 6 | 0 | 24 (3/6 runs) | 0.123 | 28.5 | 0.98–1.0 | parked after sleep in 6/6, median 234.7 s after he fell asleep; standoff gap median 0.93 m |
| hide_to_side | 6 | 0 | 0 (0/6 runs) | 0.785 | 12.0 | 0.98–0.98 | searched 6/6, found 6/6 (median 6.2 s), parked after search 0/6 |
| leave_room | 6 | 0 | 13 (5/6 runs) | 0.905 | 79.0 | 0.85–1.0 | searched 0/6, found 0/6 (median — s), parked after search 0/6 |
| charge | 6 | 0 | 0 (0/6 runs) | 0.031 | 25.0 | 1.0–1.0 | backed off 6/6 |
| close_off_centre | 6 | 0 | 0 (0/6 runs) | 0.103 | 4.0 | 0.95–1.0 | final state following 5/6 |
| wake_from_sleep | 6 | 0 | 0 (0/6 runs) | 0.762 | 11.0 | 1.0–1.0 | followed after waking 6/6, median 3.2 s |

Wall time 264.3 s.

"Dog contacts" counts moves the rover started that would have hit the dog. "Wall/furniture bumps"
counts blocked pulses. "Min gap" includes the dog walking up to the rover on its own.

## Findings

1. **Never drove into the dog** in 36 runs, including a dog charging at it, lying close and off-centre,
   and walking past. Backing off can't outrun a running dog (0.30 m/s in 250 ms pulses with pauses),
   so close approaches by the dog itself still happen; the rover only ever retreats.
2. **Got stuck on walls and kept pushing — fixed 2026-10-18.** First run (2026-10-16): following the
   dog into a doorway it caught a front corner on the frame and repeated the same arc, up to 79 blocked
   pulses in 85 s; 281 blocked pulses across the 36 runs. Fix (`ViewChange` + `Behavior.note_move` in
   `mac/dog_behavior.py`, used by the watcher): after each pulse, compare the whole frame before and
   after against the camera's own frame-to-frame noise (measured while still). Forward/turn moves that
   change the view less than 2.6x the noise count as blocked: two in a row -> back up straight, five ->
   stop ("stuck") and wait 30 s. A reverse that changes it less than 2.4x the noise -> no reversing for
   20 s (hold instead); blocked both ways -> stop. Now: 23 blocked pulses in 36 runs (worst run 13 single
   bumps over 12 min, ≥20 s apart), 2 stuck stops, both genuinely wedged in the doorway, 0 false ones.
   The factors were tuned here (free moves changed the view ≥3.0x the noise, blocked ones ≤2.2x;
   reversing changes it least, so a fixed threshold misread a quarter of free reverses as blocked):
   recheck on the real camera with the watcher's `blocked` timeline events.
3. **Search couldn't complete a full turn — fixed 2026-10-18.** Scan pairs were A/D 350 ms + S 300 ms
   (~13° per pair, ~80 s per 360°) against a 60 s limit. Now A/D 500 ms (~19° per pair, ~57 s per
   360°) and a 75 s limit.
4. **Works as intended:** follows a walking dog keeping him in view 98–100% of the time at a median
   ~0.9 m gap; parks every time once he has been still 5 min and the model says sleeping (~235 s after
   he fell asleep, because he lay awake for 60 s first); finds a dog that dashed out of view in a
   median 6 s; starts following a median 3 s after a sleeping dog gets up.

## Limits

- Kinematic rover (no wheel slip, carpet, or battery sag); speed, steering angle and stop latency are
  estimates. Measure them after assembly (`RoverParams` in `rover_sim.py`) and rerun.
- The dog is scripted and simplified: capsule body, swinging legs, wagging tail; it walks around the
  rover rather than through it. YOLO is simulated from segmentation (misses more when he is small,
  cut off by the frame edge, or lying down). The vision model is an oracle with the watcher's cadence
  and 8 s latency, 10% "unclear".
- One room layout with one doorway.
