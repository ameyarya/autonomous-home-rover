# Simulation results

Closed-loop runs of the real `mac/dog_behavior.py` + `mac/drive_client.py` against the simulated rover
and dog (`sim/run_scenarios.py`, 6 seeds per scenario, 2026-10-16). Rover numbers are estimates until
measured on the assembled chassis: 0.30 m/s at SPEED 900, 20° front-wheel angle, 157 mm wheelbase,
camera 13 cm off the floor with a 42° vertical field of view. A medium dog (~45 cm at the shoulder).

Scan (arc + reverse pair): 13.2° per pair, 28 pairs / ~79.8 s for 360°, footprint 0.16 m. Search limit: 60.0 s.

| Scenario | Runs | Dog contacts | Wall/furniture bumps | Min gap to dog (m, worst) | Pulses (median) | In view while following | Scenario check |
| --- | --- | --- | --- | --- | --- | --- | --- |
| walk_then_sleep | 6 | 0 | 73 (3/6 runs) | 0.094 | 42.5 | 0.98–1.0 | parked after sleep in 6/6, median 234.7 s after he fell asleep; standoff gap median 0.83 m |
| hide_to_side | 6 | 0 | 0 (0/6 runs) | 0.785 | 15.0 | 0.98–0.98 | searched 6/6, found 6/6 (median 5.8 s), parked after search 0/6 |
| leave_room | 6 | 0 | 208 (5/6 runs) | 0.905 | 76.5 | 1.0–1.0 | searched 0/6, found 0/6 (median — s), parked after search 0/6 |
| charge | 6 | 0 | 0 (0/6 runs) | 0.031 | 25.0 | 1.0–1.0 | backed off 6/6 |
| close_off_centre | 6 | 0 | 0 (0/6 runs) | 0.103 | 5.0 | 0.93–1.0 | final state following 5/6 |
| wake_from_sleep | 6 | 0 | 0 (0/6 runs) | 0.762 | 11.0 | 1.0–1.0 | followed after waking 6/6, median 3.2 s |

Wall time 592.3 s.

"Dog contacts" counts moves the rover started that would have hit the dog. "Min gap" includes the dog
walking up to the rover on its own (charge: the dog runs at it and stops ~3 cm away).

## Findings

1. **Never drove into the dog** in 36 runs, including a dog charging at it, lying close and off-centre,
   and walking past. Backing off can't outrun a running dog (0.30 m/s in 250 ms pulses with pauses),
   so close approaches by the dog itself still happen; the rover only ever retreats.
2. **Gets stuck on walls and keeps pushing (needs a fix).** It follows the dog into a doorway, catches
   a front corner on the frame and keeps sending the same arc: up to 79 blocked pulses over 85 s in
   one run (5 of 6 doorway runs bumped; 3 of 6 wedged), plus furniture/wall bumps in 3 of 6 living-room runs.
   The behaviour has no idea a move achieved nothing. On the real rover that is stalled motors
   grinding against the wall. Fix to try: after each pulse compare the frame before and after; if the
   view barely changed, count a blocked move; after two, back up straight and try the other side;
   after a few, stop and log "stuck".
3. **Search can't complete a full turn.** One arc + reverse pair rotates ~13° (A/D 350 ms, S 300 ms),
   so 360° takes ~28 pairs / ~80 s including the stop-and-look pauses, but the search gives up after
   60 s (~270°). Fix to try: longer scan arcs (e.g. 500 ms ≈ 19°/pair) or a 90 s search limit.
4. **Works as intended:** follows a walking dog keeping him in view 98–100% of the time at a median
   ~0.8 m gap; parks every time once he has been still 5 min and the model says sleeping (here
   ~235 s after he fell asleep, because he lay awake for 60 s first); finds a dog that dashed out of
   view in a median 5.8 s; starts following a median 3.2 s after a sleeping dog gets up.

## Limits

- Kinematic rover (no wheel slip, carpet, or battery sag); speed, steering angle and stop latency are
  estimates. Measure them after assembly (`RoverParams` in `rover_sim.py`) and rerun.
- The dog is scripted and simplified: capsule body, swinging legs, wagging tail; it walks around the
  rover rather than through it. YOLO is simulated from segmentation (misses more when he is small,
  cut off by the frame edge, or lying down). The vision model is an oracle with the watcher's cadence
  and 8 s latency, 10% "unclear".
- One room layout with one doorway.
