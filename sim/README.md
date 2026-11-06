# Simulation

## The apartment (2026-11-04)

The rover in a copy of the real apartment, behind the same interfaces as the real one, so the real
code runs unchanged: `apartment_sim.py` serves the real `drive/drive_server.py` (with a simulated
transmitter and heading stick) on :9769 and the rendered rover camera on :9010; the real
`mac/depth_service.py` and `mac/explore.py` point at it.

- `apartment/build_apartment.py` builds `apartment/apartment.xml` + `.json` from the floor plan
  (`apartment/floorplan.png`; thick lines = walls, 13.1 px/ft from the 12' x 12' bedroom; the
  balcony slider is its own "blinds" geom). The floor plan, robot-vacuum map, photos and generated
  apartment files are personal and stay local (gitignored); only the code is in git.
- Rover: measured speeds and turning radius, the receiver's key rules (rover app 0.9), servo lag,
  front-wheel play, gyro noise and bias; collisions stop it and count as bumps. Camera 7.5 cm up,
  3.8° up, 62° wide: the floor-wall line at 0.5 / 1.1 / 2.3 m lands within 2 px of the real
  rover's fitted geometry.
- `GET :9010/truth`: true pose, collisions, time pushing a wall, path length, closest approach.
- **Edit it on the dashboard**: `http://localhost:8010/sim` (Sim in the header). Top-down plan over
  the floor plan: drag furniture to move, the corner handle to resize, arrow keys nudge 5 cm,
  rotate / duplicate / delete / add, undo; 3D view to orbit. Save writes `apartment/furniture.json`
  (local) and rebuilds the simulator's apartment.
- Furniture and floor textures come from photos (`apartment/photos/`, local); exact ray depth on
  :9775 for testing driving decisions; `batch_explore.py` runs many episodes in parallel and scores
  them. Same 8 starts (seed 3) on the corrected apartment with virtual doors:

  | Explorer | Runs finishing 40 steps | Driven | Collisions | Door touches |
  |---|---|---|---|---|
  | first version (timed-out turns end the run) | 1 of 8 | 39 m | 1 | – |
  | turns that got most of the way count; arcs at the measured 37 cm/s | 4 of 8 | 90 m | 5 | 2 |
  | multi-point turns back up straight, turn forward | **8 of 8** | **101 m** | **0** | 8 (one run) |

  Every collision in the middle row was the rear corner swinging into something unseen during a
  reverse arc (vanity, boxes), which is why turns now back up straight over the path they came in on.

```sh
mac/.vision-venv/bin/python sim/apartment/build_apartment.py
sim/.venv/bin/python sim/apartment_sim.py --start 2.0,4.5,90 --seed 1 &
<detector python> mac/depth_service.py --port 9774 --source http://127.0.0.1:9010/live.jpg --drive http://127.0.0.1:9769 &
python3 mac/explore.py --steps 40 --drive http://127.0.0.1:9769 --depth http://127.0.0.1:9774 --no-capture
curl -s 127.0.0.1:9010/truth
```

First run: 17 steps, 4.8 m, no collisions, closest 20 cm, two multi-point turns, 9.2 m² mapped.
It also showed the depth model reading plain untextured sim surfaces badly (scale check 0.27 vs
1.00 at home), so realistic floor and wall textures and furniture come next (from photos and the
robot-vacuum map).

## The first living-room sim (dog companion, 2026-10)

Closed-loop tests of the real dog-companion code (`mac/dog_behavior.py`, `mac/drive_client.py`) in a
MuJoCo living room, before the chassis exists. The rover is a kinematic car (front-wheel steering,
W/A/S/D pulses through the same drive client), the dog follows scripts (walk, run, stand and wag, lie
down, sleep, charge), and the rover camera is rendered with the dog's pixels from segmentation, so
the behaviour sees the same inputs the watcher gives it: box centre and height in percent, and the
in-box pixel change at 160 × 120. Results: [RESULTS.md](RESULTS.md).

| File | What |
| --- | --- |
| `rover.xml` | Scene: 5 × 4 m room, doorway to a hall, sofa, table, rug, rover with its camera, dog. |
| `rover_sim.py` | World: rover kinematics (`RoverParams`, estimates until measured), dog scripts, collisions, camera + simulated YOLO. |
| `run_scenarios.py` | Scenario matrix in closed loop; writes `sim/out/results.json` (gitignored). |
| `summarize.py` | Table of the results (used for RESULTS.md). |
| `test_sim.py` | Checks of the simulator: pulses, arcs, walls, the dog as an obstacle, camera geometry, still vs moving pixel change. |

```sh
# setup once (Python 3.12)
python3.12 -m venv sim/.venv && sim/.venv/bin/pip install -r sim/requirements.txt
sim/.venv/bin/python -m unittest discover -s sim -p 'test_*.py'
sim/.venv/bin/python sim/run_scenarios.py --seeds 6           # ~10 min on 8 cores
sim/.venv/bin/python sim/run_scenarios.py --only leave_room --seeds 1 --trace   # per-frame trace
python3 sim/summarize.py
```

Scenarios: `walk_then_sleep` (follow, then park when he sleeps), `hide_to_side` (dashes out of view),
`leave_room` (walks out the door and down the hall), `charge` (runs at the rover), `close_off_centre`
(lying close beside it), `wake_from_sleep` (asleep in view, gets up and walks off).

After assembly: measure speed per pulse, turning radius at full A/D and the stop latency, put them in
`RoverParams`, and rerun to retune pulse lengths and the search time.
