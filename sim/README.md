# Simulation

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
