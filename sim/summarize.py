"""Summarise sim/out/results.json into a table per scenario. Run: python3 sim/summarize.py"""
import json
import statistics
import sys
from pathlib import Path

path = Path(sys.argv[1]) if len(sys.argv) > 1 else Path(__file__).with_name('out') / 'results.json'
r = json.loads(path.read_text())
by = {}
for e in r['episodes']:
    by.setdefault(e['scenario'], []).append(e)


def med(values):
    values = [v for v in values if v is not None]
    return round(statistics.median(values), 2) if values else '—'


def count(values):
    return f'{sum(bool(v) for v in values)}/{len(values)}'


s = r['scan_rotation']
print(f"Scan (arc + reverse pair): {s['deg_per_pair']}° per pair, {s['pairs_for_360']} pairs / ~{s['seconds_for_360']} s "
      f"for 360°, footprint {s['footprint_m']} m. Search limit: {r['behavior_settings']['search_limit_s']} s.\n")
print('| Scenario | Runs | Dog contacts | Wall/furniture bumps | Min gap to dog (m, worst) | Pulses (median) | In view while following | Scenario check |')
print('| --- | --- | --- | --- | --- | --- | --- | --- |')
for name, eps in by.items():
    check = ''
    if name == 'walk_then_sleep':
        check = f"parked after sleep in {count([e.get('park_after_sleep_s') for e in eps])}, median {med([e.get('park_after_sleep_s') for e in eps])} s after he fell asleep; standoff gap median {med([e.get('standoff_gap_m') for e in eps])} m"
    elif name in ('hide_to_side', 'leave_room'):
        check = f"searched {count([e['searched'] for e in eps])}, found {count([e.get('found_after_s') for e in eps])} (median {med([e.get('found_after_s') for e in eps])} s), parked after search {count([e.get('parked_after_search') for e in eps])}"
    elif name == 'charge':
        check = f"backed off {count([e.get('backed_off') for e in eps])}"
    elif name == 'wake_from_sleep':
        check = f"followed after waking {count([e.get('follow_after_wake_s') for e in eps])}, median {med([e.get('follow_after_wake_s') for e in eps])} s"
    elif name == 'close_off_centre':
        check = f"final state following {count([e['final_state'] == 'following' for e in eps])}"
    views = [e['in_view_while_following'] for e in eps if e['in_view_while_following'] is not None]
    print(f"| {name} | {len(eps)} | {sum(e['dog_contacts'] for e in eps)} | {sum(e['bumps'] for e in eps)} "
          f"({count([e['bumps'] for e in eps])} runs) | {min(e['min_gap_m'] for e in eps)} | {med([e['pulses'] for e in eps])} | "
          f"{round(min(views), 2) if views else '—'}–{round(max(views), 2) if views else '—'} | {check} |")
print(f"\nWall time {r.get('wall_time_s')} s.")
