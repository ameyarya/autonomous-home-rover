"""Dog-companion behaviour: decide what the rover should do from what the camera sees.

Pure logic (no camera, model or motor code) so it can be tested directly.

States:
  parked     rover still, watching. Dog movement -> following.
  following  keep the dog centred at a standoff distance. Dog asleep -> parked;
             dog out of view for lost_after_s -> searching.
  searching  scan in place. Dog found -> following; nothing for search_limit_s -> parked.

Blocked moves: the caller reports after each pulse whether the camera view changed
(note_move). Two blocked moves in a row -> back up straight; stuck_give_up_after -> park and
wait stuck_wait_s (the rover is wedged on a wall or furniture and pushing achieves nothing).

Motion is only judged while the rover itself is stopped (the caller passes rover_moving);
camera ego-motion would otherwise look like the dog moving.

The rover steers like a car (front-wheel steering, no spinning in place) and the receiver only
takes W/A/S/D/X, so there is no reverse-with-steering. Turning uses forward arcs: toward the dog
when there is room, after backing up straight if he is too close. Searching rotates roughly in
place by alternating a forward arc and a straight reverse (drive_client.py does the alternating).
"""
import collections
from dataclasses import dataclass, field


@dataclass
class Settings:
    sleep_after_s: float = 300.0       # dog still this long + model says sleeping -> park
    resting_factor: float = 2.0        # or still sleep_after_s * this + model says resting
    lost_after_s: float = 3.0
    search_limit_s: float = 75.0       # a full scan turn takes ~60 s (19 deg per arc + reverse pair)
    stuck_back_after: int = 2          # blocked moves in a row before backing out
    stuck_give_up_after: int = 5       # ... before stopping
    stuck_wait_s: float = 30.0
    no_reverse_after_blocked_s: float = 20.0   # after a blocked reverse, don't reverse for this long
    center_tolerance: float = 10.0     # percent of image width either side of centre
    standoff_height: tuple = (30.0, 55.0)  # dog box height, percent of image height
    charge_rate: float = 25.0          # box height growth, percent/s, that counts as charging
    move_speed: float = 0.3            # box centre speed, box-heights per second
    move_pixels: float = 12.0          # mean abs pixel change inside the box, 0-255
    motion_window_s: float = 2.0
    still_confirm_s: float = 2.0       # no movement evidence this long before calling him still


@dataclass
class Dog:
    x: float           # box centre, percent of image width
    y: float           # box centre, percent of image height
    height: float      # box height, percent of image height
    confidence: float
    pixel_change: float | None = None  # mean abs grey-level change inside the box vs previous frame


@dataclass
class Decision:
    state: str
    action: str        # hold | turn_left | turn_right (forward arcs) | forward | back | scan_left | scan_right | park | stop
    reason: str


class MotionTracker:
    """Moving/still from box displacement and in-box pixel change over a short window.

    Hysteresis: movement evidence switches to moving at once; switching back to still
    needs still_confirm_s without evidence, so readings near a threshold don't flicker.
    """

    def __init__(self, settings):
        self.settings = settings
        self.samples = collections.deque()
        self.still_since = None
        self.last_motion = None
        self.moving = False
        self.speed = 0.0
        self.approach_rate = 0.0

    def reset(self, now):
        self.samples.clear()
        self.still_since = now
        self.last_motion = None
        self.moving = False

    def update(self, now, dog):
        s = self.settings
        self.samples.append((now, dog))
        while self.samples and now - self.samples[0][0] > s.motion_window_s:
            self.samples.popleft()
        first_t, first = self.samples[0]
        span = now - first_t
        if span > 0.2:
            scale = max(dog.height, 1.0)
            moved = ((dog.x - first.x) ** 2 + (dog.y - first.y) ** 2) ** 0.5
            self.speed = moved / scale / span
            self.approach_rate = (dog.height - first.height) / span
        else:
            self.speed = self.approach_rate = 0.0
        recent_pixels = [d.pixel_change for _, d in self.samples if d.pixel_change is not None]
        busy = sum(p > s.move_pixels for p in recent_pixels)
        if self.speed > s.move_speed or busy >= 2:
            self.last_motion = now
        if self.last_motion is not None and now - self.last_motion < s.still_confirm_s:
            self.moving = True
            self.still_since = None
        else:
            self.moving = False
            if self.still_since is None:
                self.still_since = self.last_motion if self.last_motion is not None else now
        return self.moving

    def still_for(self, now):
        return 0.0 if self.still_since is None else now - self.still_since


class ViewChange:
    """Was a move blocked? Compares the whole-frame change across a pulse with the camera's own
    noise (frame-to-frame change while the rover is still), so the threshold suits the real camera.
    Reversing straight away from a scene changes it least, so reverse gets a lower threshold."""
    # Tuned in the simulator (sim/RESULTS.md): free moves changed the view >= 3.0x the noise,
    # blocked ones <= 2.2x. Recheck on the real camera ('blocked' events in the timeline).
    FACTOR = 2.6
    REVERSE_FACTOR = 2.4
    DEFAULT_NOISE = 1.2     # grey levels at 160x120, until still frames have been seen

    def __init__(self):
        self.still = collections.deque(maxlen=60)

    def note_still(self, change):
        self.still.append(change)

    def noise(self):
        if len(self.still) < 5:
            return self.DEFAULT_NOISE
        return sorted(self.still)[len(self.still) // 2]

    def threshold(self, key='W'):
        return self.noise() * (self.REVERSE_FACTOR if key == 'S' else self.FACTOR)

    def blocked(self, key, change):
        """Whether a pulse with this key was blocked, judged by the view change across it."""
        return change < self.threshold(key)


class Behavior:
    def __init__(self, settings=None):
        self.settings = settings or Settings()
        self.motion = MotionTracker(self.settings)
        self.state = 'parked'
        self.last_seen = None
        self.search_started = None
        self.sleep_reason = None
        self.last_x = 50.0         # where the dog was last seen, to scan toward that side
        self.blocked = 0           # consecutive forward/turn moves that did not change the camera view
        self.back_blocked_at = None   # when a reverse move was last blocked
        self.now = 0.0
        self.stuck_until = None

    def note_move(self, blocked, key='W'):
        """Report whether the last pulse (key W/A/S/D) was blocked; None = not judged."""
        if blocked is None:
            return
        if key == 'S':
            if blocked:
                self.back_blocked_at = self.now
        else:
            self.blocked = self.blocked + 1 if blocked else 0

    def blocked_behind(self, now):
        at = self.back_blocked_at
        return at is not None and now - at < self.settings.no_reverse_after_blocked_s

    def asleep(self, now, model_state):
        still = self.motion.still_for(now)
        s = self.settings
        if model_state == 'sleeping' and still >= s.sleep_after_s:
            return f'still {still / 60:.1f} min and model says sleeping'
        if model_state in ('sleeping', 'resting') and still >= s.sleep_after_s * s.resting_factor:
            return f'still {still / 60:.1f} min and model says {model_state}'
        return None

    def _go(self, state, now):
        self.state = state
        if state == 'searching':
            self.search_started = now

    def update(self, now, dog, model_state=None, camera_ok=True, rover_moving=False):
        self.now = now
        decision = self._update(now, dog, model_state, camera_ok, rover_moving)
        if decision.action == 'back' and self.blocked_behind(now):
            if self.blocked >= self.settings.stuck_back_after:
                # Wedged both ways: stop pushing.
                self.blocked, self.back_blocked_at = 0, None
                self._go('parked', now)
                self.stuck_until = now + self.settings.stuck_wait_s
                return Decision('parked', 'park', 'stuck: blocked ahead and behind; stopped')
            return Decision(decision.state, 'hold', decision.reason + '; but blocked behind, holding')
        return decision

    def _update(self, now, dog, model_state, camera_ok, rover_moving):
        s = self.settings
        if not camera_ok:
            return Decision(self.state, 'stop', 'camera stale; not acting on old frames')

        if dog is not None:
            self.last_seen = now
            self.last_x = dog.x
            if not rover_moving:
                self.motion.update(now, dog)

        if self.stuck_until is not None:
            if now < self.stuck_until:
                return Decision('parked', 'hold', f'stuck; waiting {self.stuck_until - now:.0f} s before trying again')
            self.stuck_until = None
        if self.state != 'parked' and self.blocked >= s.stuck_give_up_after:
            n, self.blocked = self.blocked, 0
            self._go('parked', now)
            self.stuck_until = now + s.stuck_wait_s
            return Decision('parked', 'park', f'stuck: {n} moves in a row changed nothing; stopped')
        if self.state != 'parked' and self.blocked >= s.stuck_back_after:
            return Decision(self.state, 'back', f'blocked {self.blocked} moves in a row; backing out')

        if self.state == 'parked':
            if dog is not None and self.motion.moving:
                self._go('following', now)
                return self._follow(dog, 'dog moving')
            if dog is None:
                return Decision('parked', 'hold', 'parked; dog not in view')
            return Decision('parked', 'hold', f'parked; dog still {self.motion.still_for(now):.0f} s')

        if self.state == 'following':
            if dog is None:
                if self.last_seen is not None and now - self.last_seen > s.lost_after_s:
                    self._go('searching', now)
                    return Decision('searching', self._scan_action(), 'dog out of view; scanning ' + self._scan_side())
                return Decision('following', 'hold', 'dog briefly out of view')
            reason = self.asleep(now, model_state)
            if reason:
                self._go('parked', now)
                self.sleep_reason = reason
                return Decision('parked', 'park', 'dog asleep: ' + reason)
            return self._follow(dog, 'following')

        # searching
        if dog is not None:
            self._go('following', now)
            self.motion.reset(now)
            return self._follow(dog, 'dog found')
        if now - self.search_started > s.search_limit_s:
            self._go('parked', now)
            return Decision('parked', 'park', f'dog not found in {s.search_limit_s:.0f} s; parked to wait')
        return Decision('searching', self._scan_action(), 'scanning ' + self._scan_side())

    def _scan_side(self):
        return 'left' if self.last_x < 50 else 'right'

    def _scan_action(self):
        return 'scan_' + self._scan_side()

    def _follow(self, dog, why):
        s = self.settings
        if self.motion.approach_rate > s.charge_rate:
            return Decision('following', 'back', f'{why}: dog closing fast; backing off')
        low, high = s.standoff_height
        offset = dog.x - 50
        if dog.height > high:
            # Too close: back up straight first; a forward arc would drive into him.
            return Decision('following', 'back', f'{why}: dog close ({dog.height:.0f}% tall)')
        if offset < -s.center_tolerance:
            return Decision('following', 'turn_left', f'{why}: dog at {dog.x:.0f}% (left); arc left')
        if offset > s.center_tolerance:
            return Decision('following', 'turn_right', f'{why}: dog at {dog.x:.0f}% (right); arc right')
        if dog.height < low:
            return Decision('following', 'forward', f'{why}: dog small ({dog.height:.0f}% tall)')
        return Decision('following', 'hold', f'{why}: dog centred at standoff')
