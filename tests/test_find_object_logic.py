# Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
# SPDX-License-Identifier: MIT-0
"""Offline logic tests for object-find (no robot, no AWS, no ROS2).

Reuses the greeter test harness's simulated world — a robot that only stops on
StopMove, and a detector that reports a delayed pose on an offset ROS clock —
and adds objects at arbitrary bearings so the rotate-and-scan sweep can be
exercised on a laptop. Run directly:

    python3 tests/test_find_object_logic.py
"""

import math
import sys
import threading
import time
import types
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

if "PIL" not in sys.modules:
    sys.modules["PIL"] = types.ModuleType("PIL")
    sys.modules["PIL"].Image = types.SimpleNamespace(
        frombytes=lambda *a, **k: None, open=lambda *a, **k: None
    )

from voice_agent import config  # noqa: E402
from voice_agent.go2_ros2_client import Go2ROS2Client  # noqa: E402

# Compress the timings; the sweep is otherwise minutes long.
config.GREETER_ENSURE_STANDING = False
config.GREETER_SETTLE_AFTER_TURN = 0.3
config.GREETER_MAX_TURN_TIME = 1.5
config.GREETER_FACE_TIMEOUT = 5.0
config.FIND_SETTLE_AFTER_STEP = 0.3
config.FIND_CONFIRM_FRAMES = 2
config.FIND_SCAN_STEP_DEGREES = 30.0
config.FIND_TURN_SPEED = 0.8

FRAME_W = 1280
STOP_MOVE = 1003
# The fake runs its detector faster than real hardware so sweeps finish quickly;
# capture lag is kept proportionally so staleness is still modelled.
DET_HZ = 10.0
CAPTURE_LAG = 0.1


class FakeGo2(Go2ROS2Client):
    """Go2ROS2Client with the bridge stubbed and a simulated world of objects.

    Preserves the two properties that made earlier tests misleading:
      * a zero Twist does NOT stop the robot (only StopMove does)
      * detections describe a slightly stale pose, stamped on an offset ROS clock

    `objects` maps a COCO class name to its true bearing in degrees relative to
    the robot's *initial* heading (positive = to the robot's right). Rotating the
    robot changes every object's apparent bearing together, which is what makes
    the sweep meaningful.
    """

    def __init__(self, objects=None, score=0.95, rate_factor=0.55,
                 publish_odom=True):
        super().__init__()
        self._connected = True
        self._image_width = FRAME_W
        self.twists = []
        self.sports = []
        self.objects = dict(objects or {})
        self.score = score
        self.heading = 0.0          # degrees the robot has actually turned
        self._commanded_yaw = 0.0
        # Fraction of the commanded angular rate the robot actually achieves.
        # Real hardware delivers well under 1.0 (accel ramp, foot slip, firmware
        # limiting), which is why timed open-loop turns undershot so badly: a
        # 12-step "360 deg" sweep measured only ~180-200 deg of real rotation.
        self.rate_factor = rate_factor
        self._publish_odom = publish_odom
        self._ros_offset = 1e6      # ROS clock deliberately != host clock
        self._history = []          # (capture_ts, heading)
        self._sim_stop = threading.Event()
        threading.Thread(target=self._integrate_motion, daemon=True).start()
        threading.Thread(target=self._publish_detections, daemon=True).start()

    # -- stubs --
    def _ensure_connected(self):
        pass

    def _publish_twist(self, vx, vy, vyaw):
        self.twists.append((vx, vy, vyaw))
        if vx == 0.0 and vy == 0.0 and vyaw == 0.0:
            return  # driver drops all-zero velocities
        self._commanded_yaw = vyaw

    def _publish_sport(self, api_id, parameter=None):
        self.sports.append(api_id)
        if api_id == STOP_MOVE:
            self._commanded_yaw = 0.0

    def _grab_live_frame(self, timeout=8.0):
        return None

    def _ensure_detection_subscribed(self):
        return True

    def _ensure_yaw_subscribed(self):
        return self._publish_odom

    def move(self, vx=0.0, vy=0.0, vyaw=0.0, duration=2.0):
        self.twists.append(("move", vx, vy, vyaw, duration))
        return {"status": "success", "action": "move"}

    # -- simulated physics --
    def _integrate_motion(self):
        dt = 0.02
        while not self._sim_stop.is_set():
            if self._commanded_yaw:
                # Achieved rate is only a fraction of what was commanded.
                self.heading += (math.degrees(self._commanded_yaw)
                                 * self.rate_factor * dt)
            self._history.append((time.time() + self._ros_offset, self.heading))
            if len(self._history) > 2000:
                del self._history[:1000]
            # Feed measured yaw back exactly as the /odom subscription would.
            if self._publish_odom:
                self._yaw = math.radians(self.heading)
            time.sleep(dt)

    def _apparent(self, true_bearing, heading):
        """Object bearing relative to the robot's current facing.

        ROS convention: positive angular.z is counter-clockwise (a left turn).
        Turning left makes a fixed object appear to move to the RIGHT of frame,
        so apparent bearing = true + heading. Getting this sign backwards makes
        the robot look like it is turning away from what it found.
        """
        return self._wrap(true_bearing + heading)

    def _heading_at(self, capture_ts):
        for ts, h in reversed(self._history):
            if ts <= capture_ts:
                return h
        return self._history[0][1] if self._history else 0.0

    def _publish_detections(self):
        while not self._sim_stop.is_set():
            capture_ts = time.time() + self._ros_offset - CAPTURE_LAG
            heading = self._heading_at(capture_ts)
            dets = []
            for name, true_bearing in self.objects.items():
                apparent = self._apparent(true_bearing, heading)
                px = FRAME_W * (0.5 + apparent / config.GREETER_CAMERA_HFOV)
                if 0 <= px <= FRAME_W:
                    dets.append({"class_id": name, "score": self.score,
                                 "center_x": px, "center_y": 360,
                                 "size_x": 120.0, "size_y": 120.0})
            with self._detections_lock:
                self._detections = dets
                self._detections_ts = time.time()
                self._detections_capture_ts = capture_ts
            time.sleep(1.0 / DET_HZ)

    @staticmethod
    def _wrap(deg):
        """Normalise to (-180, 180]."""
        return (deg + 180.0) % 360.0 - 180.0

    def stop_sim(self):
        self._sim_stop.set()
        self._commanded_yaw = 0.0

    # -- helpers --
    def apparent_bearing(self, name):
        if name not in self.objects:
            return None
        return self._apparent(self.objects[name], self.heading)

    @property
    def is_moving(self):
        return self._commanded_yaw != 0.0


def _full_sweep_ceiling(margin=1.5):
    """Worst-case wall-clock for a full 360deg sweep, from the constants above.

    Derived rather than hardcoded. The old fixed 60 s was only ~1.5x a *clean*
    run — a full sweep measures ~40 s here, because each 30 deg closed-loop step
    costs ~2.75 s (the turn decelerates into its target rather than stopping
    dead, which is the hardware behaviour `rate_factor` exists to model). That
    left so little headroom that a loaded machine tipped individual tests over
    the edge, failing a different one each run while the sweep was still visibly
    progressing.

    Every term below is the ceiling the production code itself enforces, so this
    can only expire if the sweep is genuinely stuck, never because it was slow.
    Deriving it also means retuning the compression constants above cannot
    silently reintroduce the same too-tight budget.
    """
    step = min(config.FIND_SCAN_STEP_DEGREES,
               config.GREETER_CAMERA_HFOV * config.FIND_STEP_FOV_FRACTION)
    steps = math.ceil(config.FIND_MAX_SWEEP_DEGREES / step) + 1
    # turn_degrees' own deadline: (target / speed) * allowance + 2 s.
    speed = min(config.FIND_TURN_SPEED, config.MAX_ANGULAR_VELOCITY)
    per_turn = (math.radians(step) / speed) * config.TURN_TIME_ALLOWANCE + 2.0
    # _confirm_target's own deadline: FIND_CONFIRM_FRAMES * 2 s.
    per_confirm = config.FIND_CONFIRM_FRAMES * 2.0
    per_step = per_turn + config.FIND_SETTLE_AFTER_STEP + per_confirm
    return (steps * per_step + per_confirm) * margin


# One budget for every sweep. Tests return as soon as the state goes terminal,
# so a passing run is no slower for this being generous — it only bounds a hang.
SWEEP_TIMEOUT = _full_sweep_ceiling()


def wait_for_find(client, timeout=None):
    """Block until the search reaches a terminal state; return the snapshot."""
    budget = SWEEP_TIMEOUT if timeout is None else timeout
    deadline = time.time() + budget
    while time.time() < deadline:
        snap = client.get_find_status()["find"]
        if snap.get("state") in ("found", "not_found", "stopped", "error"):
            return snap
        time.sleep(0.05)
    raise AssertionError(
        f"search never finished within {budget:.0f}s: {client.get_find_status()}"
    )


# ---------------------------------------------------------------------------
# Target resolution (pure, no robot)
# ---------------------------------------------------------------------------

def test_resolves_plain_coco_class():
    assert config.resolve_find_target("backpack") == "backpack"
    assert config.resolve_find_target("laptop") == "laptop"


def test_resolves_natural_language():
    assert config.resolve_find_target("my backpack") == "backpack"
    assert config.resolve_find_target("the bag") == "backpack"
    assert config.resolve_find_target("phone") == "cell phone"
    assert config.resolve_find_target("  Couch  ") == "couch"


def test_resolves_plurals():
    assert config.resolve_find_target("bottles") == "bottle"
    assert config.resolve_find_target("cats") == "cat"


def test_rejects_unfindable_targets():
    for bad in ("unicorn", "screwdriver", "", "my feelings"):
        assert config.resolve_find_target(bad) is None, bad


def test_every_alias_maps_to_a_real_coco_class():
    """A typo here would make a search silently never match anything."""
    bad = {k: v for k, v in config.FIND_ALIASES.items()
           if v not in config.COCO_CLASSES}
    assert not bad, f"aliases pointing at non-COCO classes: {bad}"


def test_unfindable_target_errors_without_moving():
    c = FakeGo2({"backpack": 40.0})
    try:
        result = c.find_object("unicorn")
        assert result["status"] == "error", result
        assert "unicorn" in result["note"]
        assert not c.twists, "must not move when the target is unfindable"
    finally:
        c.stop_sim()


# ---------------------------------------------------------------------------
# Finding
# ---------------------------------------------------------------------------

def test_finds_object_already_in_view_without_sweeping():
    c = FakeGo2({"backpack": 10.0})
    try:
        started = c.find_object("my backpack")
        assert started["status"] == "success"
        assert started["target"] == "backpack"
        snap = wait_for_find(c)
        assert snap["state"] == "found", snap
        assert snap["swept_degrees"] == 0, "no sweep needed when already in view"
        assert snap["direction"] == "straight ahead", snap
    finally:
        c.stop_find()
        c.stop_sim()


def test_finds_object_behind_the_robot_by_sweeping():
    c = FakeGo2({"backpack": -120.0})   # off to the robot's left, out of frame
    try:
        c.find_object("backpack")
        snap = wait_for_find(c)
        assert snap["state"] == "found", snap
        assert snap["swept_degrees"] > 0, "should have had to sweep"
        assert snap["found"] is True
        # Ends up pointing at it.
        assert abs(c.apparent_bearing("backpack")) < 20, \
            f"should be facing the backpack, apparent {c.apparent_bearing('backpack'):.1f}deg"
    finally:
        c.stop_find()
        c.stop_sim()


def test_reports_not_found_after_full_sweep():
    c = FakeGo2({})   # empty room
    try:
        c.find_object("backpack")
        snap = wait_for_find(c)
        assert snap["state"] == "not_found", snap
        assert snap["found"] is False
        assert snap["swept_degrees"] >= config.FIND_MAX_SWEEP_DEGREES, snap
    finally:
        c.stop_find()
        c.stop_sim()


def test_sweep_is_bounded():
    """Must not spin forever hunting something that isn't there."""
    c = FakeGo2({})
    try:
        c.find_object("backpack")
        snap = wait_for_find(c)
        assert snap["swept_degrees"] <= config.FIND_MAX_SWEEP_DEGREES + \
            config.FIND_SCAN_STEP_DEGREES, snap
    finally:
        c.stop_find()
        c.stop_sim()


def test_ignores_other_objects_while_searching():
    c = FakeGo2({"chair": 5.0, "backpack": -90.0})
    try:
        c.find_object("backpack")
        snap = wait_for_find(c)
        assert snap["state"] == "found", snap
        assert snap["target"] == "backpack"
        # A chair sitting dead ahead must not have ended the search early.
        assert snap["swept_degrees"] > 0, \
            "should have swept past the chair to reach the backpack"
    finally:
        c.stop_find()
        c.stop_sim()


def test_low_confidence_detection_is_not_a_find():
    c = FakeGo2({"backpack": 10.0}, score=0.2)  # below FIND_MIN_SCORE
    try:
        c.find_object("backpack")
        snap = wait_for_find(c)
        assert snap["state"] == "not_found", snap
    finally:
        c.stop_find()
        c.stop_sim()


# ---------------------------------------------------------------------------
# Motion safety
# ---------------------------------------------------------------------------

def test_robot_is_halted_when_search_ends():
    c = FakeGo2({"backpack": -60.0})
    try:
        c.find_object("backpack")
        wait_for_find(c)
        assert not c.is_moving, "robot must be stopped when the search ends"
        assert STOP_MOVE in c.sports, "must issue StopMove, not just a zero Twist"
    finally:
        c.stop_find()
        c.stop_sim()


def test_search_does_not_translate_by_default():
    """FIND_APPROACH defaults off — searching must never walk."""
    assert config.FIND_APPROACH is False, "approach must be opt-in"
    c = FakeGo2({"backpack": -60.0})
    try:
        c.find_object("backpack")
        wait_for_find(c)
        bad = [t for t in c.twists
               if t and t[0] == "move" or (len(t) == 3 and (t[0] or t[1]))]
        assert not bad, f"search must rotate only, got {bad}"
    finally:
        c.stop_find()
        c.stop_sim()


def test_stop_find_halts_mid_sweep():
    c = FakeGo2({})   # nothing to find, so it keeps sweeping
    try:
        c.find_object("backpack")
        time.sleep(1.0)   # let it get going
        result = c.stop_find()
        assert result["was_searching"] is True, result
        assert not c.is_moving, "must halt on stop_find"
        assert c.get_find_status()["find"]["state"] == "stopped"
    finally:
        c.stop_sim()


def test_find_stops_a_running_greeter():
    """Both rotate the robot; they must not fight over /cmd_vel_out."""
    c = FakeGo2({"backpack": 10.0})
    try:
        c.start_greeter()
        assert c._greeter_active
        c.find_object("backpack")
        assert not c._greeter_active, "starting a search must stop greeter mode"
        wait_for_find(c)
    finally:
        c.stop_find()
        c.stop_greeter()
        c.stop_sim()


def test_find_stops_the_tracker():
    c = FakeGo2({"backpack": 10.0})
    try:
        c._tracking_target = "person"
        c._tracking_thread = None
        c.find_object("backpack")
        assert c._tracking_target == "", "starting a search must stop tracking"
        wait_for_find(c)
    finally:
        c.stop_find()
        c.stop_sim()


def test_second_find_replaces_the_first():
    c = FakeGo2({})
    try:
        c.find_object("backpack")
        time.sleep(0.5)
        second = c.find_object("chair")
        assert second["status"] == "success"
        assert c.get_find_status()["find"]["target"] == "chair"
    finally:
        c.stop_find()
        c.stop_sim()


# ---------------------------------------------------------------------------
# Closed-loop rotation (the "only did 180-200 deg, not 360" bug)
# ---------------------------------------------------------------------------

def test_turn_degrees_achieves_the_requested_angle():
    """The robot under-delivers commanded rate; the turn must compensate."""
    c = FakeGo2({}, rate_factor=0.5)   # achieves half the commanded rate
    try:
        result = c.turn_degrees(90.0)
        assert result["closed_loop"] is True, result
        assert abs(result["measured_degrees"] - 90.0) <= 8.0, \
            f"asked 90deg, measured {result['measured_degrees']}deg"
        assert abs(c.heading - 90.0) <= 8.0, \
            f"simulated robot only turned {c.heading:.1f}deg"
    finally:
        c.stop_sim()


def test_turn_degrees_handles_both_directions():
    c = FakeGo2({}, rate_factor=0.6)
    try:
        c.turn_degrees(45.0)
        assert c.heading > 30, f"left turn should increase heading: {c.heading:.1f}"
        before = c.heading
        c.turn_degrees(-45.0)
        assert c.heading < before - 30, \
            f"right turn should decrease heading: {before:.1f} -> {c.heading:.1f}"
    finally:
        c.stop_sim()


def test_full_sweep_actually_covers_360_degrees():
    """Regression: counting commanded steps stopped the sweep after ~180 deg."""
    c = FakeGo2({}, rate_factor=0.55)   # matches observed hardware shortfall
    try:
        c.find_object("backpack")       # nothing to find → full sweep
        snap = wait_for_find(c)
        assert snap["state"] == "not_found", snap
        # The robot must have physically rotated ~360deg, not ~200.
        assert abs(c.heading) >= 330, \
            f"only physically turned {abs(c.heading):.0f}deg — sweep is short"
        assert snap["closed_loop"] is True, snap
    finally:
        c.stop_find()
        c.stop_sim()


def test_sweep_reports_measured_not_commanded_degrees():
    c = FakeGo2({}, rate_factor=0.5)
    try:
        c.find_object("backpack")
        time.sleep(4.0)
        snap = c.get_find_status()["find"]
        swept = snap.get("swept_degrees", 0)
        # Reported sweep should track physical heading, not command count.
        assert abs(swept - abs(c.heading)) < 25, \
            f"reported {swept}deg but physically turned {abs(c.heading):.0f}deg"
    finally:
        c.stop_find()
        c.stop_sim()


def test_aborts_when_robot_does_not_rotate():
    """A robot that ignores velocity (e.g. not standing) must not loop forever."""
    c = FakeGo2({}, rate_factor=0.0)   # commands have no effect
    try:
        c.find_object("backpack")
        # Deliberately well under SWEEP_TIMEOUT: the stall must be caught on the
        # first step, so this failing on time is itself the assertion.
        snap = wait_for_find(c, timeout=40)
        assert snap["state"] == "error", snap
        assert "turning" in snap["note"].lower(), snap
    finally:
        c.stop_find()
        c.stop_sim()


def test_falls_back_to_open_loop_without_odom():
    """No /odom must degrade gracefully, not break the search."""
    c = FakeGo2({}, rate_factor=0.55, publish_odom=False)
    try:
        result = c.turn_degrees(45.0)
        assert result["closed_loop"] is False, result
        assert result["measured_degrees"] is None, result
        assert "open-loop" in result["note"]
        assert c.heading > 0, "should still have turned, just imprecisely"
    finally:
        c.stop_sim()


# ---------------------------------------------------------------------------
# Speaking up when the search ends
# ---------------------------------------------------------------------------

def test_announces_when_object_is_found():
    """Regression: the robot found things silently and had to be asked."""
    from voice_agent import notifications
    notifications.drain()
    c = FakeGo2({"backpack": -40.0})
    try:
        c.find_object("backpack")
        snap = wait_for_find(c)
        assert snap["state"] == "found", snap
        msgs = notifications.drain()
        assert msgs, "finding the object must queue an announcement"
        joined = " ".join(msgs).lower()
        assert "backpack" in joined and "found" in joined, msgs
    finally:
        c.stop_find()
        c.stop_sim()
        notifications.drain()


def test_announces_when_object_is_not_found():
    from voice_agent import notifications
    notifications.drain()
    c = FakeGo2({}, rate_factor=0.6)
    try:
        c.find_object("backpack")
        snap = wait_for_find(c)
        assert snap["state"] == "not_found", snap
        msgs = notifications.drain()
        joined = " ".join(msgs).lower()
        assert msgs and "not" in joined or "couldn" in joined, msgs
        assert "backpack" in joined, msgs
    finally:
        c.stop_find()
        c.stop_sim()
        notifications.drain()


def test_stopped_search_does_not_announce():
    """Cancelling is a user action — the robot shouldn't narrate it."""
    from voice_agent import notifications
    c = FakeGo2({}, rate_factor=0.6)
    try:
        c.find_object("backpack")
        time.sleep(1.5)
        notifications.drain()      # clear anything from before the stop
        c.stop_find()
        time.sleep(0.5)
        assert not notifications.drain(), "stopping should not queue an announcement"
    finally:
        c.stop_sim()
        notifications.drain()


# ---------------------------------------------------------------------------
# Status reporting
# ---------------------------------------------------------------------------

def test_status_is_idle_before_any_search():
    c = FakeGo2({})
    try:
        assert c.get_find_status()["find"]["state"] == "idle"
    finally:
        c.stop_sim()


def test_status_reports_progress_while_searching():
    c = FakeGo2({})
    try:
        c.find_object("backpack")
        snap = c.get_find_status()["find"]
        assert snap["state"] == "searching", snap
        assert snap["target"] == "backpack"
        assert snap["requested"] == "backpack"
        # Give it long enough to complete at least one scan step.
        deadline = time.time() + 6.0
        while time.time() < deadline:
            snap = c.get_find_status()["find"]
            if snap.get("swept_degrees", 0) > 0:
                break
            time.sleep(0.05)
        assert snap["swept_degrees"] > 0, f"sweep should progress: {snap}"
    finally:
        c.stop_find()
        c.stop_sim()


def test_direction_words_cover_the_circle():
    d = Go2ROS2Client._describe_direction
    assert d(0) == "straight ahead"
    assert d(359) == "straight ahead"
    assert d(90) == "to my left"
    assert d(180) == "behind me"
    assert d(270) == "to my right"
    # Every angle must produce something speakable.
    for deg in range(0, 360, 7):
        assert isinstance(d(deg), str) and d(deg)


def main() -> int:
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    failures = 0
    for fn in tests:
        try:
            fn()
        except AssertionError as e:
            failures += 1
            print(f"FAIL {fn.__name__}: {e}")
        except Exception as e:  # noqa: BLE001
            failures += 1
            print(f"ERROR {fn.__name__}: {type(e).__name__}: {e}")
        else:
            print(f"pass {fn.__name__}")
    print(f"\n{len(tests) - failures}/{len(tests)} passed")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
