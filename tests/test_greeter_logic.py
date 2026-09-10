# Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
# SPDX-License-Identifier: MIT-0
"""Offline logic tests for greeter mode (no robot, no AWS, no ROS2).

Subclasses Go2ROS2Client with the transport and vision calls stubbed out, so
the greeter's decision-making — person debounce, which way it turns, one wave
per visitor, re-arming after they leave — can be checked on a laptop before
taking the dog out. Run directly:

    python3 tests/test_greeter_logic.py
"""

import math
import sys
import threading
import time
import types
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

# go2_ros2_client imports PIL at module scope for camera decoding; the greeter
# paths under test never touch it, so a stub keeps this runnable bare.
if "PIL" not in sys.modules:
    sys.modules["PIL"] = types.ModuleType("PIL")
    sys.modules["PIL"].Image = types.SimpleNamespace(
        frombytes=lambda *a, **k: None, open=lambda *a, **k: None
    )

from voice_agent import config  # noqa: E402
from voice_agent.go2_ros2_client import Go2ROS2Client  # noqa: E402

# Shrink every timing constant so the suite runs in seconds, not minutes.
config.GREETER_DESCRIBE_VISITOR = False  # no Bedrock call in tests
config.GREETER_POLL_INTERVAL = 0.05
config.GREETER_CONFIRM_POLLS = 2
# Left at the production default (1.0 s per required frame): the fake models the
# same ~0.5 s capture lag + ~0.5 s detector period as the real pipeline, so a
# shorter window here would only prove the tests can outrun real perception.
config.GREETER_RESET_AFTER = 0.3
config.GREETER_WAVE_SETTLE = 0.05
config.GREETER_ENSURE_STANDING = False  # skip BalanceStand in tests
config.GREETER_MAX_TURN_TIME = 1.5
config.GREETER_SETTLE_AFTER_TURN = 0.6  # must exceed the fake's capture lag
config.GREETER_PRE_WAVE_PAUSE = 0.05
# GREETER_MAX_CORRECTIONS and GREETER_FACE_TIMEOUT are left at their production
# values on purpose: the correction budget is tuned against GREETER_TURN_GAIN, so
# overriding it here would test a convergence behaviour the robot never uses.

FRAME_W = 1280
GREET_SECONDS = 6.0  # wall-clock allowance for one full greeting (turn + wave)


STOP_MOVE = 1003     # sport API id the driver actually forwards
BALANCE_STAND = 1002


class FakeGo2(Go2ROS2Client):
    """Go2ROS2Client with the foxglove bridge and vision model stubbed out.

    Deliberately reproduces the two real-world behaviours that made an earlier
    version pass its tests while failing on hardware:

    1. **Zero Twists do not stop the robot.** The driver filters all-zero
       velocities (robot_control_service.handle_cmd_vel), so the robot holds its
       last non-zero yaw and coasts until an explicit StopMove arrives. The
       simulated robot here does the same — it keeps rotating in a background
       thread until StopMove.
    2. **Detections describe the past.** coco_detector runs at ~2 Hz plus CPU
       inference, so the array we receive was captured up to ~0.5 s earlier.
       The fake keeps a bearing history and publishes the delayed one, stamping
       it with that capture time.

    Without both of these, "the stop is a no-op" and "it corrects against a
    stale pose" are invisible to the suite.
    """

    def __init__(self, detection_hz=2.0, capture_lag=0.5, ros_clock_offset=1e6,
                 rate_factor=0.55, true_hfov=None, coast_seconds=0.25):
        super().__init__()
        # The camera's ACTUAL field of view, which the client does not know up
        # front. Defaults to the configured estimate; tests override it to model
        # a wrong estimate, the condition that caused facing to overshoot and
        # oscillate once turns became accurate.
        self.true_hfov = true_hfov or config.GREETER_CAMERA_HFOV
        # Seconds the robot keeps rotating after StopMove.
        self.coast_seconds = coast_seconds
        self._coast_until = 0.0
        self._connected = True
        self._image_width = FRAME_W
        self.twists = []
        self.waves = 0
        self.sports = []
        # Fraction of the commanded angular rate actually achieved. Real
        # hardware delivers well under 1.0, which is why turns are closed-loop
        # against /odom yaw rather than timed.
        self.rate_factor = rate_factor
        # Simulated world: where the person actually is, in degrees off the
        # robot's heading (positive = to the robot's right).
        self.person_bearing = None
        self.person_score = 0.95
        # Commanded yaw the robot is currently executing (rad/s). Only StopMove
        # clears it — exactly like the real driver.
        self._commanded_yaw = 0.0
        self._detection_period = 1.0 / detection_hz if detection_hz else 0.05
        self._capture_lag = capture_lag
        # ROS clock deliberately offset from host time: capture stamps must
        # never be compared against time.time().
        self._ros_offset = ros_clock_offset
        self._history = []  # (capture_ts, bearing) newest last
        self._sim_stop = threading.Event()
        self._motion = threading.Thread(target=self._integrate_motion, daemon=True)
        self._motion.start()
        self._sim = threading.Thread(target=self._publish_detections, daemon=True)
        self._sim.start()

    # -- stubs --
    def _ensure_connected(self):
        pass

    def _publish_twist(self, vx, vy, vyaw):
        self.twists.append((vx, vy, vyaw))
        # The driver drops all-zero velocities, so a zero Twist must NOT stop
        # the simulated robot. Only StopMove does.
        if vx == 0.0 and vy == 0.0 and vyaw == 0.0:
            return
        self._commanded_yaw = vyaw

    def _publish_sport(self, api_id, parameter=None):
        self.sports.append(api_id)
        if api_id == STOP_MOVE:
            # Real robots don't stop instantly — they coast while decelerating,
            # which is why turn_degrees issues the stop slightly early.
            #
            # Only the FIRST StopMove starts the ramp. A robot already
            # decelerating does not begin decelerating afresh because it was told
            # to stop again, and _halt_and_settle deliberately re-issues StopMove
            # (once can be dropped or queued on a slow link). Restarting the
            # clock on every re-issue modelled a robot that can never finish
            # stopping while anyone is still asking it to: with a 0.4 s re-issue
            # against a 0.6 s coast the ramp was extended indefinitely, so the
            # settle wait ran to its full timeout and the fake rotated ~38 deg
            # further than any real dog would.
            if not self._coast_until:
                self._coast_until = time.time() + self.coast_seconds

    def perform_special_motion(self, motion_name):
        if motion_name == "hello":
            self.waves += 1
        return {"status": "success", "action": "special_motion", "motion": motion_name}

    def _grab_live_frame(self, timeout=8.0):
        return None

    def _ensure_detection_subscribed(self):
        return True

    def _ensure_yaw_subscribed(self):
        return True

    # -- simulated physics: the robot keeps turning while commanded --
    def _integrate_motion(self):
        dt = 0.02
        heading = 0.0
        while not self._sim_stop.is_set():
            yaw = self._commanded_yaw
            if yaw and self._coast_until:
                # Decelerating: keep moving (at half rate) until coast expires.
                if time.time() < self._coast_until:
                    yaw *= 0.5
                else:
                    self._commanded_yaw = 0.0
                    self._coast_until = 0.0
                    yaw = 0.0
            if yaw:
                # Achieved rate is only a fraction of the command.
                step = math.degrees(yaw) * self.rate_factor * dt
                heading += step
                # Turning right (negative yaw) reduces a positive bearing.
                if self.person_bearing is not None:
                    self.person_bearing += step
            # Feed measured yaw back exactly as the /odom subscription would.
            self._yaw = math.radians(heading)
            self._history.append((time.time() + self._ros_offset,
                                  self.person_bearing))
            if len(self._history) > 500:
                del self._history[:250]
            time.sleep(dt)

    def _bearing_at(self, capture_ts):
        """Bearing as it was `capture_lag` ago (what the camera actually saw)."""
        for ts, bearing in reversed(self._history):
            if ts <= capture_ts:
                return bearing
        return self._history[0][1] if self._history else None

    # -- simulated detector: publishes a DELAYED pixel position --
    def _publish_detections(self):
        while not self._sim_stop.is_set():
            capture_ts = time.time() + self._ros_offset - self._capture_lag
            bearing = self._bearing_at(capture_ts)
            if bearing is None:
                dets = []
            else:
                px = FRAME_W * (0.5 + bearing / self.true_hfov)
                if 0 <= px <= FRAME_W:
                    dets = [{"class_id": "person", "score": self.person_score,
                             "center_x": px, "center_y": 360,
                             "size_x": 200.0, "size_y": 400.0}]
                else:
                    dets = []  # rotated out of view
            with self._detections_lock:
                self._detections = dets
                self._detections_ts = time.time()
                self._detections_capture_ts = capture_ts
            time.sleep(self._detection_period)

    def stop_sim(self):
        self._sim_stop.set()
        self._commanded_yaw = 0.0

    # -- helpers --
    @property
    def turns(self):
        return [t for t in self.twists if t[2] != 0]

    @property
    def is_moving(self):
        return self._commanded_yaw != 0.0

    def offset_now(self):
        """Current normalised offset of the person from frame centre."""
        if self.person_bearing is None:
            return None
        return self.person_bearing / self.true_hfov


def bearing_to_px(deg):
    return FRAME_W * (0.5 + deg / config.GREETER_CAMERA_HFOV)


def wait_for_detection(client, timeout=2.0):
    """Block until the simulated detector has published at least once."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        if client._detections_ts:
            return
        time.sleep(0.02)


def wait_for_person(client, timeout=2.0):
    """Block until a person is actually visible in the detection cache."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        if client._best_person(client._fresh_detections()) is not None:
            return
        time.sleep(0.02)
    raise AssertionError("simulated detector never reported a person")


# ---------------------------------------------------------------------------
# Detection filtering
# ---------------------------------------------------------------------------

def test_stale_detections_ignored():
    c = FakeGo2()
    try:
        c.stop_sim()  # freeze the simulated detector so the cache can go stale
        c._detections = [{"class_id": "person", "score": 0.95, "center_x": 640}]
        c._detections_ts = time.time() - 99
        assert c._fresh_detections() == [], \
            "stale cache must be treated as nothing visible"
        c._detections_ts = time.time()
        assert len(c._fresh_detections()) == 1
    finally:
        c.stop_sim()


def test_low_confidence_rejected():
    c = FakeGo2()
    try:
        c.stop_sim()
        c._detections = [{"class_id": "person", "score": 0.3, "center_x": 640}]
        c._detections_ts = time.time()
        assert c._best_person(c._fresh_detections()) is None
    finally:
        c.stop_sim()


def test_picks_highest_confidence_person():
    c = FakeGo2()
    try:
        c.stop_sim()
        c._detections = [
            {"class_id": "person", "score": 0.7, "center_x": 200},
            {"class_id": "person", "score": 0.99, "center_x": 900},
        ]
        c._detections_ts = time.time()
        assert c._best_person(c._fresh_detections())["center_x"] == 900
    finally:
        c.stop_sim()


# ---------------------------------------------------------------------------
# Facing behaviour — the part that failed on hardware
# ---------------------------------------------------------------------------

def test_no_visitor_reports_error_and_never_waves():
    c = FakeGo2()
    try:
        c.person_bearing = None
        wait_for_detection(c)
        result = c.greet_visitor(0.0)
        assert result["status"] == "error", result
        assert c.waves == 0, "must not fake a greeting when nobody is there"
    finally:
        c.stop_sim()


def test_visitor_on_the_right_gets_faced():
    """A visitor 40 deg to the right must actually end up centred."""
    c = FakeGo2()
    try:
        c.person_bearing = 40.0
        wait_for_detection(c)
        result = c.greet_visitor(0.0)
        assert result["status"] == "success", result
        assert c.turns, "must issue a rotation for an off-centre visitor"
        assert all(t[2] < 0 for t in c.turns), \
            f"person on the right → negative (clockwise) yaw, got {set(t[2] for t in c.turns)}"
        assert abs(c.offset_now()) <= config.GREETER_FACE_DEADZONE + 0.02, \
            f"should end up facing them, final offset {c.offset_now():.3f}"
        assert result["faced_visitor"] is True, result
        assert c.waves == 1
        assert c.twists[-1] == (0.0, 0.0, 0.0), "must zero velocity when done"
    finally:
        c.stop_sim()


def test_visitor_on_the_left_gets_faced():
    c = FakeGo2()
    try:
        c.person_bearing = -40.0
        wait_for_detection(c)
        result = c.greet_visitor(0.0)
        assert c.turns and all(t[2] > 0 for t in c.turns), \
            f"person on the left → positive (ccw) yaw, got {set(t[2] for t in c.turns)}"
        assert abs(c.offset_now()) <= config.GREETER_FACE_DEADZONE + 0.02, \
            f"final offset {c.offset_now():.3f}"
        assert result["faced_visitor"] is True, result
    finally:
        c.stop_sim()


def test_turn_speed_is_perceptible():
    """Regression: the old P-control emitted ~0.14 rad/s, too slow to read as a turn."""
    c = FakeGo2()
    try:
        c.person_bearing = 25.0
        wait_for_detection(c)
        c.greet_visitor(0.0)
        speeds = [abs(t[2]) for t in c.turns]
        assert speeds, "expected some rotation"
        assert min(speeds) >= 0.3, \
            f"every turn command should be clearly visible, got min {min(speeds):.2f} rad/s"
        assert max(speeds) <= config.MAX_ANGULAR_VELOCITY, "must respect the safety clamp"
    finally:
        c.stop_sim()


def test_slightly_off_centre_visitor_still_turns():
    """Regression: tracking's 0.12 deadzone (24% of frame) meant no turn at all.

    A visitor ~12 deg off is only 0.1 normalised offset — inside the old
    deadzone, outside the new one — so this is exactly the case that produced
    "it greets but doesn't turn".
    """
    c = FakeGo2()
    try:
        c.person_bearing = 12.0
        offset = c.person_bearing / config.GREETER_CAMERA_HFOV
        assert offset < config.TRACKING_DEADZONE, \
            "this test is only meaningful inside the old tracking deadzone"
        wait_for_detection(c)
        c.greet_visitor(0.0)
        assert c.turns, \
            f"visitor at offset {offset:.3f} must still trigger a turn"
    finally:
        c.stop_sim()


def test_already_centred_visitor_needs_no_rotation():
    c = FakeGo2()
    try:
        c.person_bearing = 0.0
        wait_for_detection(c)
        result = c.greet_visitor(0.0)
        assert result["status"] == "success"
        assert not c.turns, f"dead-centre visitor needs no turn, got {c.turns}"
        assert result["turn_detail"]["corrections"] == 0
    finally:
        c.stop_sim()


def test_bounded_corrections_when_visitor_keeps_moving():
    """A visitor who dodges must not spin the robot indefinitely."""
    c = FakeGo2()
    try:
        c.person_bearing = 45.0
        wait_for_detection(c)

        def dodge():
            # Keep shoving them back off-centre so centring never succeeds.
            end = time.time() + 6
            while time.time() < end:
                if c.person_bearing is not None and abs(c.person_bearing) < 30:
                    c.person_bearing = 45.0
                time.sleep(0.1)
        threading.Thread(target=dodge, daemon=True).start()

        start = time.time()
        result = c.greet_visitor(0.0)
        elapsed = time.time() - start
        assert result["turn_detail"]["corrections"] <= config.GREETER_MAX_CORRECTIONS, \
            result["turn_detail"]
        assert elapsed < config.GREETER_FACE_TIMEOUT + 4, f"took {elapsed:.1f}s"
        assert c.waves == 1, "should still greet even if it couldn't centre"
        assert c.twists[-1] == (0.0, 0.0, 0.0)
    finally:
        c.stop_sim()


def test_no_turn_commands_translate_the_robot():
    """Greeter must never walk — it only rotates (keeps it lidar-independent)."""
    c = FakeGo2()
    try:
        c.person_bearing = 35.0
        wait_for_detection(c)
        c.greet_visitor(0.0)
        bad = [t for t in c.twists if t[0] != 0.0 or t[1] != 0.0]
        assert not bad, f"greeter must not command linear motion, got {bad}"
    finally:
        c.stop_sim()


# ---------------------------------------------------------------------------
# Overshoot / oscillation (the "turns too fast and passes the object" bug)
# ---------------------------------------------------------------------------

def test_does_not_overshoot_when_hfov_is_overestimated():
    """Regression: an overestimated FOV made every turn too big.

    The client assumes GREETER_CAMERA_HFOV; here the camera is really much
    narrower. Before the fix this sailed past the target, corrected back, and
    hunted. Now the sub-1.0 gain (plus runtime calibration) must keep it from
    ever crossing to the far side.
    """
    c = FakeGo2(true_hfov=80.0)   # client assumes ~110
    try:
        c.person_bearing = 30.0   # target to the right
        wait_for_person(c)
        crossings = []
        original = c._turn_for

        def spy(vyaw, duration):
            original(vyaw, duration)
            crossings.append(c.person_bearing)
        c._turn_for = spy

        c.greet_visitor(0.0)
        # It must never end up substantially on the far side of centre.
        worst = min(crossings, default=0.0)
        assert worst > -12.0, \
            f"overshot to the far side: bearings after each turn {crossings}"
    finally:
        c.stop_sim()


def test_no_direction_reversals_while_facing():
    """Hunting shows up as turn commands flipping sign; there should be none."""
    c = FakeGo2(true_hfov=85.0)
    try:
        c.person_bearing = 35.0
        wait_for_person(c)
        c.greet_visitor(0.0)
        signs = [(1 if t[2] > 0 else -1) for t in c.turns]
        reversals = sum(1 for a, b in zip(signs, signs[1:]) if a != b)
        assert reversals == 0, \
            f"turn direction reversed {reversals} time(s) — robot is hunting"
    finally:
        c.stop_sim()


def test_small_corrections_use_the_slow_speed():
    """Fast turns coast past a near target; small corrections must be gentle.

    Asserts against an absolute ceiling, not config.GREETER_FINE_TURN_SPEED —
    comparing to the same constant the code uses would make this test pass no
    matter how fast the fine speed was set.
    """
    c = FakeGo2(true_hfov=100.0)
    try:
        c.person_bearing = 12.0   # small offset → fine speed
        wait_for_person(c)
        c.greet_visitor(0.0)
        speeds = [abs(t[2]) for t in c.turns]
        assert speeds, "expected some rotation"
        assert max(speeds) <= 0.4, \
            f"small corrections must be slow enough not to coast past the " \
            f"target, saw {max(speeds):.2f} rad/s"
    finally:
        c.stop_sim()


def test_learns_the_true_hfov_from_turning():
    """Calibration should pull the estimate toward reality after a turn."""
    c = FakeGo2(true_hfov=80.0)
    try:
        assert abs(c._hfov - config.GREETER_CAMERA_HFOV) < 1e-6
        c.person_bearing = 35.0
        wait_for_person(c)
        c.greet_visitor(0.0)
        assert c._hfov_calibrated, "should have taken a calibration sample"
        assert abs(c._hfov - 80.0) < abs(config.GREETER_CAMERA_HFOV - 80.0), \
            f"estimate {c._hfov:.0f} should be closer to the true 80 than the " \
            f"initial {config.GREETER_CAMERA_HFOV:.0f}"
    finally:
        c.stop_sim()


def test_turn_lands_near_target_despite_coast():
    """The stop is issued early so deceleration doesn't carry it past."""
    c = FakeGo2(coast_seconds=0.4)
    try:
        result = c.turn_degrees(40.0, speed=0.6)
        measured = result["measured_degrees"]
        assert abs(measured - 40.0) <= 10.0, \
            f"asked 40deg, ended at {measured}deg (coast not compensated)"
    finally:
        c.stop_sim()


def test_turns_past_half_a_revolution_actually_complete():
    """Regression: anything >= ~176 deg spun until the timeout.

    Rotation used to be measured as abs(_yaw_delta(start, now)), a SHORTEST arc,
    so the measurement saturated at 180 deg and then counted back down as the
    robot kept going. The target was therefore unreachable: the loop ran out its
    full time budget and reported the wrapped remainder. On hardware a 270 deg
    request measured 141.7 and a 360 deg request measured 26.5, both after
    spinning until the deadline.

    Asserts on `ended` as well as the angle, because the old code produced a
    plausible-looking measured_degrees while having completely lost track.
    """
    for requested in (270.0, 360.0):
        c = FakeGo2()
        try:
            result = c.turn_degrees(requested, speed=0.8)
            assert result["ended"] == "target", \
                f"{requested}deg turn ended by {result['ended']}, not by " \
                f"reaching the target: {result}"
            measured = result["measured_degrees"]
            assert measured > 180.0, \
                f"measured {measured}deg for {requested}deg — still saturating " \
                f"at half a revolution"
            assert abs(measured - requested) <= 25.0, \
                f"asked {requested}deg, measured {measured}deg"
        finally:
            c.stop_sim()


def test_fast_turn_does_not_coast_past_the_target():
    """A long coast at speed must not carry the robot well beyond the request.

    Absolute bound on purpose: this is the failure mode that makes the robot
    sail past an object, so it must fail if the stop lead is removed regardless
    of how the constants are configured.
    """
    c = FakeGo2(coast_seconds=0.6)   # sluggish stop
    try:
        result = c.turn_degrees(30.0, speed=0.8)
        measured = result["measured_degrees"]
        assert measured <= 30.0 + 8.0, \
            f"asked 30deg but coasted to {measured}deg — stop issued too late"
    finally:
        c.stop_sim()


# ---------------------------------------------------------------------------
# Regressions for the two earlier hardware bugs
# ---------------------------------------------------------------------------

def test_turn_ends_with_an_explicit_stopmove():
    """A zero Twist is dropped by the driver, so StopMove must be sent."""
    c = FakeGo2()
    try:
        c.person_bearing = 40.0
        wait_for_person(c)
        c.greet_visitor(0.0)
        assert STOP_MOVE in c.sports, \
            "must issue StopMove — a zero Twist alone never reaches the robot"
    finally:
        c.stop_sim()


def test_robot_does_not_coast_after_facing():
    """Regression: the robot kept rotating after the turn 'ended'."""
    c = FakeGo2()
    try:
        c.person_bearing = 40.0
        wait_for_person(c)
        c.greet_visitor(0.0)
        assert not c.is_moving, "robot must be halted, not coasting, after facing"
        settled = c.person_bearing
        time.sleep(0.4)
        assert abs(c.person_bearing - settled) < 0.5, \
            f"bearing kept drifting {settled:.1f} -> {c.person_bearing:.1f}: still moving"
    finally:
        c.stop_sim()


def test_does_not_overshoot_the_target_bearing():
    """Regression: correcting against pre-turn frames rotated ~99deg for ~40deg."""
    c = FakeGo2()
    try:
        target = 40.0
        c.person_bearing = target
        wait_for_person(c)
        result = c.greet_visitor(0.0)
        turned = result["turn_detail"]["turned_degrees"]
        # Total rotation is the real overshoot signal (the bug spent ~99 deg on a
        # 40 deg target). Correction COUNT is not: the gain deliberately
        # undershoots, so several small same-direction turns are the healthy
        # pattern — only their sum tells you whether it sailed past.
        assert turned <= target * 1.5, \
            f"turned {turned}deg for a {target}deg target — overshooting"
        assert result["turn_detail"]["corrections"] <= \
            config.GREETER_MAX_CORRECTIONS, result["turn_detail"]
        assert abs(c.person_bearing) < abs(target), \
            f"should end closer to centre: {target} -> {c.person_bearing:.1f}"
    finally:
        c.stop_sim()


def test_stop_greeter_issues_stopmove():
    c = FakeGo2()
    try:
        c.person_bearing = None
        wait_for_detection(c)
        c.start_greeter()
        c.sports.clear()
        c.stop_greeter()
        assert STOP_MOVE in c.sports, "stop_greeter must actually halt the robot"
    finally:
        c.stop_sim()


def test_wave_happens_after_motion_stops():
    """The robot ignores gestures while it believes it is walking."""
    c = FakeGo2()
    try:
        c.person_bearing = 30.0
        wait_for_person(c)
        # Record whether the robot was still moving at the moment of the wave.
        moving_at_wave = []
        original = c.perform_special_motion

        def spy(name):
            if name == "hello":
                moving_at_wave.append(c.is_moving)
            return original(name)
        c.perform_special_motion = spy

        c.greet_visitor(0.0)
        assert moving_at_wave == [False], \
            f"wave must be sent while stopped, was moving={moving_at_wave}"
    finally:
        c.stop_sim()


def test_capture_stamp_is_not_host_time():
    """Capture stamps come from the ROS clock — never compare to time.time()."""
    c = FakeGo2()
    try:
        wait_for_detection(c)
        assert abs(c._capture_ts() - time.time()) > 1000, \
            "fake must model an offset ROS clock so cross-clock bugs surface"
    finally:
        c.stop_sim()


# ---------------------------------------------------------------------------
# Watch mode
# ---------------------------------------------------------------------------

def test_watch_mode_one_wave_per_visitor_then_rearms():
    c = FakeGo2()
    try:
        c.person_bearing = None
        wait_for_detection(c)
        c.start_greeter()

        c.person_bearing = 0.0          # visitor arrives, dead centre
        time.sleep(GREET_SECONDS)
        assert c.waves == 1, f"one wave while the same visitor stays, got {c.waves}"

        c.person_bearing = None          # visitor leaves → re-arm
        time.sleep(config.GREETER_RESET_AFTER + 0.4)

        c.person_bearing = 0.0           # next visitor
        time.sleep(GREET_SECONDS)
        assert c.waves == 2, f"should greet the next visitor, got {c.waves}"

        status = c.get_greeter_status()["greeter"]
        assert status["state"] == "watching"
        assert len(status["new_greetings"]) == 2, status
        assert status["last_greeting"] is not None
        assert c.get_greeter_status()["greeter"]["new_greetings"] == [], \
            "pending queue must drain on read so greetings aren't spoken twice"

        result = c.stop_greeter()
        assert result["was_running"] and result["greeted_count"] == 2, result
        assert c.twists[-1] == (0.0, 0.0, 0.0)
    finally:
        c.stop_greeter()
        c.stop_sim()


def test_watch_mode_announces_the_greeting():
    """The wave is immediate, so the spoken welcome should be too."""
    from voice_agent import notifications
    notifications.drain()
    c = FakeGo2()
    try:
        c.person_bearing = None
        wait_for_detection(c)
        c.start_greeter()
        c.person_bearing = 0.0
        time.sleep(GREET_SECONDS)
        assert c.waves == 1, c.waves
        msgs = notifications.drain()
        assert msgs, "watch-mode greeting must queue an announcement"
        joined = " ".join(msgs).lower()
        assert "waved" in joined or "greet" in joined, msgs
    finally:
        c.stop_greeter()
        c.stop_sim()
        notifications.drain()


def test_starting_greeter_stops_tracker():
    c = FakeGo2()
    try:
        c.person_bearing = None
        wait_for_detection(c)
        c._tracking_target = "person"
        c._tracking_thread = None
        c.start_greeter()
        assert c._tracking_target == "", \
            "tracker must yield so the two don't fight over /cmd_vel_out"
    finally:
        c.stop_greeter()
        c.stop_sim()


def test_double_start_is_safe():
    c = FakeGo2()
    try:
        c.person_bearing = None
        wait_for_detection(c)
        c.start_greeter()
        again = c.start_greeter()
        assert again["status"] == "success" and "already" in again["note"].lower()
    finally:
        c.stop_greeter()
        c.stop_sim()


def test_status_and_stop_before_ever_starting():
    c = FakeGo2()
    try:
        status = c.get_greeter_status()["greeter"]
        assert status["state"] == "idle"
        assert status["greeted_count"] == 0
        assert status["last_greeting"] is None
        assert c.stop_greeter()["was_running"] is False
    finally:
        c.stop_sim()


def test_history_is_capped():
    c = FakeGo2()
    config.GREETER_HISTORY_MAX = 3
    try:
        c.person_bearing = 0.0
        wait_for_person(c)
        for _ in range(5):
            c._perform_greeting(c._best_person(c._fresh_detections()))
        assert len(c._greeter_history) == 3, len(c._greeter_history)
    finally:
        config.GREETER_HISTORY_MAX = 10
        c.stop_sim()


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
