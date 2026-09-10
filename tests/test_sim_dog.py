# Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
# SPDX-License-Identifier: MIT-0
"""Physics tests for the MuJoCo simulation transport (no robot, no AWS).

Runs the real simulation — `strands-robots` + MuJoCo — and asserts on measured
state, because every interesting failure here is silent. A mistuned pose still
returns `{"status": "success"}` while the dog lies splayed on the floor; a walk
that ghosts through the workbench reports the distance it was asked for. So the
assertions are geometric: hip heights, base height, distance covered, heading.

Slower than the other suites (~1 min: the poses have to settle under gravity)
but needs no robot, no container, no AWS and no network after the first run,
which clones MuJoCo Menagerie for the Go2's meshes. Run directly:

    python3 tests/test_sim_dog.py
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import mujoco  # noqa: E402

from voice_agent.go2_sim_client import Go2SimClient  # noqa: E402
from voice_agent.sim_dog import MOTIONS, POSES, UNSIMULATED  # noqa: E402

# One simulation for the whole suite: building the world costs ~1.5 s and the
# tests are written to leave the dog standing, so they compose.
CLIENT: Go2SimClient = None  # type: ignore[assignment]

STAND_HEIGHT = 0.256   # measured settled base height in the stance pose
TOLERANCE = 0.03


def hip_heights():
    """World z of the front-left and rear-left hips.

    The honest way to tell a sit from a collapse: base height alone cannot,
    because a dog in the splits with its chest down and a dog sitting on its
    haunches put the base at a similar height. The front-to-rear hip difference
    is what distinguishes them.
    """
    dog = CLIENT._dog
    m, d = dog._m, dog._d
    front = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "unitree_go2/FL_hip")
    rear = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "unitree_go2/RL_hip")
    return float(d.xpos[front][2]), float(d.xpos[rear][2])


def test_stands_level_under_gravity():
    res = CLIENT.stand_up()
    assert res["status"] == "success", res
    front, rear = hip_heights()
    assert abs(res["body_height"] - STAND_HEIGHT) < TOLERANCE, res["body_height"]
    assert abs(front - rear) < 0.03, (front, rear)   # level, not pitched


def test_pose_targets_are_within_joint_limits():
    """A target outside `jnt_range` is unreachable, so the joint buzzes forever."""
    dog = CLIENT._dog
    for name, pose in POSES.items():
        for i, target in enumerate(pose):
            lo, hi = dog._lo[i], dog._hi[i]
            assert lo <= target <= hi, f"{name}[{i}]={target} outside [{lo}, {hi}]"


def test_sit_pitches_back_onto_haunches():
    assert CLIENT.sit()["status"] == "success"
    front, rear = hip_heights()
    assert front - rear > 0.15, f"not sitting: front={front:.3f} rear={rear:.3f}"
    assert rear < 0.20, f"haunches not down: rear={rear:.3f}"
    CLIENT.stand_up()


def test_stand_down_puts_the_belly_on_the_floor():
    res = CLIENT.stand_down()
    assert res["status"] == "success", res
    assert res["body_height"] < 0.15, res
    assert res["note"] == "crouched", res
    CLIENT.stand_up()


def test_walk_covers_the_commanded_distance():
    CLIENT.stand_up()
    res = CLIENT.move(vx=0.35, duration=3.0)
    assert res["status"] == "success", res
    assert not res["blocked"], res
    # 0.35 m/s for 3 s. Generous lower bound: the dog starts from rest.
    assert 0.75 < res["travelled_metres"] < 1.15, res
    assert abs(res["turned_degrees"]) < 8, res     # walks straight


def test_turn_tracks_the_commanded_yaw_rate():
    CLIENT.stand_up()
    res = CLIENT.move(vyaw=0.6, duration=2.0)
    assert res["status"] == "success", res
    # 0.6 rad/s for 2 s = 68.8 deg.
    assert 55 < res["turned_degrees"] < 80, res
    assert res["travelled_metres"] < 0.1, res      # turns in place
    CLIENT.move(vyaw=-0.6, duration=2.0)           # face forward again


def test_walking_into_the_furniture_reports_blocked():
    """The base pose is imposed, so nothing but this check stops it ghosting."""
    CLIENT.stand_up()
    res = CLIENT.move(vx=0.5, duration=9.0)
    assert res["status"] == "success", res
    assert res["blocked"], res
    assert res.get("note"), "a blocked walk must carry a note the agent can relay"
    # It must have stopped somewhere short of the far wall (x = 4.2).
    assert CLIENT.get_state()["position"][0] < 3.7, CLIENT.get_state()


def test_every_gesture_completes_and_leaves_a_known_posture():
    for name in sorted(MOTIONS):
        CLIENT.stand_up()
        res = CLIENT.perform_special_motion(name)
        assert res["status"] == "success", (name, res)
        assert res["completed"], f"{name} did not finish"
        posture = CLIENT.get_state()["posture"]
        assert posture in ("standing", "sitting", "lying"), (name, posture)
    CLIENT.stand_up()


def test_unsimulated_tricks_are_refused_not_faked():
    for name in UNSIMULATED:
        res = CLIENT.perform_special_motion(name)
        assert res["status"] == "error", (name, res)
        assert res["note"], name


def test_unknown_motion_lists_what_is_available():
    res = CLIENT.perform_special_motion("moonwalk")
    assert res["status"] == "error", res
    assert "hello" in res["message"], res


def test_detector_tools_report_their_absence():
    """They must error, not no-op: a silent success teaches the agent to lie."""
    for call in (CLIENT.get_detections, CLIENT.get_find_status,
                 CLIENT.get_greeter_status, CLIENT.start_greeter,
                 lambda: CLIENT.start_tracking("person"),
                 lambda: CLIENT.find_object("backpack"),
                 lambda: CLIENT.greet_visitor(0.0)):
        res = call()
        assert res["status"] == "error", res
        assert res["note"], res


def test_head_camera_renders_a_varied_frame():
    """Guards the two failure modes a flat frame hides: no world, or lens inside a geom."""
    frame = CLIENT._grab_live_frame()
    assert frame.size == (640, 480), frame.size
    colours = frame.convert("RGB").getcolors(maxcolors=200_000)
    assert colours is not None and len(colours) > 50, \
        f"frame has {0 if colours is None else len(colours)} colours — camera may be inside a wall"


def test_state_reports_simulation_health():
    state = CLIENT.get_state()
    assert state["simulated"] is True
    assert state["sim"]["realtime_factor"] > 0.3, state["sim"]
    assert CLIENT._dog._fault is None, CLIENT._dog._fault


def main() -> int:
    global CLIENT
    print("building the simulation (first run clones MuJoCo Menagerie)...")
    CLIENT = Go2SimClient(scene="lab", viewer=False)
    CLIENT.connect()

    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    failures = 0
    try:
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
    finally:
        CLIENT.disconnect()
    print(f"\n{len(tests) - failures}/{len(tests)} passed")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
