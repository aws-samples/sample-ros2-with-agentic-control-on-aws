# Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
# SPDX-License-Identifier: MIT-0
"""MuJoCo Go2 simulator — the motion engine behind the sim transport.

Wraps `strands_robots.Robot("unitree_go2")` (Strands Labs' robots lab,
https://strandsagents.com/docs/labs/robots/) into something that answers the
same high-level verbs the real dog does: stand, sit, lie down, walk, wave.
`Go2SimClient` in `go2_sim_client.py` is the thin adapter that turns those into
the `Go2ClientProtocol` the voice agent's tools call.

Why this file exists at all — what the lab gives us and what it does not:

  * It gives us the Go2's real MJCF model (MuJoCo Menagerie, auto-downloaded by
    `robot_descriptions`), a physics world, cameras, offscreen rendering and an
    interactive viewer. That is the expensive part and we use all of it.

  * It does NOT give us a Go2 locomotion controller. The Menagerie model's 12
    actuators are `<motor>` — pure TORQUE, ctrlrange +/-23.7 Nm (+/-45.43 at the
    knees) — so `send_action({"FL_thigh": 0.9})` applies 0.9 Nm, not a 0.9 rad
    target. Left alone the dog simply collapses. The shipped `mock` policy is a
    sinusoid, and `go2_walk_forward` is an RL *benchmark spec* (a reward +
    success predicate), i.e. a thing to train a gait against, not a gait.

So the joint-level control loop is ours:

  Poses (stand / sit / lie / gestures) are PHYSICS-HONEST. A PD law
  (`tau = kp*(q* - q) - kd*qdot`, clipped to the model's own torque limits) runs
  at the 500 Hz physics rate in a background thread. The dog holds its own
  weight against gravity, its feet make real contacts, and it settles at
  base z ~= 0.25 m in stance. Nothing is teleported.

  Walking is ANIMATED, not earned. An open-loop trot (diagonal pairs, stride and
  foot lift scaled by commanded speed) plays on the legs under the same PD law,
  but the BASE is driven kinematically: its pose is integrated from the
  commanded twist and written into `qpos` every step, held level and at walking
  height. Feet slide rather than propel, and the dog cannot fall over while
  walking.

  The first cut injected only base *velocity* and left height and attitude to
  physics. It looks better on paper and does not survive contact: planted feet
  resist the imposed velocity, the friction couple builds a yaw moment, and the
  dog spins out and lands on its back within two seconds of "walk forward". A
  demo that never faceplants mid-sentence beats a physically-earned gait we do
  not have. Drop a trained policy in here (see `run_policy` / `create_policy` in
  the lab docs) to replace `_trot_overlay` and `_drive_base`.

Threading: one background thread owns every MuJoCo touch (PD write, twist
injection, `mj_step`, viewer sync), serialized on the engine's own `_lock` —
the same lock `get_frame` takes, so frame grabs from the agent thread are safe.
`mjData` is not thread-safe and stepping it from two threads corrupts the
constraint solver ("nefc under-allocation"), which is why commands from the
agent are queued as plain setpoints and applied by that one thread.
"""

from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass, field
from typing import Optional

import numpy as np

from . import config

# Leg order used everywhere in this file: front-left, front-right, rear-left,
# rear-right. Matches the model's actuator order, so a 12-vector can be written
# to `data.ctrl` directly.
LEGS = ("FL", "FR", "RL", "RR")
JOINTS = ("hip", "thigh", "calf")

# Room-fixed camera, for screenshots and pose checks. Not the camera the agent's
# vision tools look through — that one is config.SIM_CAMERA_NAME.
#
# Fixed to the room rather than mounted on the dog on purpose: a camera parented
# to the base pitches and rolls with it, so a 35-degree sit and a level stand
# render nearly identically (the horizon moves instead of the dog) and the view
# is useless for judging a pose. The cost is that the dog can walk out of frame.
ROOM_CAMERA = "room"

# Menagerie's `home` keyframe stance, per leg: (hip, thigh, calf) in radians.
# The dog settles at base z ~= 0.25 m here — the Go2's nominal ~0.32 m standing
# height is measured with the firmware's own balance controller, which holds a
# taller stance than this static pose.
STAND = (0.0, 0.9, -1.8)


def _pose(fl, fr, rl, rr) -> np.ndarray:
    """Build a 12-vector target from four (hip, thigh, calf) triples."""
    return np.array(fl + fr + rl + rr, dtype=float)


def _uniform(triple) -> np.ndarray:
    """Same (hip, thigh, calf) on all four legs."""
    return _pose(triple, triple, triple, triple)


# --- Named poses ------------------------------------------------------------
# All four are within the model's joint limits and settle under PD without
# fighting a limit (a target outside `jnt_range` is clamped in `_set_target`,
# but a target *at* a limit chatters against it).
POSES: dict[str, np.ndarray] = {
    # Balanced stance — the pose every motion returns to.
    "stand": _uniform(STAND),
    # StandDown / crouch: knees folded, belly ~5 cm off the floor. Safe to
    # "power off" from, like the real 1005 StandDown.
    "lie": _uniform((0.0, 1.32, -2.65)),
    # Sit: rear legs folded right under, front legs propping the chest up, so
    # physics pitches the body back ~35 deg onto its haunches (front hip settles
    # at z ~= 0.36, rear at ~0.13).
    #
    # The front calf angle is what makes or breaks this. A foot only stays put if
    # it sits roughly under its own hip, which for this leg geometry means
    # calf ~= -2 * thigh; extending the front legs forward instead (a small or
    # negative thigh with a deep knee) puts the contact ahead of the shoulder,
    # the feet slide out, and the dog ends up in the splits with its chest on the
    # floor — a pose that reads as "collapsed", not "sitting".
    "sit": _pose(
        (0.0, 0.90, -0.85),    # FL — front, propped, foot under the shoulder
        (0.0, 0.90, -0.85),    # FR
        (0.0, 1.55, -2.70),    # RL — rear, tucked right under
        (0.0, 1.55, -2.70),    # RR
    ),
    # Stretch: front end down and reaching, rear end up.
    "stretch": _pose(
        (0.0, 1.10, -2.30),
        (0.0, 1.10, -2.30),
        (0.0, 0.30, -1.10),
        (0.0, 0.30, -1.10),
    ),
}

# Front-left paw raised off the sit pose, at two hip angles — alternating
# between them is the wave. The other three legs keep their sit targets, which is
# what holds the dog up while it waves.
_SIT_FRONT = (0.0, 0.90, -0.85)
_SIT_REAR = (0.0, 1.55, -2.70)
_WAVE_A = _pose((0.35, -0.45, -1.30), _SIT_FRONT, _SIT_REAR, _SIT_REAR)
_WAVE_B = _pose((-0.15, -0.75, -1.05), _SIT_FRONT, _SIT_REAR, _SIT_REAR)

# Both front paws up from the sit — the "heart" gesture's static analogue.
_HEART = _pose((0.30, -0.60, -1.15), (-0.30, -0.60, -1.15), _SIT_REAR, _SIT_REAR)

# Front-left paw sweeping backwards along the floor (Scrape).
_SCRAPE_A = _pose((0.0, -0.55, -0.75), (0.0, 0.90, -1.80), (0.0, 0.90, -1.80), (0.0, 0.90, -1.80))
_SCRAPE_B = _pose((0.0, 0.55, -1.55), (0.0, 0.90, -1.80), (0.0, 0.90, -1.80), (0.0, 0.90, -1.80))

# Hips swaying side to side in a stand (WiggleHips / "content").
_SWAY_L = _uniform((0.28, 0.90, -1.80))
_SWAY_R = _uniform((-0.28, 0.90, -1.80))

# Body bouncing low/high, and a diagonal "shimmy", for the dance routines.
_LOW = _uniform((0.0, 1.05, -2.10))
_HIGH = _uniform((0.0, 0.62, -1.45))
_SHIMMY = _pose((0.22, 0.75, -1.62), (-0.22, 1.02, -2.00), (-0.22, 0.75, -1.62), (0.22, 1.02, -2.00))


@dataclass
class Motion:
    """A gesture: waypoints the PD target is interpolated through, in order.

    Each waypoint is `(seconds_to_reach_it, 12-vector)`. Linear interpolation
    between waypoints keeps the target continuous, which matters — stepping the
    setpoint discontinuously spikes the PD torque and kicks the dog off balance.
    """

    name: str
    waypoints: list[tuple[float, np.ndarray]]
    # Pose the dog is left in, for state reporting and for deciding whether it
    # must stand before walking.
    ends_as: str = "standing"
    # Right the body first if it has ended up on its side — what the real
    # robot's RecoveryStand does before it stands. See `_right_body`.
    rights: bool = False

    @property
    def duration(self) -> float:
        return sum(w[0] for w in self.waypoints)


def _wave(times: int = 3) -> list[tuple[float, np.ndarray]]:
    out = [(0.8, POSES["sit"]), (0.5, _WAVE_A)]
    for _ in range(times):
        out += [(0.35, _WAVE_B), (0.35, _WAVE_A)]
    return out + [(0.5, POSES["sit"])]


MOTIONS: dict[str, Motion] = {
    "stand_up": Motion("stand_up", [(1.2, POSES["stand"])], "standing", rights=True),
    "balance_stand": Motion("balance_stand", [(0.8, POSES["stand"])], "standing", rights=True),
    "stand_down": Motion("stand_down", [(1.5, POSES["lie"])], "lying"),
    "sit": Motion("sit", [(1.4, POSES["sit"])], "sitting"),
    "rise_sit": Motion("rise_sit", [(1.2, POSES["stand"])], "standing", rights=True),
    "hello": Motion("hello", _wave(3), "sitting"),
    "heart": Motion(
        "heart",
        [(0.8, POSES["sit"]), (0.6, _HEART), (1.4, _HEART), (0.6, POSES["sit"])],
        "sitting",
    ),
    "stretch": Motion(
        "stretch",
        [(1.2, POSES["stretch"]), (1.2, POSES["stretch"]), (1.2, POSES["stand"])],
        "standing",
    ),
    "content": Motion(
        "content",
        [(0.5, _SWAY_L), (0.5, _SWAY_R), (0.5, _SWAY_L), (0.5, _SWAY_R), (0.5, POSES["stand"])],
        "standing",
    ),
    "scrape": Motion(
        "scrape",
        [(0.6, _SCRAPE_A), (0.4, _SCRAPE_B), (0.4, _SCRAPE_A), (0.4, _SCRAPE_B), (0.6, POSES["stand"])],
        "standing",
    ),
    "dance1": Motion(
        "dance1",
        [(0.45, _LOW), (0.45, _HIGH), (0.45, _LOW), (0.45, _HIGH),
         (0.5, _SWAY_L), (0.5, _SWAY_R), (0.6, POSES["stand"])],
        "standing",
    ),
    "dance2": Motion(
        "dance2",
        [(0.5, _SHIMMY), (0.5, _SWAY_R), (0.5, _SHIMMY), (0.5, _SWAY_L),
         (0.4, _LOW), (0.4, _HIGH), (0.6, POSES["stand"])],
        "standing",
    ),
    # Crouch-and-extend. Under PD this really does leave the ground, which is
    # also why it can land badly — `stand_up` recovers.
    "front_jump": Motion(
        "front_jump",
        [(0.5, _LOW), (0.12, _HIGH), (0.5, POSES["stand"]), (0.5, POSES["stand"])],
        "standing",
    ),
}

# Real-robot motions with no honest sim counterpart. The flips are ballistic
# whole-body manoeuvres the firmware performs with torque control we do not
# model; faking them by teleporting the base would put the dog through the floor
# and teach the agent that "do a backflip" always succeeds.
UNSIMULATED = {
    "front_flip": "a front flip",
    "back_flip": "a back flip",
    "left_flip": "a left flip",
    "front_pounce": "a pounce",
}


# --- Scenes -----------------------------------------------------------------
# Something for the camera (and so describe_scene) to talk about: a walled room
# with a workbench down one side and a clear lane straight ahead. `add_object`
# takes FULL extents in metres, not MuJoCo half-extents.
#
# The dog spawns at the origin facing +x. The furniture is deliberately NOT in
# that lane: with a workbench parked 1.9 m dead ahead, "walk forward for three
# seconds" ends with the dog's nose against a 0.74 m wall of wood, and every
# frame describe_scene grabs after that is a flat brown rectangle. Keeping the
# lane open to the far wall means the default demo commands do something worth
# looking at.
#
# Static objects (is_static=True) are welded to the world AND become the
# footprints `_free` refuses to walk into. Movable props (ball, cube) are left
# free so the dog can shove them around.
LAB_SCENE = [
    # Room: 6.2 m x 5.3 m, walls 2.5 m tall.
    dict(name="wall_front", shape="box", position=[4.25, 0.05, 1.25],
         size=[0.1, 5.5, 2.5], color=[0.88, 0.87, 0.84, 1.0], is_static=True),
    dict(name="wall_back", shape="box", position=[-2.05, 0.05, 1.25],
         size=[0.1, 5.5, 2.5], color=[0.88, 0.87, 0.84, 1.0], is_static=True),
    dict(name="wall_left", shape="box", position=[1.1, 2.75, 1.25],
         size=[6.4, 0.1, 2.5], color=[0.86, 0.85, 0.82, 1.0], is_static=True),
    dict(name="wall_right", shape="box", position=[1.1, -2.65, 1.25],
         size=[6.4, 0.1, 2.5], color=[0.86, 0.85, 0.82, 1.0], is_static=True),
    # Workbench down the left-hand side, with the usual desk clutter.
    dict(name="workbench", shape="box", position=[2.3, 1.7, 0.37],
         size=[1.8, 0.7, 0.74], color=[0.55, 0.42, 0.30, 1.0], is_static=True),
    dict(name="monitor", shape="box", position=[2.5, 1.9, 0.94],
         size=[0.5, 0.06, 0.36], color=[0.10, 0.10, 0.12, 1.0], is_static=True),
    dict(name="laptop", shape="box", position=[1.9, 1.6, 0.755],
         size=[0.34, 0.24, 0.03], color=[0.30, 0.31, 0.34, 1.0], is_static=True),
    dict(name="red_cube", shape="box", position=[2.15, 1.5, 0.775],
         size=[0.07, 0.07, 0.07], color=[0.85, 0.12, 0.12, 1.0], mass=0.2),
    dict(name="yellow_bottle", shape="cylinder", position=[2.95, 1.6, 0.85],
         size=[0.07, 0.07, 0.22], color=[0.95, 0.80, 0.15, 1.0], mass=0.3),
    dict(name="chair_near", shape="box", position=[1.45, 1.0, 0.23],
         size=[0.45, 0.45, 0.46], color=[0.15, 0.15, 0.17, 1.0], is_static=True),
    dict(name="chair_far", shape="box", position=[2.75, 1.0, 0.23],
         size=[0.45, 0.45, 0.46], color=[0.15, 0.15, 0.17, 1.0], is_static=True),
    # On the floor to the dog's right, where it can nudge it.
    dict(name="blue_ball", shape="sphere", position=[1.7, -0.55, 0.06],
         size=[0.12], color=[0.15, 0.35, 0.85, 1.0], mass=0.2),
]

SCENES = {"lab": LAB_SCENE, "empty": []}


@dataclass
class _Command:
    """Setpoints the agent thread writes and the sim thread reads."""

    motion: Optional[Motion] = None       # gesture to start on the next tick
    twist: tuple = (0.0, 0.0, 0.0)        # vx, vy, vyaw — body frame
    twist_until: float = 0.0              # wall-clock deadline for the twist
    lock: threading.Lock = field(default_factory=threading.Lock)


class SimDog:
    """A Unitree Go2 in MuJoCo, driven by high-level verbs.

    Lifecycle: `start()` builds the world and spins up the physics thread,
    `close()` tears it down. Everything in between is thread-safe.
    """

    def __init__(
        self,
        scene: str = "lab",
        viewer: bool = False,
        robot_name: str = "unitree_go2",
        camera_size: tuple = (640, 480),
    ):
        self._scene_name = scene
        self._want_viewer = viewer
        self._robot_name = robot_name
        self._cam_w, self._cam_h = camera_size

        self._sim = None            # strands_robots MuJoCoSimEngine
        self._mj = None             # the mujoco module
        self._m = None              # MjModel
        self._d = None              # MjData
        self._lock = None           # the engine's RLock — see module docstring

        self._qadr = np.zeros(12, dtype=int)   # qpos index per leg joint
        self._vadr = np.zeros(12, dtype=int)   # qvel index per leg joint
        self._base_qadr = 0                    # free-joint qpos index
        self._base_vadr = 0                    # free-joint qvel index
        self._lo = np.zeros(12)                # joint limits, from the model
        self._hi = np.zeros(12)
        self._tau_lim = np.full(12, 23.7)      # torque limits, from the model

        self._target = POSES["stand"].copy()   # live PD setpoint
        self._pose_label = "standing"
        self._cmd = _Command()

        # Active gesture: waypoint list, index, and elapsed time in this leg of
        # the interpolation. Owned by the sim thread.
        self._motion: Optional[Motion] = None
        self._m_idx = 0
        self._m_t = 0.0
        self._m_from = self._target.copy()
        self._motion_done = threading.Event()
        self._motion_done.set()

        self._gait_phase = 0.0
        self._moving = False
        self._blocked = False
        self._progress_ref = None    # (x, y, wall_time, distance_commanded)
        self._obstacles: list = []   # (x0, x1, y0, y1) footprints, body-inflated
        self._viewer_open = False

        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()
        self._started = False
        self._realtime_factor = 1.0
        self._fault: Optional[str] = None   # why the physics loop stopped, if it did

    # --- lifecycle ---------------------------------------------------------

    def start(self) -> None:
        """Build the world, place the camera, open the viewer, start stepping.

        The viewer must be up BEFORE the physics thread starts:
        `launch_passive` runs its own `mj_forward` on the shared `mjData`, and
        racing that against `mj_step` is exactly what corrupts the solver.
        """
        import mujoco
        from strands_robots import Robot

        self._mj = mujoco
        # mesh=False: the lab auto-joins a Zenoh peer-to-peer mesh otherwise,
        # which needs the `mesh` extra and a network we do not want in a demo.
        self._sim = Robot(self._robot_name, mesh=False)
        self._m, self._d = self._sim.mj_model, self._sim.mj_data
        self._lock = self._sim._lock
        self._resolve_indices()
        self._build_scene()
        self._add_cameras()
        # Start standing rather than dropping from the model's 0.445 m spawn
        # height with its legs straight — the fall is ugly and can end on its
        # side before the agent has said a word.
        self._plant_stance()

        if self._want_viewer:
            self._open_viewer()

        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="sim-dog", daemon=True)
        self._thread.start()
        self._started = True

    def close(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        if self._sim is not None:
            try:
                if self._viewer_open:
                    self._sim.close_viewer()
                self._sim.cleanup()
            except Exception:  # noqa: BLE001 — teardown is best-effort
                pass
        self._started = False

    @property
    def started(self) -> bool:
        return self._started

    @property
    def viewer_open(self) -> bool:
        return self._viewer_open

    def _open_viewer(self) -> None:
        """Open the interactive window, tolerating hosts that cannot.

        On macOS `mujoco.viewer.launch_passive` refuses to run under plain
        `python` — it needs the `mjpython` launcher so the window can own the
        UI thread. Headless hosts (containers, EC2) have no display at all.
        Neither is fatal: the sim runs fine unwatched, and `describe_scene`
        renders offscreen regardless.
        """
        res = self._sim.open_viewer()
        if res.get("status") == "success":
            self._viewer_open = True
            return
        note = ""
        for c in res.get("content", []):
            note = c.get("text", note)
        print(f"[SimDog] viewer unavailable: {note}")
        if "mjpython" in note:
            print("[SimDog] run the agent with .venv-sim/bin/mjpython to get a window "
                  "(make voice-sim does this for you)")

    # --- model introspection ----------------------------------------------

    def _resolve_indices(self) -> None:
        """Cache qpos/qvel addresses and limits straight off the model.

        Read rather than hardcoded: `add_robot` namespaces every joint
        (`unitree_go2/FL_hip_joint`) and the free base joint is unnamed, so the
        addresses depend on how the world was assembled.
        """
        mj, m = self._mj, self._m
        for i, (leg, jnt) in enumerate((l, j) for l in LEGS for j in JOINTS):
            name = f"{self._robot_name}/{leg}_{jnt}_joint"
            jid = mj.mj_name2id(m, mj.mjtObj.mjOBJ_JOINT, name)
            if jid < 0:  # un-namespaced fallback
                jid = mj.mj_name2id(m, mj.mjtObj.mjOBJ_JOINT, f"{leg}_{jnt}_joint")
            if jid < 0:
                raise RuntimeError(f"joint {name} not found in the compiled model")
            self._qadr[i] = m.jnt_qposadr[jid]
            self._vadr[i] = m.jnt_dofadr[jid]
            self._lo[i], self._hi[i] = m.jnt_range[jid]

        free = [j for j in range(m.njnt) if m.jnt_type[j] == mj.mjtJoint.mjJNT_FREE]
        if not free:
            raise RuntimeError("the Go2 model has no floating base joint")
        self._base_qadr = int(m.jnt_qposadr[free[0]])
        self._base_vadr = int(m.jnt_dofadr[free[0]])

        # Torque ceilings from the model itself: +/-23.7 Nm hips and thighs,
        # +/-45.43 Nm knees. Clipping to these keeps the PD law inside what the
        # real actuators could deliver.
        rng = m.actuator_ctrlrange
        self._tau_lim = np.array([max(abs(rng[i][0]), abs(rng[i][1])) or 23.7
                                  for i in range(m.nu)], dtype=float)

    def _build_scene(self) -> None:
        r = config.SIM_BODY_RADIUS
        for spec in SCENES.get(self._scene_name, LAB_SCENE):
            res = self._sim.add_object(**spec)
            if res.get("status") != "success":
                print(f"[SimDog] add_object({spec['name']}) failed: {res}")
                continue
            if spec.get("is_static"):
                px, py = spec["position"][0], spec["position"][1]
                sx, sy = spec["size"][0] / 2.0, spec["size"][1] / 2.0
                self._obstacles.append((px - sx - r, px + sx + r, py - sy - r, py + sy + r))
        # add_object recompiles the MJCF, which hands back fresh model/data
        # objects — re-bind or every later write lands on the discarded ones.
        self._rebind()

    def _add_cameras(self) -> None:
        """Mount the head camera, and a chase camera for looking at the dog.

        Both are parented to `<robot>/base`, so `position`/`target` are in the
        base's local frame and the views ride the body — pitching when the dog
        pitches, swinging when it turns, exactly like the real front camera.

        The head camera sits where the Go2's own front camera does and is the
        one the vision tools use. The room camera exists because the built-in
        free camera frames the whole model extent: in a 6 m room that renders
        the dog as a speck, which is useless for checking a pose or grabbing a
        screenshot on a host with no viewer window.
        """
        head = self._sim.add_camera(
            config.SIM_CAMERA_NAME,
            position=[0.28, 0.0, 0.03],
            target=[2.5, 0.0, -0.35],
            fov=config.SIM_CAMERA_FOV,
            width=self._cam_w,
            height=self._cam_h,
            parent_body=f"{self._robot_name}/base",
        )
        if head.get("status") != "success":
            raise RuntimeError(f"head camera could not be mounted: {head}")

        room = self._sim.add_camera(
            ROOM_CAMERA,
            position=[-1.5, -2.1, 1.25],
            target=[1.3, 0.35, 0.25],
            fov=60.0,
            width=self._cam_w,
            height=self._cam_h,
        )
        if room.get("status") != "success":
            print(f"[SimDog] room camera unavailable: {room}")
        self._rebind()

    def _rebind(self) -> None:
        """Re-read model/data handles and indices after an MJCF recompile."""
        self._m, self._d = self._sim.mj_model, self._sim.mj_data
        self._resolve_indices()

    def _plant_stance(self) -> None:
        """Place the dog in its stance pose, standing on the floor, at rest."""
        with self._lock:
            self._d.qpos[self._qadr] = POSES["stand"]
            self._d.qpos[self._base_qadr + 2] = 0.30
            self._d.qvel[:] = 0.0
            self._mj.mj_forward(self._m, self._d)

    # --- commands (called from the agent thread) ---------------------------

    def play(self, name: str) -> Motion:
        """Queue a named gesture. Returns the Motion so callers can time it."""
        self._ensure_alive()
        motion = MOTIONS[name]
        with self._cmd.lock:
            self._cmd.motion = motion
            self._cmd.twist = (0.0, 0.0, 0.0)   # a gesture cancels walking
            self._cmd.twist_until = 0.0
        self._motion_done.clear()
        return motion

    def wait_for_motion(self, motion: Motion, extra: float = 1.5) -> bool:
        """Block until the gesture finishes. False means it was still going.

        The wait is scaled by the measured realtime factor, because a gesture's
        duration is in SIM seconds while this blocks in WALL seconds. Waiting a
        flat `duration + margin` looks fine at 1.0x and then reports perfectly
        good gestures as unfinished the moment the host is busy — which the agent
        faithfully relays to the user as a failure.
        """
        rtf = max(0.15, self._realtime_factor)
        return self._motion_done.wait(timeout=motion.duration / rtf + extra)

    def set_twist(self, vx: float, vy: float, vyaw: float, duration: float) -> None:
        """Walk at a body-frame twist for `duration` seconds, then stop.

        The deadline lives with the setpoint so the dog stops on its own even if
        the caller dies mid-walk — the same watchdog shape the real driver has.
        """
        self._ensure_alive()
        self._blocked = False        # stale from a previous walk otherwise
        self._progress_ref = None
        with self._cmd.lock:
            self._cmd.twist = (vx, vy, vyaw)
            self._cmd.twist_until = time.time() + duration

    def halt(self) -> None:
        with self._cmd.lock:
            self._cmd.twist = (0.0, 0.0, 0.0)
            self._cmd.twist_until = 0.0

    def grab_frame(self):
        """Render the head camera to a PIL RGB image.

        `get_frame` takes the same lock the physics thread holds, so this
        returns a coherent frame rather than one torn mid-step.
        """
        from PIL import Image

        self._ensure_alive()
        rgb, _ = self._sim.get_frame(config.SIM_CAMERA_NAME)
        return Image.fromarray(rgb)

    def grab_world_frame(self):
        """Render the room camera — the dog seen from the corner of the lab.

        Not what the agent's vision tools use; this is for eyeballing a pose or
        grabbing a screenshot on a host with no viewer window.
        """
        from PIL import Image

        rgb, _ = self._sim.get_frame(ROOM_CAMERA)
        return Image.fromarray(rgb)

    def state(self) -> dict:
        """Base pose, posture and sim health — a snapshot, cheap to call."""
        b = self._base_qadr
        with self._lock:
            pos = [float(v) for v in self._d.qpos[b:b + 3]]
            quat = [float(v) for v in self._d.qpos[b + 3:b + 7]]
            sim_t = float(self._d.time)
        w, x, y, z = quat
        yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
        if self._fault is not None:
            return {"fault": self._fault, "posture": "unknown",
                    "body_height": round(pos[2], 3),
                    "position": [round(v, 3) for v in pos],
                    "yaw_degrees": round(math.degrees(yaw), 1),
                    "obstacle_ahead": False, "sim_time": round(sim_t, 2),
                    "realtime_factor": 0.0, "viewer": self._viewer_open}
        return {
            "position": [round(v, 3) for v in pos],
            "body_height": round(pos[2], 3),
            "yaw_degrees": round(math.degrees(yaw), 1),
            "posture": "walking" if self._moving else self._pose_label,
            "obstacle_ahead": self._blocked,
            "sim_time": round(sim_t, 2),
            "realtime_factor": round(self._realtime_factor, 2),
            "viewer": self._viewer_open,
        }

    def object_names(self) -> list:
        return [o["name"] for o in SCENES.get(self._scene_name, LAB_SCENE)]

    # --- the sim thread ----------------------------------------------------

    def _run(self) -> None:
        """Run the physics loop, and record why if it ever stops.

        A daemon thread that dies quietly is the worst outcome available here:
        the dog would freeze mid-pose while every tool kept returning success,
        and the agent would cheerfully narrate a walk that never happened. So the
        loop's failure is captured and re-raised at the next command instead.
        """
        try:
            self._loop()
        except BaseException as e:  # noqa: BLE001 — including MuJoCo's FatalError
            import traceback

            self._fault = f"{type(e).__name__}: {e}"
            print(f"[SimDog] physics loop stopped: {self._fault}")
            traceback.print_exc()

    def _ensure_alive(self) -> None:
        """Raise if the physics loop has stopped, so the tool reports an error."""
        if self._fault is not None:
            raise RuntimeError(
                f"the simulation stopped stepping ({self._fault}) — restart the agent"
            )

    def _loop(self) -> None:
        """Step physics in real time: PD legs, kinematic base drive, viewer sync.

        Batching matters. Sleeping every 2 ms burns the timer's resolution on
        scheduler overhead, so steps go out in ~20 ms batches under one lock
        acquisition and the thread sleeps off whatever wall-clock is left.
        """
        dt = float(self._m.opt.timestep)
        batch = max(1, int(round(0.02 / dt)))
        next_wall = time.perf_counter()
        last_sync = 0.0
        sync_period = 1.0 / 30.0
        # Realtime factor is measured over a rolling window, not since startup.
        # A lifetime average reads 58x in the first millisecond (one 20 ms batch
        # of physics costs a fraction of that in wall time) and then, once it has
        # settled, averages away exactly the transient slowdowns that
        # `wait_for_motion` needs to know about.
        mark_sim, mark_wall = float(self._d.time), time.perf_counter()

        while not self._stop.is_set():
            with self._lock:
                for _ in range(batch):
                    self._tick(dt)
                    self._mj.mj_step(self._m, self._d)

            now = time.perf_counter()
            if self._viewer_open and now - last_sync >= sync_period:
                last_sync = now
                try:
                    handle = getattr(self._sim, "_viewer_handle", None)
                    if handle is not None:
                        handle.sync()
                    else:
                        self._viewer_open = False
                except Exception:  # noqa: BLE001 — a closed window is not an error
                    self._viewer_open = False

            if now - mark_wall >= config.SIM_RTF_WINDOW:
                self._realtime_factor = (float(self._d.time) - mark_sim) / (now - mark_wall)
                mark_sim, mark_wall = float(self._d.time), now
            next_wall += batch * dt
            slack = next_wall - time.perf_counter()
            if slack > 0:
                time.sleep(slack)
            else:
                next_wall = time.perf_counter()   # fell behind; do not spiral

    def _tick(self, dt: float) -> None:
        """One physics step's worth of control: setpoint, torques, base twist."""
        self._advance_motion(dt)
        twist = self._active_twist()
        target = self._target
        if twist is not None:
            target = self._trot_overlay(twist, dt)
            self._drive_base(twist, dt)
        q = self._d.qpos[self._qadr]
        qd = self._d.qvel[self._vadr]
        tau = config.SIM_KP * (target - q) - config.SIM_KD * qd
        self._d.ctrl[:] = np.clip(tau, -self._tau_lim, self._tau_lim)

    def _active_twist(self):
        """The twist to apply this step, or None when the dog should hold still.

        A walk request while the dog is sitting or lying is honoured by standing
        up first — the real robot likewise refuses to walk out of a crouch, and
        silently ignoring the command is the worse failure.
        """
        with self._cmd.lock:
            twist, until = self._cmd.twist, self._cmd.twist_until
        if until <= time.time() or twist == (0.0, 0.0, 0.0):
            if self._moving:
                self._moving = False
                self._gait_phase = 0.0
                self._pose_label = "standing"
                # `_blocked` deliberately survives the stop: move() reads it
                # after the walk has ended, to tell the user why the dog is not
                # where they asked it to be.
                self._progress_ref = None
            return None
        if self._motion is not None:
            return None                      # a gesture is mid-play; let it finish
        if self._pose_label != "standing":
            self._start_motion(MOTIONS["stand_up"])
            return None
        self._moving = True
        return twist

    def _trot_overlay(self, twist, dt: float) -> np.ndarray:
        """Open-loop trot on top of the stance pose.

        Diagonal pairs (FL+RR, then FR+RL) swing in antiphase. Within a leg the
        thigh swings fore/aft (protraction) and the knee tucks on the up-stroke
        so the foot clears the floor. Stride grows with commanded speed, and a
        yaw command biases the left and right strides apart so the dog visibly
        pivots rather than sliding round.
        """
        vx, vy, vyaw = twist
        speed = math.hypot(vx, vy)
        freq = config.SIM_GAIT_FREQ_HZ
        self._gait_phase = (self._gait_phase + 2.0 * math.pi * freq * dt) % (2.0 * math.pi)

        # Stride scales with the fraction of top speed asked for, with a floor
        # so a slow walk still lifts its feet instead of shuffling.
        s_lin = min(1.0, speed / max(config.MAX_LINEAR_VELOCITY, 1e-6))
        s_ang = min(1.0, abs(vyaw) / max(config.MAX_ANGULAR_VELOCITY, 1e-6))
        amp = config.SIM_STRIDE_RAD * max(0.35, max(s_lin, s_ang))
        lift = config.SIM_FOOT_LIFT_RAD * max(0.35, max(s_lin, s_ang))

        target = POSES["stand"].copy()
        for i, leg in enumerate(LEGS):
            phase = self._gait_phase + (0.0 if leg in ("FL", "RR") else math.pi)
            swing = math.sin(phase)
            # Forward walking retracts the leg on the down-stroke; walking
            # backwards mirrors it.
            direction = -1.0 if vx < 0 else 1.0
            # Turning: outside legs take a longer stride than inside ones.
            turn_bias = vyaw * config.SIM_TURN_STRIDE_BIAS * (1.0 if leg in ("FL", "RL") else -1.0)
            target[3 * i + 1] += direction * amp * swing - turn_bias      # thigh
            target[3 * i + 2] += -lift * max(swing, 0.0)                  # calf tucks
            # Sidestepping leans the hips into the direction of travel.
            target[3 * i + 0] += vy * config.SIM_STRAFE_HIP_RAD
        return target

    def _drive_base(self, twist, dt: float) -> None:
        """Integrate the base pose from the commanded twist and write it.

        This is the animated half of the gait (see the module docstring). The
        pose is imposed, not simulated: position from the body-frame twist,
        attitude levelled to yaw only, height held at the walking height with a
        small gait-synced bob so the body rises and falls with its own steps.
        Base velocity is zeroed each step so the leg contacts cannot accumulate
        into the base and throw the dog off course.

        Poses are untouched by any of this — the moment the twist expires the
        base goes back to being fully physical.
        """
        vx, vy, vyaw = twist
        b, v = self._base_qadr, self._base_vadr
        yaw = self._base_yaw() + vyaw * dt
        c, s = math.cos(yaw), math.sin(yaw)

        # An imposed base pose ignores contact, so without this the dog walks
        # straight through the workbench and out through the wall — and the head
        # camera ends up inside a solid geom, rendering a flat brown field that
        # describe_scene then earnestly describes. Each axis is tested on its own
        # so the dog slides along an obstacle instead of sticking to it.
        x, y = float(self._d.qpos[b + 0]), float(self._d.qpos[b + 1])
        nx, ny = x + (vx * c - vy * s) * dt, y + (vx * s + vy * c) * dt
        if self._free(nx, y):
            self._d.qpos[b + 0] = nx
        if self._free(x, ny):
            self._d.qpos[b + 1] = ny
        self._note_progress(x, y, math.hypot(vx, vy) * dt)

        self._d.qpos[b + 2] = config.SIM_WALK_HEIGHT + config.SIM_WALK_BOB * math.sin(
            2.0 * self._gait_phase
        )
        self._d.qpos[b + 3] = math.cos(yaw / 2.0)
        self._d.qpos[b + 4] = 0.0
        self._d.qpos[b + 5] = 0.0
        self._d.qpos[b + 6] = math.sin(yaw / 2.0)
        self._d.qvel[v:v + 6] = 0.0

    def _note_progress(self, x: float, y: float, wanted_step: float) -> None:
        """Track whether the walk is actually getting anywhere.

        "Blocked" is a progress measurement, not an instantaneous footprint
        test: pressed against the workbench, contact jitter nudges the base a
        hair back below the boundary every step, the next step is allowed again,
        and an instantaneous flag flickers true/false at 500 Hz — so whatever
        the agent happened to sample read "not blocked" while the dog stood
        still. Comparing distance covered against distance asked for over a
        window is immune to that.
        """
        now = time.time()
        if self._progress_ref is None:
            self._progress_ref = (x, y, now, 0.0)
            return
        rx, ry, rt, wanted = self._progress_ref
        wanted += wanted_step
        window = now - rt
        if window < config.SIM_PROGRESS_WINDOW:
            self._progress_ref = (rx, ry, rt, wanted)
            return
        moved = math.hypot(x - rx, y - ry)
        # Sticky for the rest of this walk (cleared by `set_twist`), because the
        # question `move()` answers is "was this walk obstructed?", not "is it
        # obstructed right now". A dog that jams against the bench for two
        # seconds and then slides free covers a fraction of what was asked for,
        # and reporting the final window's answer ("clear!") alongside a distance
        # a third of the commanded one is how the agent ends up insisting it
        # walked somewhere it did not.
        if wanted > 0.02 and moved < 0.25 * wanted:
            self._blocked = True
        self._progress_ref = (x, y, now, 0.0)

    def _free(self, x: float, y: float) -> bool:
        """Is the dog's footprint clear of the scene's fixed obstacles at (x, y)?

        Axis-aligned boxes only, from the scene spec, inflated by the body
        radius — the furniture in `SCENES` is all axis-aligned, and this runs
        every physics step. Movable props (the ball, the cube) are deliberately
        left out so the dog can shove them around.
        """
        for x0, x1, y0, y1 in self._obstacles:
            if x0 < x < x1 and y0 < y < y1:
                return False
        return True

    def _base_yaw(self) -> float:
        b = self._base_qadr
        w, x, y, z = self._d.qpos[b + 3:b + 7]
        return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

    def _tipped(self) -> bool:
        """True when the dog is on its side or has collapsed.

        Measured off the base's own up-axis rather than roll/pitch angles, so a
        dog lying at any heading reads the same. The height test catches the
        splayed-legs collapse, which stays nearly level but cannot recover under
        PD alone.
        """
        b = self._base_qadr
        w, x, y, z = self._d.qpos[b + 3:b + 7]
        # World-frame z component of the body's local +z axis.
        up = 1.0 - 2.0 * (x * x + y * y)
        return up < 0.75 or float(self._d.qpos[b + 2]) < 0.16

    def _right_body(self) -> None:
        """Set the dog back on its feet, level, at standing height.

        The real robot's RecoveryStand is a scripted whole-body manoeuvre with
        torque authority we do not model; PD alone cannot roll a Go2 off its
        side. Righting it kinematically keeps "stand up" meaning what the user
        asked for instead of leaving the dog stuck on its back for the rest of
        the session. Position and heading are preserved — only attitude and
        height are corrected.
        """
        b, v = self._base_qadr, self._base_vadr
        yaw = self._base_yaw()
        self._d.qpos[b + 2] = 0.30
        self._d.qpos[b + 3] = math.cos(yaw / 2.0)
        self._d.qpos[b + 4] = 0.0
        self._d.qpos[b + 5] = 0.0
        self._d.qpos[b + 6] = math.sin(yaw / 2.0)
        self._d.qpos[self._qadr] = POSES["stand"]
        self._d.qvel[:] = 0.0
        self._mj.mj_forward(self._m, self._d)

    # --- gesture playback --------------------------------------------------

    def _advance_motion(self, dt: float) -> None:
        """Interpolate the PD setpoint along the active gesture's waypoints."""
        with self._cmd.lock:
            queued = self._cmd.motion
            self._cmd.motion = None
        if queued is not None:
            self._start_motion(queued)

        if self._motion is None:
            return

        span, goal = self._motion.waypoints[self._m_idx]
        self._m_t += dt
        frac = 1.0 if span <= 0 else min(1.0, self._m_t / span)
        self._set_target(self._m_from + (goal - self._m_from) * frac)

        if frac >= 1.0:
            self._m_idx += 1
            self._m_t = 0.0
            self._m_from = goal.copy()
            if self._m_idx >= len(self._motion.waypoints):
                self._pose_label = self._motion.ends_as
                self._motion = None
                self._motion_done.set()

    def _start_motion(self, motion: Motion) -> None:
        if motion.rights and self._tipped():
            self._right_body()
        self._motion = motion
        self._m_idx = 0
        self._m_t = 0.0
        self._m_from = self._target.copy()
        self._motion_done.clear()

    def _set_target(self, target: np.ndarray) -> None:
        """Install a PD setpoint, clamped to the model's own joint limits.

        A setpoint outside `jnt_range` cannot be reached, so the PD term never
        settles and the joint buzzes against its stop.
        """
        self._target = np.clip(target, self._lo, self._hi)
