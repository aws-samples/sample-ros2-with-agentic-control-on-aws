# Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
# SPDX-License-Identifier: MIT-0
"""Simulation transport — the same Go2 client interface, backed by MuJoCo.

Implements `client_registry.Go2ClientProtocol` on top of `sim_dog.SimDog`, so
`main_sim.py` can install it with `set_client()` and every tool in `tools.py`
works unchanged: the agent, the prompt, the Nova Sonic session and the vision
tools have no idea whether they are driving a real dog or a simulated one.

What works exactly as it does on hardware:
  stand_up, stand_down, sit, move, turn_degrees, stop, perform_action (bar the
  flips), get_robot_state, describe_scene, recall_scene, compare_scenes.

What does not, and says so instead of pretending:
  everything downstream of the COCO detector (get_detections, tracking,
  find_object, the greeter). Those are container nodes, not robot features —
  the sim has no detector node. Each returns a
  `status: "error"` with a note the agent can relay, which is the whole point:
  a tool that silently no-ops teaches the model it succeeded, and the dog then
  claims to be following you around an empty room.
"""

from __future__ import annotations

import math
import time
from typing import Optional

from . import config
from .scene_vision import SceneVisionMixin
from .sim_dog import MOTIONS, UNSIMULATED, SimDog

# Tools that exist only because a container node provides them. Keyed by the
# note the agent gets back, so the model can explain the gap in its own words.
_NEEDS_DETECTOR = (
    "That needs the camera's object detector, which runs as a node in the robot's "
    "ROS 2 container and is not part of the simulation. Say so plainly — do not "
    "pretend to see or follow anything."
)


class Go2SimClient(SceneVisionMixin):
    """Drive a simulated Go2. Same verbs as `Go2ROS2Client`, no robot required."""

    def __init__(self, scene: Optional[str] = None, viewer: bool = False):
        self._dog = SimDog(
            scene=scene or config.SIM_SCENE,
            viewer=viewer,
            robot_name=config.SIM_ROBOT,
            camera_size=(config.SIM_CAMERA_WIDTH, config.SIM_CAMERA_HEIGHT),
        )
        self._connected = False
        self._init_scene_vision()

    # --- lifecycle ---------------------------------------------------------

    def connect(self) -> bool:
        """Build the world and start stepping. Mirrors the ROS client's connect().

        Slow the first time only: the Go2's MJCF and meshes are pulled from
        MuJoCo Menagerie by `robot_descriptions` and cached under
        ~/.cache/robot_descriptions, which is a ~40 s clone. Afterwards this is
        a couple of seconds.
        """
        self._dog.start()
        self._connected = True
        return True

    def disconnect(self) -> None:
        self._dog.close()
        self._connected = False

    @property
    def is_connected(self) -> bool:
        return self._connected

    # --- vision hooks ------------------------------------------------------

    def _grab_live_frame(self, timeout: float = 8.0):
        """Render the head camera. Unlike the bridge, this can never be stale."""
        return self._dog.grab_frame()

    # --- posture -----------------------------------------------------------

    def stand_up(self) -> dict:
        motion = self._dog.play("stand_up")
        self._dog.wait_for_motion(motion)
        return {"status": "success", "action": "stand_up",
                "body_height": self._dog.state()["body_height"]}

    def stand_down(self) -> dict:
        motion = self._dog.play("stand_down")
        self._dog.wait_for_motion(motion)
        height = self._dog.state()["body_height"]
        return {"status": "success", "action": "stand_down", "body_height": height,
                "note": "crouched" if height < 0.15 else "lowered"}

    def balance_stand(self) -> dict:
        motion = self._dog.play("balance_stand")
        self._dog.wait_for_motion(motion)
        return {"status": "success", "action": "balance_stand"}

    def sit(self) -> dict:
        motion = self._dog.play("sit")
        self._dog.wait_for_motion(motion)
        return {"status": "success", "action": "sit"}

    def rise_sit(self) -> dict:
        motion = self._dog.play("rise_sit")
        self._dog.wait_for_motion(motion)
        return {"status": "success", "action": "rise_sit"}

    # --- locomotion --------------------------------------------------------

    def move(self, vx: float = 0.0, vy: float = 0.0, vyaw: float = 0.0,
             duration: float = 2.0) -> dict:
        """Walk at a body-frame twist for `duration` seconds, then stop.

        Blocks for the duration, like the ROS transport does, and reports what
        actually happened rather than what was asked for: the sim knows exactly
        how far the dog got, so a walk that ended up wedged against the
        workbench comes back with `blocked: True` and the distance covered. That
        is strictly better feedback than the real robot can give, and the agent
        uses it to tell the user why the dog is not where they wanted it.
        """
        vx = max(-config.MAX_LINEAR_VELOCITY, min(config.MAX_LINEAR_VELOCITY, vx))
        vy = max(-config.MAX_LINEAR_VELOCITY, min(config.MAX_LINEAR_VELOCITY, vy))
        vyaw = max(-config.MAX_ANGULAR_VELOCITY, min(config.MAX_ANGULAR_VELOCITY, vyaw))
        duration = min(duration, 10.0)

        before = self._dog.state()
        self._dog.set_twist(vx, vy, vyaw, duration)
        time.sleep(duration + 0.3)   # +settle, so the report is of a stopped dog
        after = self._dog.state()

        dx = after["position"][0] - before["position"][0]
        dy = after["position"][1] - before["position"][1]
        travelled = round((dx * dx + dy * dy) ** 0.5, 2)
        turned = round(after["yaw_degrees"] - before["yaw_degrees"], 1)

        result = {
            "status": "success", "action": "move",
            "vx": vx, "vy": vy, "vyaw": vyaw, "duration": duration,
            "travelled_metres": travelled,
            "turned_degrees": turned,
            "blocked": after["obstacle_ahead"],
        }
        if after["obstacle_ahead"]:
            result["note"] = (
                "Something in the room stopped the dog short — it is up against "
                "furniture or a wall. Tell the user, and turn before walking again."
            )
        return result

    def turn_degrees(self, degrees: float, speed: Optional[float] = None) -> dict:
        """Rotate in place by `degrees` (positive = left/CCW) and report the result.

        Open-loop command, measured report: the sim is told to turn for
        `angle / speed` seconds and then the achieved rotation is read back out
        of the dog's pose. `closed_loop` is True because the number returned is a
        real measurement — unlike the hardware transport there is no odometry
        drift or unmet velocity command between the two.
        """
        if speed is None:
            speed = config.GREETER_TURN_SPEED
        speed = min(abs(speed), config.MAX_ANGULAR_VELOCITY)
        if not speed or not degrees:
            return {"status": "success", "action": "turn_degrees",
                    "requested_degrees": round(degrees, 1),
                    "measured_degrees": 0.0, "closed_loop": True}

        vyaw = speed if degrees > 0 else -speed
        duration = math.radians(abs(degrees)) / speed

        # Accumulated from samples taken DURING the turn, not from a single
        # before/after pair: yaw_degrees wraps at +/-180, so any one difference
        # saturates there and a 270 deg turn would report 90. Increments between
        # samples are far from the seam, so summing them measures any angle.
        prev = self._dog.state()["yaw_degrees"]
        turned = 0.0
        self._dog.set_twist(0.0, 0.0, vyaw, duration)
        end = time.time() + duration + 0.3   # +settle, so the report is of a stopped dog
        while time.time() < end:
            time.sleep(0.02)
            now = self._dog.state()["yaw_degrees"]
            turned += (now - prev + 180.0) % 360.0 - 180.0
            prev = now

        return {
            "status": "success", "action": "turn_degrees",
            "requested_degrees": round(degrees, 1),
            "measured_degrees": round(abs(turned), 1),
            "closed_loop": True,
        }

    def stop(self) -> dict:
        self._dog.halt()
        return {"status": "success", "action": "stop"}

    def perform_special_motion(self, motion_name: str) -> dict:
        """Play a gesture by the same names the real robot's sport modes use."""
        name = motion_name.lower()

        if name in UNSIMULATED:
            return {
                "status": "error", "action": "special_motion", "motion": name,
                "note": (
                    f"The simulated dog cannot do {UNSIMULATED[name]} — it is a "
                    "ballistic move the real robot's firmware performs and the "
                    "simulation has no controller for. Tell the user it is a "
                    "hardware-only trick and offer a dance, a wave or a stretch."
                ),
            }

        if name not in MOTIONS:
            return {"status": "error", "action": "special_motion",
                    "message": f"Unknown motion '{motion_name}'. "
                               f"Available: {sorted(MOTIONS)}"}

        motion = self._dog.play(name)
        finished = self._dog.wait_for_motion(motion)
        return {"status": "success", "action": "special_motion", "motion": name,
                "completed": finished}

    # --- state -------------------------------------------------------------

    def get_state(self) -> dict:
        state = self._dog.state()
        if state.get("fault"):
            return {"status": "error", "action": "get_robot_state",
                    "simulated": True,
                    "note": f"The simulation has stopped stepping ({state['fault']}). "
                            "Tell the user the simulated robot needs restarting; "
                            "nothing you command will move it."}
        return {
            "status": "success",
            "connected": self._connected,
            "connection_type": f"mujoco simulation ({config.SIM_ROBOT})",
            "body_height": state["body_height"],
            "posture": state["posture"],
            "position": state["position"],
            "heading_degrees": state["yaw_degrees"],
            "simulated": True,
            "sim": {
                "scene": self._dog._scene_name,
                "sim_time": state["sim_time"],
                "realtime_factor": state["realtime_factor"],
                "viewer_open": state["viewer"],
            },
        }

    # --- detector-dependent tools (absent in simulation) -------------------

    def get_detections(self) -> dict:
        return {"status": "error", "action": "get_detections", "note": _NEEDS_DETECTOR}

    def start_tracking(self, target: str = "person") -> dict:
        return {"status": "error", "action": "start_tracking", "note": _NEEDS_DETECTOR}

    def stop_tracking(self) -> dict:
        return {"status": "success", "action": "stop_tracking",
                "note": "Nothing was being tracked — there is no tracker in simulation."}

    def find_object(self, target: str) -> dict:
        return {"status": "error", "action": "find_object", "target": target,
                "note": _NEEDS_DETECTOR}

    def get_find_status(self) -> dict:
        return {"status": "error", "action": "get_find_status", "note": _NEEDS_DETECTOR}

    def stop_find(self) -> dict:
        return {"status": "success", "action": "stop_find",
                "note": "No search was running — there is no detector in simulation."}

    def greet_visitor(self, wait_seconds: float = 0.0) -> dict:
        return {"status": "error", "action": "greet_visitor", "note": _NEEDS_DETECTOR}

    def start_greeter(self) -> dict:
        return {"status": "error", "action": "start_greeter", "note": _NEEDS_DETECTOR}

    def stop_greeter(self) -> dict:
        return {"status": "success", "action": "stop_greeter",
                "note": "Greeter mode was not running — it needs the detector."}

    def get_greeter_status(self) -> dict:
        return {"status": "error", "action": "get_greeter_status", "note": _NEEDS_DETECTOR}

