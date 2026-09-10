# Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
# SPDX-License-Identifier: MIT-0
"""WebRTC-based client for the Unitree Go2 Air robot (MCF mode).

Used by `main_sonic.py` — the *direct* transport that bypasses this SDK's ROS2
container and talks straight to the robot over WiFi via the unitree_webrtc_connect
library (Unitree's TURN server for remote mode, or AP/STA for local mode).

The Go2 with firmware >= 1.1.7 defaults to MCF (Motion Control Framework) mode,
which uses different API IDs than classic "normal" sport mode.

Exposes the same public method surface as `Go2ROS2Client`, so the Strands tools
in `tools.py` work against either transport unchanged.
"""

import asyncio
import json
import random
import time
import threading
from typing import Optional

from unitree_webrtc_connect import (
    UnitreeWebRTCConnection,
    WebRTCConnectionMethod,
    RTC_TOPIC,
    SPORT_CMD_MCF,
)

from . import config


class Go2Client:
    """Async WebRTC client for the Unitree Go2 Air (MCF mode).

    Connects via Unitree's TURN server (remote) or directly (local AP/STA) and
    sends MCF sport mode commands over the WebRTC data channel.
    """

    def __init__(self):
        self.conn: Optional[UnitreeWebRTCConnection] = None
        self._connected = False
        self._loop: Optional[asyncio.AbstractEventLoop] = None
        self._thread: Optional[threading.Thread] = None
        self._state = {
            "mode": None,
            "progress": None,
            "body_height": None,
        }

    def connect(self) -> bool:
        """Establish WebRTC connection to the robot."""
        try:
            self._loop = asyncio.new_event_loop()
            self._thread = threading.Thread(target=self._run_loop, daemon=True)
            self._thread.start()

            future = asyncio.run_coroutine_threadsafe(self._connect(), self._loop)
            future.result(timeout=30)

            self._connected = True
            print("[Go2Client] Connected via WebRTC")
            return True

        except Exception as e:
            print(f"[Go2Client] Connection failed: {e}")
            self._connected = False
            return False

    async def _connect(self):
        """Internal async connection setup."""
        if config.CONN_TYPE == "remote":
            password = config.get_unitree_password()
            self.conn = UnitreeWebRTCConnection(
                WebRTCConnectionMethod.Remote,
                serialNumber=config.ROBOT_SERIAL,
                username=config.UNITREE_EMAIL,
                password=password,
            )
        elif config.CONN_TYPE == "local":
            kwargs = {}
            if config.AES_128_KEY:
                kwargs["aes_128_key"] = config.AES_128_KEY

            if config.LOCAL_MODE == "ap":
                self.conn = UnitreeWebRTCConnection(
                    WebRTCConnectionMethod.LocalAP, **kwargs
                )
            else:  # sta
                self.conn = UnitreeWebRTCConnection(
                    WebRTCConnectionMethod.LocalSTA,
                    ip=config.ROBOT_IP,
                    **kwargs,
                )
        else:
            raise ValueError(
                f"Unknown UNITREE_CONN_TYPE: '{config.CONN_TYPE}'. Use 'remote' or 'local'."
            )

        await self.conn.connect()

        # Subscribe to sport mode state
        self.conn.datachannel.pub_sub.subscribe(
            RTC_TOPIC["LF_SPORT_MOD_STATE"], self._on_state
        )
        await asyncio.sleep(1)

    def _on_state(self, message):
        """Callback for sport mode state updates."""
        data = message.get("data", {})
        self._state["mode"] = data.get("mode")
        self._state["progress"] = data.get("progress")
        self._state["body_height"] = data.get("body_height")

    def _run_loop(self):
        """Run the asyncio event loop in a background thread."""
        asyncio.set_event_loop(self._loop)
        self._loop.run_forever()

    def _call_sync(self, coro, timeout=10):
        """Run an async coroutine from the sync context."""
        self._ensure_connected()
        future = asyncio.run_coroutine_threadsafe(coro, self._loop)
        return future.result(timeout=timeout)

    async def _mcf_call(self, api_id: int, parameter=None):
        """Request/response MCF sport call."""
        payload = {"api_id": api_id}
        if parameter is not None:
            payload["parameter"] = parameter
        response = await self.conn.datachannel.pub_sub.publish_request_new(
            RTC_TOPIC["SPORT_MOD"], payload
        )
        code = response.get("data", {}).get("header", {}).get("status", {}).get("code", -1)
        data = response.get("data", {}).get("data", "")
        return code, data

    def _mcf_move_fire(self, vx=0.0, vy=0.0, vyaw=0.0):
        """Fire-and-forget MCF Move command (no reply). Must be sent repeatedly."""
        api_id = SPORT_CMD_MCF["Move"]
        generated_id = int(time.time() * 1000) % 2147483648 + random.randint(0, 1000)
        request_payload = {
            "header": {
                "identity": {"id": generated_id, "api_id": api_id},
                "policy": {"priority": 0, "noreply": True},
            },
            "parameter": json.dumps({"x": vx, "y": vy, "z": vyaw}),
            "binary": [],
        }
        self.conn.datachannel.pub_sub.publish_without_callback(
            RTC_TOPIC["SPORT_MOD"], request_payload
        )

    @property
    def is_connected(self) -> bool:
        return self._connected

    # --- Posture Commands ---

    def stand_up(self) -> dict:
        """Recovery stand + BalanceStand to prepare for commands."""
        code, _ = self._call_sync(self._mcf_call(SPORT_CMD_MCF["RecoveryStand"]))
        time.sleep(2)
        code2, _ = self._call_sync(self._mcf_call(SPORT_CMD_MCF["BalanceStand"]))
        return {"status": "success", "action": "stand_up", "codes": [code, code2]}

    def stand_down(self) -> dict:
        """Robot crouches down (safe to power off)."""
        code, _ = self._call_sync(self._mcf_call(SPORT_CMD_MCF["StandDown"]))
        return {"status": "success", "action": "stand_down", "code": code}

    def balance_stand(self) -> dict:
        """Balance stand — prepares robot for gestures and movement."""
        code, _ = self._call_sync(self._mcf_call(SPORT_CMD_MCF["BalanceStand"]))
        return {"status": "success", "action": "balance_stand", "code": code}

    def sit(self) -> dict:
        """Robot sits."""
        code, _ = self._call_sync(self._mcf_call(SPORT_CMD_MCF["Sit"]))
        return {"status": "success", "action": "sit", "code": code}

    def rise_sit(self) -> dict:
        """Robot stands up from sitting."""
        code, _ = self._call_sync(self._mcf_call(SPORT_CMD_MCF["RiseSit"]))
        return {"status": "success", "action": "rise_sit", "code": code}

    # --- Movement Commands ---

    def move(self, vx: float = 0.0, vy: float = 0.0, vyaw: float = 0.0,
             duration: float = 2.0) -> dict:
        """Send velocity command repeatedly for a duration then stop.

        Args:
            vx: Forward/backward (m/s). Positive = forward.
            vy: Left/right (m/s). Positive = left.
            vyaw: Rotation (rad/s). Positive = counter-clockwise.
            duration: How long to move (seconds, max 10).
        """
        self._ensure_connected()

        # Clamp to safety limits
        vx = max(-config.MAX_LINEAR_VELOCITY, min(config.MAX_LINEAR_VELOCITY, vx))
        vy = max(-config.MAX_LINEAR_VELOCITY, min(config.MAX_LINEAR_VELOCITY, vy))
        vyaw = max(-config.MAX_ANGULAR_VELOCITY, min(config.MAX_ANGULAR_VELOCITY, vyaw))
        duration = min(duration, 10.0)

        # Run the movement loop in the async event loop to avoid blocking issues
        async def _do_move():
            start = time.time()
            while time.time() - start < duration:
                self._mcf_move_fire(vx, vy, vyaw)
                await asyncio.sleep(0.1)
            await self._mcf_call(SPORT_CMD_MCF["StopMove"])

        future = asyncio.run_coroutine_threadsafe(_do_move(), self._loop)
        future.result(timeout=duration + 5)

        return {
            "status": "success",
            "action": "move",
            "vx": vx, "vy": vy, "vyaw": vyaw,
            "duration": duration,
        }

    def stop(self) -> dict:
        """Stop all movement."""
        code, _ = self._call_sync(self._mcf_call(SPORT_CMD_MCF["StopMove"]))
        return {"status": "success", "action": "stop", "code": code}

    # --- Special Motions (MCF) ---

    def perform_special_motion(self, motion_name: str) -> dict:
        """Execute a named special motion using MCF API IDs.

        Available motions:
            hello, stretch, content, dance1, dance2, heart, scrape,
            front_flip, front_jump, front_pounce, back_flip, left_flip,
            sit, rise_sit, stand_up, stand_down, balance_stand
        """
        motion_map = {
            "hello": SPORT_CMD_MCF["Hello"],
            "stretch": SPORT_CMD_MCF["Stretch"],
            "content": SPORT_CMD_MCF["Content"],
            "dance1": SPORT_CMD_MCF["Dance1"],
            "dance2": SPORT_CMD_MCF["Dance2"],
            "heart": SPORT_CMD_MCF["Heart"],
            "scrape": SPORT_CMD_MCF["Scrape"],
            "front_flip": SPORT_CMD_MCF["FrontFlip"],
            "front_jump": SPORT_CMD_MCF["FrontJump"],
            "front_pounce": SPORT_CMD_MCF["FrontPounce"],
            "back_flip": SPORT_CMD_MCF["BackFlip"],
            "left_flip": SPORT_CMD_MCF["LeftFlip"],
            "sit": SPORT_CMD_MCF["Sit"],
            "rise_sit": SPORT_CMD_MCF["RiseSit"],
            "stand_up": SPORT_CMD_MCF["RecoveryStand"],
            "stand_down": SPORT_CMD_MCF["StandDown"],
            "balance_stand": SPORT_CMD_MCF["BalanceStand"],
        }

        cmd_id = motion_map.get(motion_name.lower())
        if cmd_id is None:
            available = list(motion_map.keys())
            return {
                "status": "error",
                "message": f"Unknown motion '{motion_name}'. Available: {available}",
            }

        # Some motions need a parameter
        needs_param = {"front_flip", "back_flip", "left_flip"}
        param = {"data": True} if motion_name.lower() in needs_param else None

        code, _ = self._call_sync(self._mcf_call(cmd_id, param))
        return {"status": "success", "action": "special_motion", "motion": motion_name, "code": code}

    # --- State ---

    def get_state(self) -> dict:
        """Get current robot state."""
        try:
            code, data = self._call_sync(
                self._mcf_call(
                    SPORT_CMD_MCF["GetState"],
                    ["state", "bodyHeight", "speedLevel", "gait"]
                )
            )
            mcf_state = json.loads(data) if code == 0 and data else {}
        except Exception:
            mcf_state = {}

        return {
            "status": "success",
            "connected": self._connected,
            "connection_type": f"webrtc ({config.CONN_TYPE})",
            "body_height": self._state.get("body_height"),
            "mcf_state": mcf_state,
        }

    # --- Scene description ---

    def describe_scene(self, question: str = "") -> dict:
        """Scene description is only wired for the ROS2/Foxglove transport.

        The ROS2 path grabs /camera/image_raw off the foxglove bridge and calls
        Bedrock host-side. The direct-WebRTC path doesn't expose the camera that
        way, so steer the user to the container path instead.
        """
        return {
            "status": "error",
            "message": (
                "Scene description requires the ROS2 container path. "
                "Run the agent with `python -m voice_agent.main` (with the SDK "
                "container running so the camera is on the foxglove bridge)."
            ),
        }

    def recall_scene(self, question: str = "", seconds_ago: float = 30.0) -> dict:
        """Temporal recall is only wired for the ROS2/Foxglove transport.

        Robot memory is built from camera snapshots taken during describe_scene
        calls, which only the ROS2 path supports.
        """
        return {
            "status": "error",
            "message": (
                "Robot memory requires the ROS2 container path. "
                "Run the agent with `python -m voice_agent.main`."
            ),
        }

    def compare_scenes(self, question: str = "", seconds_ago: float = 30.0) -> dict:
        """Scene comparison is only wired for the ROS2/Foxglove transport.

        It needs both the camera feed and the snapshot memory, which only the
        ROS2 path provides.
        """
        return {
            "status": "error",
            "message": (
                "Scene comparison requires the ROS2 container path. "
                "Run the agent with `python -m voice_agent.main`."
            ),
        }

    # --- Helpers ---

    def _ensure_connected(self):
        if not self._connected:
            raise RuntimeError("Robot not connected. Call connect() first.")

    def disconnect(self):
        """Gracefully close the WebRTC connection."""
        if self._loop and self.conn:
            try:
                future = asyncio.run_coroutine_threadsafe(
                    self.conn.disconnect(), self._loop
                )
                future.result(timeout=5)
            except Exception:
                pass
        if self._loop and self._loop.is_running():
            self._loop.call_soon_threadsafe(self._loop.stop)
        self._connected = False
