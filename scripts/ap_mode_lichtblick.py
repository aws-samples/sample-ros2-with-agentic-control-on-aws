#!/usr/bin/env python3
# Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
# SPDX-License-Identifier: MIT-0
"""
Standalone AP-mode test: Go2 WebRTC (LocalAP) -> Foxglove WebSocket -> Lichtblick.

Why this exists
---------------
The rest of this repo reaches the dog either over LAN from the ROS2 container
(CONN_TYPE=webrtc -> LocalSTA) or through Unitree's TURN server
(CONN_TYPE=remote) -- and the remote path keeps getting blocked by Tencent's
EdgeOne WAF in front of global-robot-api.unitree.com (HTTP 567 / HTML challenge
page; see Go2Connection.CloudRejectedError).

AP mode sidesteps the cloud completely: you join the robot's OWN WiFi hotspot,
the robot is at a fixed 192.168.12.1, and the entire handshake is local HTTP on
:9991. No login, no access token, no TURN server -- not one request to Unitree.
The per-device AES-128 key is still needed (firmware >= 1.1.15 answers
con_notify with data2=3), but that's already cached in docker/.env.

This script is deliberately standalone -- no ROS2, no container, no colcon:

  1. brings up the LocalAP WebRTC connection (the thing under test),
  2. subscribes to the robot's state topics and the front camera,
  3. serves a minimal Foxglove WebSocket server so Lichtblick can attach and
     you can actually SEE it working.

Usage
-----
    # 1. Join the dog's hotspot on the Mac's WiFi (SSID is usually GO2_xxxxxx;
    #    credentials are on the sticker in the battery bay / in the app).
    # 2. Make sure nothing else holds the robot's single WebRTC slot:
    #    `make down` and close the Unitree mobile app.
    .venv/bin/python scripts/ap_mode_lichtblick.py

    # then in Lichtblick:
    #    Open connection -> Foxglove WebSocket -> ws://localhost:8765

Handshake only, no viewer (fastest AP smoke test, exits immediately):
    .venv/bin/python scripts/ap_mode_lichtblick.py --probe-only

Baseline on ordinary WiFi -- proves the script itself is sound before you go
switch networks (uses ROBOT_IP from docker/.env):
    .venv/bin/python scripts/ap_mode_lichtblick.py --mode sta

Lidar / SLAM / Nav2 are out of scope here: the voxel-map decode and
pointcloud_to_laserscan live in the container. This is camera + telemetry.
"""

from __future__ import annotations

import argparse
import asyncio
import base64
import contextlib
import io
import json
import logging
import os
import socket
import struct
import sys
import time
from typing import Any, Dict, Optional

# Import order matters, same as go2_connection.py: unitree_webrtc_connect must
# be imported before anything drags in aiortc.mediastreams, or aiortc's DTLS
# path gets poisoned and the peer connection hangs at "connecting" forever.
from unitree_webrtc_connect import (  # noqa: E402
    RTC_TOPIC,
    UnitreeWebRTCConnection,
    WebRTCConnectionMethod,
)

AP_ROBOT_IP = "192.168.12.1"
AP_SUBNET_PREFIX = "192.168.12."
SIGNALING_PORT = 9991
FOXGLOVE_SUBPROTOCOL = "foxglove.websocket.v1"

log = logging.getLogger("ap-test")


# ---------------------------------------------------------------------------
# Config
# ---------------------------------------------------------------------------

def read_docker_env() -> Dict[str, str]:
    """Read docker/.env so we reuse the AES key / IP the container uses.

    Same plain KEY=value parse as voice_agent/config.py's _docker_env(); copied
    rather than imported so this file stays runnable from any cwd with no
    package setup. Returns {} if the file isn't there.
    """
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        os.pardir, "docker", ".env")
    out: Dict[str, str] = {}
    try:
        with open(path) as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith("#") or "=" not in line:
                    continue
                k, _, v = line.partition("=")
                # Values in docker/.env may carry a trailing inline comment.
                out[k.strip()] = v.split("#")[0].strip().strip('"').strip("'")
    except OSError:
        pass
    return out


# ---------------------------------------------------------------------------
# Preflight
# ---------------------------------------------------------------------------

def route_source_ip(dst: str) -> Optional[str]:
    """Local address the OS would use to reach `dst`. Sends no packets."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect((dst, 1))
        return s.getsockname()[0]
    except OSError:
        return None
    finally:
        s.close()


def probe_tcp(host: str, port: int, timeout: float = 3.0) -> bool:
    with contextlib.closing(socket.socket()) as s:
        s.settimeout(timeout)
        return s.connect_ex((host, port)) == 0


def preflight(mode: str, robot_ip: str, aes_key: str) -> bool:
    """Fail loudly and specifically instead of timing out inside the handshake."""
    ok = True

    src = route_source_ip(robot_ip)
    print(f"[i] route to {robot_ip} leaves via local address: {src or 'NO ROUTE'}")
    if mode == "ap" and not (src or "").startswith(AP_SUBNET_PREFIX):
        print(f"[!] expected a {AP_SUBNET_PREFIX}x source address. Your Mac does not look "
              f"like it is joined to the robot's hotspot.")
        ok = False

    if probe_tcp(robot_ip, SIGNALING_PORT):
        print(f"[ok] signaling port {robot_ip}:{SIGNALING_PORT} is open")
    else:
        print(f"[!] cannot reach {robot_ip}:{SIGNALING_PORT}")
        if mode == "ap":
            print("    - is the Mac's WiFi joined to the dog's hotspot (SSID GO2_xxxxxx)?")
            print("    - is the dog awake? it drops WiFi in standby / on low battery")
        else:
            print("    - is ROBOT_IP in docker/.env still current? (`make find-robot`)")
            print("    - corporate WiFi client isolation silently blocks this")
        ok = False

    if not aes_key:
        print("[!] no AES_128_KEY. Firmware >=1.1.15 answers con_notify with data2=3 and "
              "the handshake cannot decrypt the robot's public key without it. "
              "Set AES_128_KEY in docker/.env or in the environment.")
        ok = False
    elif len(aes_key) != 32:
        print(f"[!] AES_128_KEY is {len(aes_key)} chars, expected 32 hex chars")
        ok = False
    else:
        print(f"[ok] AES key present ({aes_key[:4]}...)")

    print("[i] the robot serves ONE WebRTC peer at a time — if this hangs or the robot "
          "answers 'reject', run `make down` and close the Unitree mobile app first")
    return ok


# ---------------------------------------------------------------------------
# Foxglove WebSocket server (minimal, read-only side of protocol v1)
# ---------------------------------------------------------------------------

TIME_SCHEMA = {
    "type": "object",
    "title": "time",
    "properties": {
        "sec": {"type": "integer", "minimum": 0},
        "nsec": {"type": "integer", "minimum": 0, "maximum": 999999999},
    },
}

COMPRESSED_IMAGE_SCHEMA = {
    "title": "foxglove.CompressedImage",
    "type": "object",
    "properties": {
        "timestamp": TIME_SCHEMA,
        "frame_id": {"type": "string"},
        "data": {"type": "string", "contentEncoding": "base64"},
        "format": {"type": "string"},
    },
}

# Curated scalars, so Lichtblick's Plot / Gauge / State-transition panels have
# concrete numeric message paths to bind to (the raw passthrough topics below
# are for the Raw Messages panel).
ROBOT_STATE_SCHEMA = {
    "title": "go2.RobotState",
    "type": "object",
    "properties": {
        "timestamp": TIME_SCHEMA,
        "battery_soc": {"type": "integer"},
        "power_v": {"type": "number"},
        "power_a": {"type": "number"},
        "mode": {"type": "integer"},
        "gait_type": {"type": "integer"},
        "progress": {"type": "number"},
        "body_height": {"type": "number"},
        "vel_x": {"type": "number"},
        "vel_y": {"type": "number"},
        "yaw_speed": {"type": "number"},
        "roll": {"type": "number"},
        "pitch": {"type": "number"},
        "yaw": {"type": "number"},
    },
}

RAW_JSON_SCHEMA = {"type": "object", "additionalProperties": True}


class FoxgloveServer:
    """Just enough of the Foxglove WebSocket server protocol for a viewer.

    Implements serverInfo, advertise, subscribe/unsubscribe and binary
    MESSAGE_DATA frames with JSON-encoded payloads. No client publishing,
    services, parameters or time control -- none of which a read-only test
    viewer needs.
    """

    OP_MESSAGE_DATA = 0x01

    def __init__(self) -> None:
        self._channels: Dict[int, dict] = {}
        self._next_id = 1
        self._subs: Dict[Any, Dict[int, int]] = {}   # ws -> {sub_id: channel_id}
        self._last_sent: Dict[int, float] = {}

    def add_channel(self, topic: str, schema_name: str, schema: dict) -> int:
        cid = self._next_id
        self._next_id += 1
        self._channels[cid] = {
            "id": cid,
            "topic": topic,
            "encoding": "json",
            "schemaName": schema_name,
            "schema": json.dumps(schema),
            "schemaEncoding": "jsonschema",
        }
        return cid

    def subscriber_count(self, channel_id: int) -> int:
        return sum(1 for subs in self._subs.values()
                   for cid in subs.values() if cid == channel_id)

    async def handle_client(self, ws) -> None:
        peer = getattr(ws, "remote_address", None)
        self._subs[ws] = {}
        print(f"[ok] viewer connected: {peer}")
        try:
            await ws.send(json.dumps({
                "op": "serverInfo",
                "name": "go2-ap-mode-test",
                "capabilities": [],
                "supportedEncodings": [],
                "metadata": {},
                "sessionId": str(time.time_ns()),
            }))
            await ws.send(json.dumps({
                "op": "advertise",
                "channels": list(self._channels.values()),
            }))
            async for raw in ws:
                if isinstance(raw, (bytes, bytearray)):
                    continue  # no client publishing
                try:
                    msg = json.loads(raw)
                except json.JSONDecodeError:
                    continue
                op = msg.get("op")
                if op == "subscribe":
                    for s in msg.get("subscriptions", []):
                        cid, sid = s.get("channelId"), s.get("id")
                        if cid in self._channels and sid is not None:
                            self._subs[ws][sid] = cid
                            log.info("subscribed: %s", self._channels[cid]["topic"])
                elif op == "unsubscribe":
                    for sid in msg.get("subscriptionIds", []):
                        self._subs[ws].pop(sid, None)
        except Exception as e:  # ConnectionClosed lands here too
            log.debug("viewer loop ended: %s", e)
        finally:
            self._subs.pop(ws, None)
            print(f"[i] viewer disconnected: {peer}")

    async def send(self, channel_id: int, payload: dict,
                   min_interval: float = 0.0) -> None:
        """Broadcast one JSON message. `min_interval` throttles per channel.

        The robot pushes lowstate/sportmodestate at ~50 Hz, which is more than a
        viewer needs and enough to make the JSON encode show up in the event
        loop; every caller passes a rate cap.
        """
        if not self._subs:
            return
        # Only throttled calls touch the clock. Stamping it on every send would
        # mean an unthrottled send silently swallows the next throttled one on
        # the same channel.
        if min_interval:
            now = time.monotonic()
            if now - self._last_sent.get(channel_id, 0.0) < min_interval:
                return
            self._last_sent[channel_id] = now

        body = json.dumps(payload).encode("utf-8")
        ts = time.time_ns()
        for ws, subs in list(self._subs.items()):
            for sid, cid in subs.items():
                if cid != channel_id:
                    continue
                frame = struct.pack("<BIQ", self.OP_MESSAGE_DATA, sid, ts) + body
                try:
                    await ws.send(frame)
                except Exception:
                    pass  # client went away; handle_client's finally cleans up


def now_time() -> dict:
    ns = time.time_ns()
    return {"sec": ns // 1_000_000_000, "nsec": ns % 1_000_000_000}


def curate(low: dict, sport: dict) -> dict:
    """Flatten the interesting scalars out of lowstate + sportmodestate.

    Field names follow what the container's publisher reads, so they stay
    comparable: ros2_publisher.py (bms_state.soc, power_v, imu_state.rpy) and
    robot_data_service.py (mode, gait_type, body_height, velocity).
    """
    bms = low.get("bms_state") or {}
    imu = sport.get("imu_state") or low.get("imu_state") or {}
    rpy = list(imu.get("rpy") or [0.0, 0.0, 0.0])
    vel = list(sport.get("velocity") or [0.0, 0.0, 0.0])
    return {
        "timestamp": now_time(),
        "battery_soc": int(bms.get("soc") or 0),
        "power_v": float(low.get("power_v") or 0.0),
        "power_a": float(low.get("power_a") or 0.0),
        "mode": int(sport.get("mode") or 0),
        "gait_type": int(sport.get("gait_type") or 0),
        "progress": float(sport.get("progress") or 0.0),
        "body_height": float(sport.get("body_height") or 0.0),
        "vel_x": float(vel[0]) if len(vel) > 0 else 0.0,
        "vel_y": float(vel[1]) if len(vel) > 1 else 0.0,
        "yaw_speed": float(vel[2]) if len(vel) > 2 else 0.0,
        "roll": float(rpy[0]) if len(rpy) > 0 else 0.0,
        "pitch": float(rpy[1]) if len(rpy) > 1 else 0.0,
        "yaw": float(rpy[2]) if len(rpy) > 2 else 0.0,
    }


def encode_jpeg(frame, quality: int) -> bytes:
    """av.VideoFrame -> JPEG bytes (Pillow; runs off the event loop thread)."""
    buf = io.BytesIO()
    frame.to_image().save(buf, format="JPEG", quality=quality)
    return buf.getvalue()


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

async def run(args: argparse.Namespace) -> int:
    env = read_docker_env()
    aes_key = (os.environ.get("AES_128_KEY") or env.get("AES_128_KEY") or "").strip()
    robot_ip = (AP_ROBOT_IP if args.mode == "ap"
                else (args.ip or os.environ.get("ROBOT_IP")
                      or env.get("ROBOT_IP") or AP_ROBOT_IP))

    print(f"=== Go2 {args.mode.upper()}-mode test -> {robot_ip} ===")
    if not preflight(args.mode, robot_ip, aes_key) and not args.force:
        print("\n[fail] preflight failed. Fix the above, or pass --force to try anyway.")
        return 2

    if args.mode == "ap":
        # The library pins ip=192.168.12.1 for LocalAP itself and sends an empty
        # SDP `id` (LocalSTA sends "STA_localNetwork") -- that difference is the
        # whole reason to use LocalAP rather than just repointing ROBOT_IP.
        conn = UnitreeWebRTCConnection(
            WebRTCConnectionMethod.LocalAP, aes_128_key=aes_key or None)
    else:
        conn = UnitreeWebRTCConnection(
            WebRTCConnectionMethod.LocalSTA, ip=robot_ip,
            aes_128_key=aes_key or None)

    print(f"[..] connecting ({args.mode})... no cloud calls on this path")
    t0 = time.monotonic()
    try:
        await conn.connect()
    except Exception as e:
        print(f"\n[fail] handshake failed after {time.monotonic() - t0:.1f}s: "
              f"{type(e).__name__}: {e}")
        print("    RobotBusyError / 'reject'  -> another peer holds the slot "
              "(container, phone app); stop it and wait ~15s")
        print("    AesKeyRequiredError        -> firmware needs AES_128_KEY")
        print("    AesKeyRejectedError        -> wrong AES key for this robot")
        print("    LocalSignalingPortError    -> neither :9991 nor :8081 answered")
        return 1
    print(f"[ok] handshake complete in {time.monotonic() - t0:.1f}s "
          f"-- data channel open and validated")

    if args.probe_only:
        print("\n[pass] AP mode works. Rerun without --probe-only to view it in Lichtblick.")
        await conn.disconnect()
        return 0

    server = FoxgloveServer()
    ch_state = server.add_channel("/robot/state", "go2.RobotState", ROBOT_STATE_SCHEMA)
    ch_low = server.add_channel("/lowstate", "go2.LowState", RAW_JSON_SCHEMA)
    ch_sport = server.add_channel("/sportmodestate", "go2.SportModeState", RAW_JSON_SCHEMA)
    ch_image = server.add_channel(
        "/camera/image_raw/compressed", "foxglove.CompressedImage",
        COMPRESSED_IMAGE_SCHEMA)

    latest: Dict[str, dict] = {"low": {}, "sport": {}}
    counters = {"low": 0, "sport": 0, "frames": 0}

    # pub_sub callbacks are invoked synchronously from the data channel's
    # on_message coroutine, i.e. already on this event loop -- so scheduling a
    # task here is safe (no call_soon_threadsafe needed).
    def on_lowstate(message):
        latest["low"] = message.get("data") or {}
        counters["low"] += 1
        asyncio.ensure_future(server.send(ch_low, latest["low"], min_interval=0.1))
        asyncio.ensure_future(server.send(
            ch_state, curate(latest["low"], latest["sport"]), min_interval=0.2))

    def on_sportstate(message):
        latest["sport"] = message.get("data") or {}
        counters["sport"] += 1
        asyncio.ensure_future(server.send(ch_sport, latest["sport"], min_interval=0.1))

    conn.datachannel.pub_sub.subscribe(RTC_TOPIC["LOW_STATE"], on_lowstate)
    conn.datachannel.pub_sub.subscribe(RTC_TOPIC["LF_SPORT_MOD_STATE"], on_sportstate)
    print("[ok] subscribed to rt/lf/lowstate + rt/lf/sportmodestate")

    if not args.no_video:
        min_frame_interval = 1.0 / max(args.fps, 1)
        last_frame = [0.0]

        async def on_track(track):
            from aiortc.mediastreams import MediaStreamError
            try:
                while True:
                    frame = await track.recv()
                    counters["frames"] += 1
                    if counters["frames"] == 1:
                        print("[ok] first video frame received "
                              f"({frame.width}x{frame.height})")
                    now = time.monotonic()
                    if now - last_frame[0] < min_frame_interval:
                        continue
                    if server.subscriber_count(ch_image) == 0:
                        continue
                    last_frame[0] = now
                    jpeg = await asyncio.to_thread(encode_jpeg, frame, args.quality)
                    await server.send(ch_image, {
                        "timestamp": now_time(),
                        "frame_id": "front_camera",
                        "format": "jpeg",
                        "data": base64.b64encode(jpeg).decode("ascii"),
                    })
            except MediaStreamError:
                print("[i] video track ended")
            except Exception as e:
                print(f"[!] video loop error: {type(e).__name__}: {e}")

        # Register the callback BEFORE asking the robot to start streaming,
        # otherwise the first frame can land before anyone is listening.
        conn.video.add_track_callback(on_track)
        conn.video.switchVideoChannel(True)
        print("[..] requested front-camera stream")

    async def heartbeat():
        while True:
            await asyncio.sleep(10)
            print(f"[i] lowstate={counters['low']} sport={counters['sport']} "
                  f"frames={counters['frames']} "
                  f"soc={(latest['low'].get('bms_state') or {}).get('soc')}%")

    from websockets.asyncio.server import serve
    async with serve(server.handle_client, args.host, args.port,
                     subprotocols=[FOXGLOVE_SUBPROTOCOL], max_size=None):
        print(f"\n[ok] Foxglove WebSocket server on ws://{args.host}:{args.port}")
        print("     Lichtblick -> Open connection -> Foxglove WebSocket -> "
              f"ws://localhost:{args.port}")
        print("     topics: /robot/state  /lowstate  /sportmodestate  "
              "/camera/image_raw/compressed")
        print("     Ctrl-C to stop.\n")
        hb = asyncio.ensure_future(heartbeat())
        try:
            await asyncio.Event().wait()
        finally:
            hb.cancel()
    return 0


def main() -> int:
    p = argparse.ArgumentParser(
        description="Test Go2 AP mode (no Unitree cloud) and view it in Lichtblick.")
    p.add_argument("--mode", choices=("ap", "sta"), default="ap",
                   help="ap = robot hotspot at 192.168.12.1 (default); "
                        "sta = ordinary shared WiFi, for a baseline comparison")
    p.add_argument("--ip", default=None,
                   help="robot IP for --mode sta (default: ROBOT_IP from docker/.env)")
    p.add_argument("--host", default="127.0.0.1", help="bind address for the viewer server")
    p.add_argument("--port", type=int, default=8765,
                   help="viewer port (8765 = Lichtblick's default; the container "
                        "publishes its bridge on 8766 so both can run)")
    p.add_argument("--fps", type=float, default=10.0, help="max camera frames/s to forward")
    p.add_argument("--quality", type=int, default=70, help="JPEG quality 1-95")
    p.add_argument("--no-video", action="store_true", help="telemetry only")
    p.add_argument("--probe-only", action="store_true",
                   help="do the handshake, report pass/fail, exit")
    p.add_argument("--force", action="store_true", help="connect even if preflight fails")
    p.add_argument("-v", "--verbose", action="store_true", help="library debug logging")
    args = p.parse_args()

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.WARNING,
        format="%(levelname)s %(name)s: %(message)s")
    logging.getLogger("ap-test").setLevel(
        logging.DEBUG if args.verbose else logging.INFO)

    try:
        return asyncio.run(run(args))
    except KeyboardInterrupt:
        print("\n[i] stopped")
        return 0


if __name__ == "__main__":
    sys.exit(main())
