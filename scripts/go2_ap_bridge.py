#!/usr/bin/env python3
# Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
# SPDX-License-Identifier: MIT-0
"""
Inverted bridge: the laptop owns the dog's WebRTC link, the cloud gets the data.

    dog (AP 192.168.12.1)
      │  WebRTC LocalAP        <- laptop INITIATES
      ▼
    laptop: this script
      │  Foxglove client       <- laptop INITIATES (ws://localhost:9876 via `make tunnel`)
      ▼
    EC2 foxglove_bridge :8765  ->  ROS graph: coco_detector (T4), KVS, AgentCore
      │
      └── /cmd_vel_out, /webrtc_req  ──back over the same socket──►  laptop ──►  dog

Why inverted
------------
The container on EC2 cannot hold the robot link when the dog is in AP mode: on the
local path the library builds RTCConfiguration(iceServers=[]), so both peers offer
only host candidates and EC2's 10.x address is unreachable from a dog that is its
own gateway with no uplink. Relaying that needs a TURN server on the laptop's AP
address — and on a managed laptop, corporate pf (Amazon + CrowdStrike anchors)
drops unsolicited inbound on the tunnel, which is exactly what a relay must accept.
Measured: EC2's packets reach utun4 (Ipkts climb) and macOS answers none (Opkts flat).

So every connection here is laptop-initiated, which is what pf permits. The dog
link works because aiortc dials out; the cloud link works because this is a
websocket CLIENT.

Nothing here needs a container, and nothing on EC2 needs an inbound rule: `make
tunnel` is an SSM port-forward the laptop opens.

What EC2 still buys you: the T4 running coco_detector at coco_model:=best (46.7 mAP
vs ~3900 ms/frame on laptop CPU), KVS, and the in-VPC AgentCore runtime — all of
which are ROS nodes or ROS consumers, which is why the data has to land in EC2's ROS
graph rather than in a viewer. foxglove_bridge's clientPublish does exactly that: it
creates real ROS publishers for topics a websocket client advertises. voice_agent
already relies on this to drive the dog, so the mechanism is proven in this repo.

Run the EC2 container WITHOUT its own robot link, or the two will fight over the
dog's single WebRTC slot:  CONN_TYPE=bridge  (anything not in
('webrtc','remote','cyclonedds') makes go2_driver_node skip connect_robots() and
skip the reconnect watchdog, so it still serves the ROS graph and nothing else).

Usage
-----
    make tunnel                       # terminal 1, leave open
    ./scripts/go2_ap_bridge.py        # terminal 2 (joined to the dog's hotspot)

    # or straight at a bridge, e.g. after `make open-ec2`:
    ./scripts/go2_ap_bridge.py --bridge ws://<ec2-public-ip>:8765

    ./scripts/go2_ap_bridge.py --dry-run   # check both links, publish nothing, exit

Published to EC2:
    /camera/image_raw/compressed   sensor_msgs/CompressedImage
    /go2_states                    go2_interfaces/Go2State   (motion state; NO battery)
    /lowstate                      go2_interfaces/LowState   (battery: bms_state.soc)
    /point_cloud2                  sensor_msgs/PointCloud2   (--no-lidar to skip)
    /tf                            tf2_msgs/TFMessage        (odom->base_link)
    /odom                          nav_msgs/Odometry         (closes the loop on turns)
    /joint_states                  sensor_msgs/JointState    (articulates the URDF)
Subscribed from EC2:
    /cmd_vel_out                   geometry_msgs/Twist
    /webrtc_req                    go2_interfaces/WebRtcReq

Still out of scope: SLAM and Nav2 (both are ROS nodes with no equivalent here).

The T4 detector reads the compressed topic automatically here: robot.launch.py
derives coco_image_topic/coco_compressed from CONN_TYPE, so CONN_TYPE=bridge points
it at /camera/image_raw/compressed with no launch arguments and no republisher hop
(a local link still gets the raw topic). Against an image predating that, the
detector sits on raw /camera/image_raw — which nothing here publishes — and goes
silent along with everything downstream of it: 'find a person', 'follow me', the
greeter, the boxes in Lichtblick and the KVS stream. Only 'what do you see?' keeps
working, since it reads this topic off foxglove_bridge directly. Fix by pushing a
current image: make push-image && make restart-ec2.
"""

from __future__ import annotations

import argparse
import asyncio
import contextlib
import io
import json
import os
import struct
import sys
import time
from typing import Callable, Dict, Optional

# unitree_webrtc_connect before anything drags in aiortc.mediastreams — same
# import-order rule as go2_connection.py, or DTLS hangs at "connecting".
from unitree_webrtc_connect import (  # noqa: E402
    RTC_TOPIC,
    UnitreeWebRTCConnection,
    WebRTCConnectionMethod,
)

import aiohttp  # noqa: E402

# Reuse the container's own command builders so the JSON that reaches the dog is
# byte-for-byte what the proven path sends. command_generator.py is pure Python (no
# rclpy), so it imports fine outside the container.
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), os.pardir,
                                "go2_robot_sdk"))
from go2_robot_sdk.application.utils.command_generator import (  # noqa: E402
    gen_mov_command,
)

AP_ROBOT_IP = "192.168.12.1"
DEFAULT_BRIDGE = "ws://localhost:9876"   # what `make tunnel` puts the bridge on

# foxglove_bridge 3.3.0 (Rust SDK) speaks "foxglove.sdk.v1", NOT the older
# "foxglove.websocket.v1", and wants ROS2-style schema names (pkg/msg/Name).
# voice_agent/go2_ros2_client.py learned this the hard way; keep them in step.
FOXGLOVE_SUBPROTOCOL = "foxglove.sdk.v1"

CDR_HEADER = bytes([0x00, 0x01, 0x00, 0x00])  # CDR_LE


# ---------------------------------------------------------------------------
# CDR encoding
#
# XCDR1 little-endian, the ROS2-over-DDS wire format. Fields align to their own
# size, measured from the start of the BODY (i.e. after the 4-byte header).
# Strings are uint32 length INCLUDING the null terminator, then bytes, then \0.
# Sequences (uint8[]) are uint32 count then raw bytes. Fixed arrays (float32[3])
# carry no length prefix.
#
# These are the exact inverses of the decoders in voice_agent/go2_ros2_client.py
# (cdr_decode_compressed_image_to_pil, cdr_decode_go2state) — that file is the
# authoritative layout reference, and tests/round-trip against it.
# ---------------------------------------------------------------------------

class CdrWriter:
    def __init__(self) -> None:
        self.buf = bytearray()

    def _align(self, size: int) -> None:
        # Relative to the body start, which is why the header is not in self.buf.
        pad = (-len(self.buf)) % size
        self.buf.extend(b"\x00" * pad)

    def u8(self, v: int) -> "CdrWriter":
        self.buf.append(v & 0xFF)
        return self

    def i8(self, v: int) -> "CdrWriter":
        self.buf.extend(struct.pack("<b", max(-128, min(127, int(v)))))
        return self

    def u16(self, v: int) -> "CdrWriter":
        self._align(2); self.buf.extend(struct.pack("<H", int(v) & 0xFFFF)); return self

    def u8_array(self, vals, n: int) -> "CdrWriter":
        """Fixed-size uint8[n] — no length prefix, no alignment."""
        vals = list(vals)[:n] + [0] * max(0, n - len(vals))
        self.buf.extend(bytes(int(v) & 0xFF for v in vals))
        return self

    def i8_array(self, vals, n: int) -> "CdrWriter":
        vals = list(vals)[:n] + [0] * max(0, n - len(vals))
        self.buf.extend(struct.pack(f"<{n}b",
                                    *[max(-128, min(127, int(v))) for v in vals]))
        return self

    def u16_array(self, vals, n: int) -> "CdrWriter":
        vals = list(vals)[:n] + [0] * max(0, n - len(vals))
        self._align(2)
        self.buf.extend(struct.pack(f"<{n}H", *[int(v) & 0xFFFF for v in vals]))
        return self

    def u32_array(self, vals, n: int) -> "CdrWriter":
        vals = list(vals)[:n] + [0] * max(0, n - len(vals))
        self._align(4)
        self.buf.extend(struct.pack(f"<{n}I", *[int(v) & 0xFFFFFFFF for v in vals]))
        return self

    def i16(self, v: int) -> "CdrWriter":
        self._align(2); self.buf.extend(struct.pack("<h", v)); return self

    def i32(self, v: int) -> "CdrWriter":
        self._align(4); self.buf.extend(struct.pack("<i", v)); return self

    def u32(self, v: int) -> "CdrWriter":
        self._align(4); self.buf.extend(struct.pack("<I", v)); return self

    def f32(self, v: float) -> "CdrWriter":
        self._align(4); self.buf.extend(struct.pack("<f", v)); return self

    def f64(self, v: float) -> "CdrWriter":
        self._align(8); self.buf.extend(struct.pack("<d", v)); return self

    def f64_array(self, vals, n: int) -> "CdrWriter":
        vals = list(vals)[:n] + [0.0] * max(0, n - len(vals))
        self._align(8)
        self.buf.extend(struct.pack(f"<{n}d", *[float(v) for v in vals]))
        return self

    def f32_array(self, vals, n: int) -> "CdrWriter":
        vals = list(vals)[:n] + [0.0] * max(0, n - len(vals))
        self._align(4)
        self.buf.extend(struct.pack(f"<{n}f", *[float(v) for v in vals]))
        return self

    def i16_array(self, vals, n: int) -> "CdrWriter":
        vals = list(vals)[:n] + [0] * max(0, n - len(vals))
        self._align(2)
        self.buf.extend(struct.pack(f"<{n}h", *[int(v) for v in vals]))
        return self

    def string(self, s: str) -> "CdrWriter":
        raw = s.encode("utf-8") + b"\x00"
        self.u32(len(raw))
        self.buf.extend(raw)
        return self

    def byte_seq(self, data: bytes) -> "CdrWriter":
        self.u32(len(data))
        self.buf.extend(data)
        return self

    def stamp(self, ns: Optional[int] = None) -> "CdrWriter":
        ns = time.time_ns() if ns is None else ns
        return self.i32(ns // 1_000_000_000).u32(ns % 1_000_000_000)

    def done(self) -> bytes:
        return CDR_HEADER + bytes(self.buf)


def cdr_encode_compressed_image(jpeg: bytes, frame_id: str = "front_camera",
                                fmt: str = "jpeg") -> bytes:
    """sensor_msgs/CompressedImage: Header(stamp, frame_id), string format, uint8[] data."""
    return (CdrWriter()
            .stamp()
            .string(frame_id)
            .string(fmt)
            .byte_seq(jpeg)
            .done())


def cdr_encode_go2state(sport: dict) -> bytes:
    """go2_interfaces/Go2State, in the field order of go2_interfaces/msg/Go2State.msg.

    The whole message is written, not just the prefix the existing decoder reads —
    a short message would make any consumer that reads further (foot_force, the
    foot_* arrays) read past the end.
    """
    return (CdrWriter()
            .u8(int(sport.get("mode") or 0))
            .i32(int(sport.get("progress") or 0))
            .u8(int(sport.get("gait_type") or 0))
            .f32(float(sport.get("foot_raise_height") or 0.0))
            .f32_array(sport.get("position") or [], 3)
            .f32(float(sport.get("body_height") or 0.0))
            .f32_array(sport.get("velocity") or [], 3)
            .f32_array(sport.get("range_obstacle") or [], 4)
            .i16_array(sport.get("foot_force") or [], 4)
            .f32_array(sport.get("foot_position_body") or [], 12)
            .f32_array(sport.get("foot_speed_body") or [], 12)
            .done())


def cdr_encode_low_state(low: dict) -> bytes:
    """go2_interfaces/LowState — the whole message, in .msg field order.

    This one exists for the battery gauge: soc lives at bms_state.soc, and CDR is
    positional, so everything ahead of it (head, sn, version, bandwidth, the IMU,
    and all TWENTY MotorStates) has to be encoded correctly or bms_state lands at
    the wrong offset and the gauge reads garbage. There is no shortcut version.

    Nested layouts come from go2_interfaces/msg/{IMU,MotorState,BmsState}.msg.
    """
    w = CdrWriter()
    w.u8_array(low.get("head") or [], 2)
    w.u8(int(low.get("level_flag") or 0))
    w.u8(int(low.get("frame_reserve") or 0))
    w.u32_array(low.get("sn") or [], 2)
    w.u32_array(low.get("version") or [], 2)
    w.u16(int(low.get("bandwidth") or 0))

    imu = low.get("imu_state") or {}
    w.f32_array(imu.get("quaternion") or [], 4)
    w.f32_array(imu.get("gyroscope") or [], 3)
    w.f32_array(imu.get("accelerometer") or [], 3)
    w.f32_array(imu.get("rpy") or [], 3)
    w.i8(int(imu.get("temperature") or 0))

    motors = low.get("motor_state") or []
    for i in range(20):                      # fixed-size array: always 20
        m = motors[i] if i < len(motors) else {}
        w.u8(int(m.get("mode") or 0))
        w.f32(float(m.get("q") or 0.0))
        w.f32(float(m.get("dq") or 0.0))
        w.f32(float(m.get("ddq") or 0.0))
        w.f32(float(m.get("tau_est") or 0.0))
        w.f32(float(m.get("q_raw") or 0.0))
        w.f32(float(m.get("dq_raw") or 0.0))
        w.f32(float(m.get("ddq_raw") or 0.0))
        w.i8(int(m.get("temperature") or 0))
        w.u32(int(m.get("lost") or 0))
        w.u32_array(m.get("reserve") or [], 2)

    bms = low.get("bms_state") or {}
    w.u8(int(bms.get("version_high") or 0))
    w.u8(int(bms.get("version_low") or 0))
    w.u8(int(bms.get("status") or 0))
    w.u8(int(bms.get("soc") or 0))           # <- the battery gauge
    w.i32(int(bms.get("current") or 0))
    w.u16(int(bms.get("cycle") or 0))
    w.i8_array(bms.get("bq_ntc") or [], 2)
    w.i8_array(bms.get("mcu_ntc") or [], 2)
    w.u16_array(bms.get("cell_vol") or [], 15)

    w.i16_array(low.get("foot_force") or [], 4)
    w.i16_array(low.get("foot_force_est") or [], 4)
    w.u32(int(low.get("tick") or 0))
    w.u8_array(low.get("wireless_remote") or [], 40)
    w.u8(int(low.get("bit_flag") or 0))
    w.f32(float(low.get("adc_reel") or 0.0))
    w.i8(int(low.get("temperature_ntc1") or 0))
    w.i8(int(low.get("temperature_ntc2") or 0))
    w.f32(float(low.get("power_v") or 0.0))
    w.f32(float(low.get("power_a") or 0.0))
    w.u16_array(low.get("fan_frequency") or [], 4)
    w.u32(int(low.get("reserve") or 0))
    w.u32(int(low.get("crc") or 0))
    return w.done()


def voxel_to_xyzi(positions, uvs, res: float, origin, intense_limiter: float = 0.0):
    """Voxel map -> Nx4 (x, y, z, intensity).

    A faithful reimplementation of update_meshes_for_cloud2() from
    go2_robot_sdk/infrastructure/sensors/lidar_decoder.py. It is not imported
    because that module does `from ament_index_python import ...` at import time
    (for the wasm path), which only resolves inside the container — the function
    itself is pure numpy. Keep the two in step if either changes.
    """
    import numpy as np
    pos = np.array(positions).reshape(-1, 3).astype(np.float32)
    pos *= res
    pos += origin
    uv = np.array(uvs, dtype=np.float32).reshape(-1, 2)
    intensities = np.min(uv, axis=1, keepdims=True)
    combined = np.hstack((pos, intensities))
    filtered = combined[combined[:, -1] > intense_limiter]
    return np.unique(filtered, axis=0)


def cdr_encode_odometry(px: float, py: float, pz: float,
                        qx: float, qy: float, qz: float, qw: float) -> bytes:
    """nav_msgs/Odometry, matching the container's _publish_odometry_topic.

    Covariance and twist are left zero, exactly as the container leaves them — the
    robot reports a pose, not an uncertainty. The +0.07 z offset is the container's
    too, so the two agree.

    voice_agent reads only pose.pose.orientation from this (cdr_decode_odom_yaw) to
    close the loop on turn_by_degrees; without /odom that falls back to a timed
    open-loop turn and says so.
    """
    w = CdrWriter()
    w.stamp().string("odom")          # header
    w.string("base_link")             # child_frame_id
    w.f64(px).f64(py).f64(pz + 0.07)  # pose.pose.position
    w.f64(qx).f64(qy).f64(qz).f64(qw)  # pose.pose.orientation
    w.f64_array([], 36)               # pose.covariance
    w.f64(0.0).f64(0.0).f64(0.0)      # twist.twist.linear
    w.f64(0.0).f64(0.0).f64(0.0)      # twist.twist.angular
    w.f64_array([], 36)               # twist.covariance
    return w.done()


# Joint order and the motor indices behind it are copied from the container's
# publish_joint_state. The mapping is NOT identity and not in leg order — FL comes
# from motors 3,4,5 and FR from 0,1,2 — so getting it wrong renders a dog whose
# legs move in the wrong places rather than failing outright.
JOINT_NAMES = [
    'FL_hip_joint', 'FL_thigh_joint', 'FL_calf_joint',
    'FR_hip_joint', 'FR_thigh_joint', 'FR_calf_joint',
    'RL_hip_joint', 'RL_thigh_joint', 'RL_calf_joint',
    'RR_hip_joint', 'RR_thigh_joint', 'RR_calf_joint',
]
JOINT_MOTOR_IDX = [3, 4, 5, 0, 1, 2, 9, 10, 11, 6, 7, 8]


def cdr_encode_joint_state(motors: list) -> bytes:
    """sensor_msgs/JointState: Header, string[] name, float64[] position/velocity/effort.

    velocity and effort are published empty, as the container does — robot_state_publisher
    only needs positions to articulate the URDF.
    """
    w = CdrWriter()
    w.stamp().string("")              # header (the container leaves frame_id empty)
    w.u32(len(JOINT_NAMES))
    for n in JOINT_NAMES:
        w.string(n)
    w.u32(len(JOINT_MOTOR_IDX))
    for i in JOINT_MOTOR_IDX:
        m = motors[i] if i < len(motors) else {}
        w.f64(float(m.get("q") or 0.0))
    w.u32(0)                          # velocity[]
    w.u32(0)                          # effort[]
    return w.done()


def cdr_encode_tf(px: float, py: float, pz: float,
                  qx: float, qy: float, qz: float, qw: float,
                  parent: str = "odom", child: str = "base_link") -> bytes:
    """tf2_msgs/TFMessage carrying one odom->base_link transform.

    Without this the 3D panel has no relation between the cloud's `odom` frame and
    the robot model's `base_link`, so the dog renders at the odom origin while the
    cloud sits around wherever the robot actually is — the "dog is off to the side"
    symptom. The container publishes this from _publish_odom_transform(); this is the
    same transform, including its +0.07 z offset, so the two look identical.

    Note the float64 fields: translation and rotation are 8-byte aligned, unlike
    everything else in this file.
    """
    w = CdrWriter()
    w.u32(1)                       # transforms.length
    w.stamp()
    w.string(parent)               # header.frame_id
    w.string(child)                # child_frame_id
    w.f64(px).f64(py).f64(pz + 0.07)
    w.f64(qx).f64(qy).f64(qz).f64(qw)
    return w.done()


def cdr_encode_pointcloud2(pts, frame_id: str = "odom") -> bytes:
    """sensor_msgs/PointCloud2 with x,y,z,intensity float32 — same 4 fields the
    container's publish_lidar_data uses, so /point_cloud2 looks identical."""
    import numpy as np
    arr = np.asarray(pts, dtype=np.float32).reshape(-1, 4)
    blob = arr.tobytes()
    n = arr.shape[0]
    point_step = 16
    w = CdrWriter()
    w.stamp().string(frame_id)
    w.u32(1)                    # height: unordered cloud
    w.u32(n)                    # width
    w.u32(4)                    # fields.length
    for name, offset in (("x", 0), ("y", 4), ("z", 8), ("intensity", 12)):
        w.string(name)
        w.u32(offset)
        w.u8(7)                 # PointField.FLOAT32
        w.u32(1)                # count
    w.u8(0)                     # is_bigendian
    w.u32(point_step)
    w.u32(point_step * n)       # row_step
    w.byte_seq(blob)
    w.u8(1)                     # is_dense
    return w.done()


# ---------------------------------------------------------------------------
# CDR decoding (inverses of cdr_encode_twist / cdr_encode_webrtc_req)
# ---------------------------------------------------------------------------

class CdrReader:
    def __init__(self, data: bytes) -> None:
        self.data = data
        self.base = 4          # skip the CDR header
        self.p = 4

    def _align(self, size: int) -> None:
        self.p += (-(self.p - self.base)) % size

    def i64(self) -> int:
        self._align(8); v = struct.unpack_from("<q", self.data, self.p)[0]; self.p += 8
        return v

    def u8(self) -> int:
        v = self.data[self.p]; self.p += 1
        return v

    def u32(self) -> int:
        self._align(4); v = struct.unpack_from("<I", self.data, self.p)[0]; self.p += 4
        return v

    def f64(self) -> float:
        self._align(8); v = struct.unpack_from("<d", self.data, self.p)[0]; self.p += 8
        return v

    def string(self) -> str:
        n = self.u32()
        s = self.data[self.p:self.p + n].rstrip(b"\x00").decode("utf-8", errors="replace")
        self.p += n
        return s


def cdr_decode_twist(data: bytes) -> tuple:
    """geometry_msgs/Twist -> (vx, vy, vz, wx, wy, wz). Six float64, no padding."""
    r = CdrReader(data)
    return tuple(r.f64() for _ in range(6))


def cdr_decode_webrtc_req(data: bytes) -> dict:
    """go2_interfaces/WebRtcReq: int64 id, string topic, int64 api_id, string parameter, uint8 priority."""
    r = CdrReader(data)
    out = {"id": r.i64(), "topic": r.string(), "api_id": r.i64(),
           "parameter": r.string()}
    try:
        out["priority"] = r.u8()
    except IndexError:
        out["priority"] = 0
    return out


# ---------------------------------------------------------------------------
# ros2msg schemas (nested types after a separator line, as foxglove_bridge wants)
# ---------------------------------------------------------------------------

SEP = "=" * 80 + "\n"

COMPRESSED_IMAGE_SCHEMA = (
    "std_msgs/Header header\n"
    "string format\n"
    "uint8[] data\n"
    + SEP + "MSG: std_msgs/Header\n"
    "builtin_interfaces/Time stamp\n"
    "string frame_id\n"
    + SEP + "MSG: builtin_interfaces/Time\n"
    "int32 sec\n"
    "uint32 nanosec\n"
)

def _stable_view(cloud: dict, pose, radius: float, res: float,
                 cloud_res: float, max_pts: int):
    """The published cloud: points within `radius` of `pose`, stably decimated.

    This is deliberately NOT "the N nearest points". Nearest-N is unstable: every
    time a new observation lands closer than the current cut-off it DISPLACES a
    point that was already on screen, so as the map densifies the published set
    keeps being re-chosen and the view churns. Measured on a simulated stationary
    robot with realistic re-observation, nearest-N replaced ~41% of published
    points per message.

    A radius crop can only ever ADD points while the robot is still, so what is
    already drawn stays drawn. When the crop is still too big for the wire, it is
    decimated on a hash of the VOXEL KEY rather than on distance or array position:
    a given voxel is always kept or always dropped, so filling in the map elsewhere
    cannot reshuffle what is already visible. (Tuples of ints hash deterministically
    in CPython — only str/bytes hashing is randomised — so this is stable across
    runs, not just within one.)
    """
    import numpy as np
    r2 = radius * radius
    px, py, pz = pose
    # Coarse cells are derived from the FINE VOXEL KEY by integer division, never
    # from the float position. int(x / cloud_res) looked equivalent and was not:
    # it truncates toward zero (so cells straddle the origin asymmetrically) and it
    # sits on a floating-point knife edge — 0.1 * 10 evaluates to 0.999..., which
    # flips a point into the neighbouring cell and makes it jump. Integer floor
    # division has neither problem and is exact.
    ratio = max(1, int(round(cloud_res / max(res, 1e-6))))
    cell_m = ratio * res

    # One point per FIXED coarse cell. The cell size is a constant, so whether a
    # given point is kept depends only on where it is — not on how many points the
    # map currently holds. That is the whole trick: an earlier version derived a
    # stride from len(cloud), and because the stride changed as the map grew, a
    # different set of voxels passed the filter every message. That churned 61% of
    # published points per message, worse than the nearest-N it replaced.
    # Each occupied cell contributes exactly one point AT ITS CENTRE, with the
    # brightest intensity seen in it. The published position is therefore a pure
    # function of the cell, so a cell either appears or does not — a point can never
    # move. That is what finally makes the view stable, and it took three attempts
    # to get there:
    #   - nearest-N            churned ~39%/msg: new near points displace shown ones
    #   - count-derived stride churned ~61%/msg: the stride shifts as the map grows,
    #                          so a different set of voxels passes the filter
    #   - representative point churned ~39%/msg: new fine voxels inside a cell can
    #     picked from the cell    outrank the current representative, so it jumps
    # The cost is honest: positions are quantised to `cloud_res` (10 cm by default),
    # which for a viewer is a normal voxel-grid render and arguably reads cleaner
    # than a jittering exact-position cloud.
    cells: dict = {}
    for fine_key, v in cloud.items():
        key = (fine_key[0] // ratio, fine_key[1] // ratio, fine_key[2] // ratio)
        # Crop on the CELL CENTRE, not the raw stored position. Both are nearly the
        # same distance, but the centre is derived from the integer key, so a cell
        # sitting exactly at the radius cannot flicker in and out when a fresh
        # observation nudges the stored position by a fraction of a voxel.
        cx, cy, cz = (key[0] + 0.5) * cell_m, (key[1] + 0.5) * cell_m, (key[2] + 0.5) * cell_m
        if (cx - px) ** 2 + (cy - py) ** 2 + (cz - pz) ** 2 > r2:
            continue
        prev = cells.get(key)
        if prev is None or v[3] > prev:
            cells[key] = v[3]
    if not cells:
        return np.empty((0, 4), dtype=np.float32)

    out = np.array([((kx + 0.5) * cell_m, (ky + 0.5) * cell_m,
                     (kz + 0.5) * cell_m, inten)
                    for (kx, ky, kz), inten in cells.items()], dtype=np.float32)
    if out.shape[0] > max_pts:
        # Safety valve only. Reaching it means the radius/resolution combination is
        # too generous for the wire budget, and any fix here has to drop points
        # somehow — so say so rather than silently churning the view.
        out = _nearest(out, pose, max_pts)
    return out


def _nearest(arr, pose, n: int):
    """The n points of `arr` closest to `pose`. O(n) via argpartition.

    Selecting by distance is what makes the cloud stable: the same nearby points
    survive every message, so the view stops churning. A full sort would work too
    but this runs on every lidar message with up to ~150k points.
    """
    import numpy as np
    if arr.shape[0] <= n:
        return arr
    d = ((arr[:, 0] - pose[0]) ** 2 + (arr[:, 1] - pose[1]) ** 2
         + (arr[:, 2] - pose[2]) ** 2)
    idx = np.argpartition(d, n - 1)[:n]
    return arr[idx]


POINTCLOUD2_SCHEMA = (
    "std_msgs/Header header\n"
    "uint32 height\n"
    "uint32 width\n"
    "sensor_msgs/PointField[] fields\n"
    "bool is_bigendian\n"
    "uint32 point_step\n"
    "uint32 row_step\n"
    "uint8[] data\n"
    "bool is_dense\n"
    + SEP + "MSG: std_msgs/Header\n"
    "builtin_interfaces/Time stamp\n"
    "string frame_id\n"
    + SEP + "MSG: builtin_interfaces/Time\n"
    "int32 sec\n"
    "uint32 nanosec\n"
    + SEP + "MSG: sensor_msgs/PointField\n"
    "uint8 INT8=1\n"
    "uint8 UINT8=2\n"
    "uint8 INT16=3\n"
    "uint8 UINT16=4\n"
    "uint8 INT32=5\n"
    "uint8 UINT32=6\n"
    "uint8 FLOAT32=7\n"
    "uint8 FLOAT64=8\n"
    "string name\n"
    "uint32 offset\n"
    "uint8 datatype\n"
    "uint32 count\n"
)

ODOM_SCHEMA = (
    "std_msgs/Header header\n"
    "string child_frame_id\n"
    "geometry_msgs/PoseWithCovariance pose\n"
    "geometry_msgs/TwistWithCovariance twist\n"
    + SEP + "MSG: std_msgs/Header\n"
    "builtin_interfaces/Time stamp\nstring frame_id\n"
    + SEP + "MSG: builtin_interfaces/Time\nint32 sec\nuint32 nanosec\n"
    + SEP + "MSG: geometry_msgs/PoseWithCovariance\n"
    "geometry_msgs/Pose pose\nfloat64[36] covariance\n"
    + SEP + "MSG: geometry_msgs/Pose\n"
    "geometry_msgs/Point position\ngeometry_msgs/Quaternion orientation\n"
    + SEP + "MSG: geometry_msgs/Point\nfloat64 x\nfloat64 y\nfloat64 z\n"
    + SEP + "MSG: geometry_msgs/Quaternion\nfloat64 x\nfloat64 y\nfloat64 z\nfloat64 w\n"
    + SEP + "MSG: geometry_msgs/TwistWithCovariance\n"
    "geometry_msgs/Twist twist\nfloat64[36] covariance\n"
    + SEP + "MSG: geometry_msgs/Twist\n"
    "geometry_msgs/Vector3 linear\ngeometry_msgs/Vector3 angular\n"
    + SEP + "MSG: geometry_msgs/Vector3\nfloat64 x\nfloat64 y\nfloat64 z\n"
)

JOINT_STATE_SCHEMA = (
    "std_msgs/Header header\n"
    "string[] name\n"
    "float64[] position\n"
    "float64[] velocity\n"
    "float64[] effort\n"
    + SEP + "MSG: std_msgs/Header\n"
    "builtin_interfaces/Time stamp\nstring frame_id\n"
    + SEP + "MSG: builtin_interfaces/Time\nint32 sec\nuint32 nanosec\n"
)

TF_SCHEMA = (
    "geometry_msgs/TransformStamped[] transforms\n"
    + SEP + "MSG: geometry_msgs/TransformStamped\n"
    "std_msgs/Header header\n"
    "string child_frame_id\n"
    "geometry_msgs/Transform transform\n"
    + SEP + "MSG: std_msgs/Header\n"
    "builtin_interfaces/Time stamp\n"
    "string frame_id\n"
    + SEP + "MSG: builtin_interfaces/Time\n"
    "int32 sec\n"
    "uint32 nanosec\n"
    + SEP + "MSG: geometry_msgs/Transform\n"
    "geometry_msgs/Vector3 translation\n"
    "geometry_msgs/Quaternion rotation\n"
    + SEP + "MSG: geometry_msgs/Vector3\n"
    "float64 x\nfloat64 y\nfloat64 z\n"
    + SEP + "MSG: geometry_msgs/Quaternion\n"
    "float64 x\nfloat64 y\nfloat64 z\nfloat64 w\n"
)

LOW_STATE_SCHEMA = (
    "uint8[2] head\n"
    "uint8 level_flag\n"
    "uint8 frame_reserve\n"
    "uint32[2] sn\n"
    "uint32[2] version\n"
    "uint16 bandwidth\n"
    "IMU imu_state\n"
    "MotorState[20] motor_state\n"
    "BmsState bms_state\n"
    "int16[4] foot_force\n"
    "int16[4] foot_force_est\n"
    "uint32 tick\n"
    "uint8[40] wireless_remote\n"
    "uint8 bit_flag\n"
    "float32 adc_reel\n"
    "int8 temperature_ntc1\n"
    "int8 temperature_ntc2\n"
    "float32 power_v\n"
    "float32 power_a\n"
    "uint16[4] fan_frequency\n"
    "uint32 reserve\n"
    "uint32 crc\n"
    + SEP + "MSG: go2_interfaces/IMU\n"
    "float32[4] quaternion\n"
    "float32[3] gyroscope\n"
    "float32[3] accelerometer\n"
    "float32[3] rpy\n"
    "int8 temperature\n"
    + SEP + "MSG: go2_interfaces/MotorState\n"
    "uint8 mode\n"
    "float32 q\n"
    "float32 dq\n"
    "float32 ddq\n"
    "float32 tau_est\n"
    "float32 q_raw\n"
    "float32 dq_raw\n"
    "float32 ddq_raw\n"
    "int8 temperature\n"
    "uint32 lost\n"
    "uint32[2] reserve\n"
    + SEP + "MSG: go2_interfaces/BmsState\n"
    "uint8 version_high\n"
    "uint8 version_low\n"
    "uint8 status\n"
    "uint8 soc\n"
    "int32 current\n"
    "uint16 cycle\n"
    "int8[2] bq_ntc\n"
    "int8[2] mcu_ntc\n"
    "uint16[15] cell_vol\n"
)

GO2_STATE_SCHEMA = (
    "uint8 mode\n"
    "int32 progress\n"
    "uint8 gait_type\n"
    "float32 foot_raise_height\n"
    "float32[3] position\n"
    "float32 body_height\n"
    "float32[3] velocity\n"
    "float32[4] range_obstacle\n"
    "int16[4] foot_force\n"
    "float32[12] foot_position_body\n"
    "float32[12] foot_speed_body\n"
)


# ---------------------------------------------------------------------------
# Foxglove WebSocket CLIENT
# ---------------------------------------------------------------------------

class FoxgloveClient:
    """Client half of the Foxglove protocol: publish CDR, subscribe to CDR.

    Publishing uses the clientPublish capability, which makes foxglove_bridge
    create genuine ROS publishers — that is how data from here lands in EC2's ROS
    graph where coco_detector and the KVS producer can see it.
    """

    def __init__(self, url: str) -> None:
        self.url = url
        self._ws: Optional[aiohttp.ClientWebSocketResponse] = None
        self._session: Optional[aiohttp.ClientSession] = None
        self._client_channels: Dict[str, int] = {}
        self._next_client_id = 1
        self._server_channels: Dict[str, int] = {}
        self._subs: Dict[int, Callable[[bytes], None]] = {}
        self._next_sub_id = 1
        self._pending_subs: Dict[str, Callable[[bytes], None]] = {}
        self.server_name = "?"
        self.capabilities: list = []
        self.published = 0
        self.received = 0
        self.statuses: list = []
        self._seen_ops: set = set()

    async def connect(self) -> None:
        self._session = aiohttp.ClientSession()
        self._ws = await self._session.ws_connect(
            self.url, protocols=(FOXGLOVE_SUBPROTOCOL,), heartbeat=20,
            max_msg_size=0)
        print(f"[ok] connected to bridge {self.url} "
              f"(subprotocol {self._ws.protocol or 'none negotiated'})")

    async def close(self) -> None:
        with contextlib.suppress(Exception):
            if self._ws:
                await self._ws.close()
        with contextlib.suppress(Exception):
            if self._session:
                await self._session.close()

    def advertise(self, topic: str, schema_name: str, schema: str) -> int:
        cid = self._next_client_id
        self._next_client_id += 1
        self._client_channels[topic] = cid
        self._send_json({
            "op": "advertise",
            "channels": [{
                "id": cid,
                "topic": topic,
                "encoding": "cdr",
                "schemaName": schema_name,
                "schema": schema,
                "schemaEncoding": "ros2msg",
            }],
        })
        return cid

    def want(self, topic: str, cb: Callable[[bytes], None]) -> None:
        """Subscribe as soon as the server advertises `topic` (it may not yet)."""
        self._pending_subs[topic] = cb
        if topic in self._server_channels:
            self._do_subscribe(topic)

    def _do_subscribe(self, topic: str) -> None:
        cb = self._pending_subs.get(topic)
        if cb is None or topic not in self._server_channels:
            return
        sid = self._next_sub_id
        self._next_sub_id += 1
        self._subs[sid] = cb
        self._send_json({"op": "subscribe", "subscriptions": [
            {"id": sid, "channelId": self._server_channels[topic]}]})
        print(f"[ok] subscribed to {topic}")
        self._pending_subs.pop(topic, None)

    def _send_json(self, obj: dict) -> None:
        assert self._ws is not None
        asyncio.ensure_future(self._ws.send_str(json.dumps(obj)))

    def publish(self, topic: str, cdr: bytes) -> None:
        cid = self._client_channels.get(topic)
        if cid is None or self._ws is None:
            return
        # Client message: [opcode 0x01][channelId u32 LE][payload]. Note there is
        # no timestamp on the client->server frame, unlike server->client.
        frame = struct.pack("<BI", 0x01, cid) + cdr
        self.published += 1
        asyncio.ensure_future(self._ws.send_bytes(frame))

    async def run(self) -> None:
        """Read loop. Dispatches server advertisements and message data."""
        assert self._ws is not None
        async for msg in self._ws:
            if msg.type == aiohttp.WSMsgType.TEXT:
                try:
                    payload = json.loads(msg.data)
                except json.JSONDecodeError:
                    continue
                op = payload.get("op")
                if op == "serverInfo":
                    self.server_name = payload.get("name", "?")
                    self.capabilities = payload.get("capabilities", []) or []
                    print(f"[i] bridge: {self.server_name}  "
                          f"capabilities={self.capabilities}")
                    if "clientPublish" not in self.capabilities:
                        print("[!] the bridge does NOT advertise clientPublish — "
                              "nothing this script publishes will reach the ROS "
                              "graph. Check foxglove_bridge's send_buffer_limit / "
                              "capabilities settings.")
                elif op == "advertise":
                    for ch in payload.get("channels", []):
                        self._server_channels[ch["topic"]] = ch["id"]
                        if ch["topic"] in self._pending_subs:
                            self._do_subscribe(ch["topic"])
                elif op == "unadvertise":
                    gone = set(payload.get("channelIds", []))
                    for t, c in list(self._server_channels.items()):
                        if c in gone:
                            del self._server_channels[t]
                elif op == "status":
                    # The bridge reports a REJECTED client advertise here, among
                    # other things. Ignoring this op was a real mistake: a publish
                    # to a channel the bridge refused fails completely silently, so
                    # the symptom is "my topics never appear" with no error anywhere.
                    level = {0: "debug", 1: "info", 2: "warn", 3: "error"}.get(
                        payload.get("level"), payload.get("level"))
                    text = payload.get("message", "")
                    self.statuses.append((level, text))
                    marker = "!!" if level in ("warn", "error") else "i"
                    print(f"[{marker}] bridge status ({level}): {text}")
                else:
                    # Anything else (advertiseServices, parameterValues, ...) is
                    # irrelevant here, but log once so a silent protocol surprise
                    # is visible rather than swallowed.
                    if op not in self._seen_ops:
                        self._seen_ops.add(op)
                        print(f"[i] ignoring bridge op {op!r}")
            elif msg.type == aiohttp.WSMsgType.BINARY:
                data = msg.data
                if not data or data[0] != 0x01 or len(data) < 13:
                    continue
                sub_id = struct.unpack_from("<I", data, 1)[0]
                cb = self._subs.get(sub_id)
                if cb is None:
                    continue
                self.received += 1
                try:
                    cb(data[13:])          # [op][subId u32][ts u64] then payload
                except Exception as e:
                    print(f"[!] subscriber callback error: {type(e).__name__}: {e}")
            elif msg.type in (aiohttp.WSMsgType.CLOSED, aiohttp.WSMsgType.ERROR):
                break
        print("[!] bridge connection closed")


# ---------------------------------------------------------------------------
# The bridge itself
# ---------------------------------------------------------------------------

def encode_jpeg(frame, quality: int) -> bytes:
    buf = io.BytesIO()
    frame.to_image().save(buf, format="JPEG", quality=quality)
    return buf.getvalue()


class Go2ApBridge:
    def __init__(self, args) -> None:
        self.args = args
        self.conn: Optional[UnitreeWebRTCConnection] = None
        self.fox: Optional[FoxgloveClient] = None
        self.sport: dict = {}
        self.low: dict = {}
        self.counts = {"frames": 0, "state": 0, "low": 0, "lidar": 0, "points": 0,
                       "tf": 0, "joints": 0, "cmd_vel": 0, "webrtc_req": 0}
        self._last_frame = 0.0
        self._last_state = 0.0
        self._last_low = 0.0
        self._last_lidar = 0.0
        self._last_tf = 0.0
        self._cloud: dict = {}
        # Robot pose in odom, kept for the nearest-N cloud selection.
        self._pose = (0.0, 0.0, 0.0)
        self._warned_cap = False

    # --- dog -> cloud ------------------------------------------------------

    def _on_sportstate(self, message) -> None:
        self.sport = message.get("data") or {}
        now = time.monotonic()
        if now - self._last_state < 1.0 / max(self.args.state_hz, 0.1):
            return
        self._last_state = now
        self.counts["state"] += 1
        if self.fox:
            self.fox.publish("/go2_states", cdr_encode_go2state(self.sport))

    def _on_lowstate(self, message) -> None:
        self.low = message.get("data") or {}
        now = time.monotonic()
        if now - self._last_low < 1.0 / max(self.args.state_hz, 0.1):
            return
        self._last_low = now
        self.counts["low"] += 1
        if self.fox:
            # Republished specifically because Go2State has NO battery field — soc
            # lives at /lowstate.bms_state.soc, which is what the battery gauge and
            # `make battery` read. An earlier version skipped this on the assumption
            # nothing consumed /lowstate; that assumption was wrong.
            self.fox.publish("/lowstate", cdr_encode_low_state(self.low))
            # Articulates the URDF in the 3D panel. robot_state_publisher turns
            # these 12 positions into the leg transforms; without them the dog
            # renders as a single rigid body.
            motors = self.low.get("motor_state") or []
            if motors:
                self.counts["joints"] += 1
                self.fox.publish("/joint_states", cdr_encode_joint_state(motors))

    def _on_odom(self, message) -> None:
        """rt/utlidar/robot_pose -> /tf (odom->base_link).

        Same source the container uses (ROBOTODOM in robot_data_service), so the
        transform matches what normal mode publishes.
        """
        pose = (message.get("data") or {}).get("pose") or {}
        pos = pose.get("position") or {}
        rot = pose.get("orientation") or {}
        if not pos or not rot:
            return
        now = time.monotonic()
        if now - self._last_tf < 1.0 / max(self.args.tf_hz, 0.1):
            return
        self._last_tf = now
        self.counts["tf"] += 1
        self._pose = (float(pos.get("x") or 0.0), float(pos.get("y") or 0.0),
                      float(pos.get("z") or 0.0))
        x, y, z = (float(pos.get("x") or 0.0), float(pos.get("y") or 0.0),
                   float(pos.get("z") or 0.0))
        ox, oy, oz, ow = (float(rot.get("x") or 0.0), float(rot.get("y") or 0.0),
                          float(rot.get("z") or 0.0), float(rot.get("w") or 1.0))
        if self.fox:
            self.fox.publish("/tf", cdr_encode_tf(x, y, z, ox, oy, oz, ow))
            self.fox.publish("/odom", cdr_encode_odometry(x, y, z, ox, oy, oz, ow))

    def _on_lidar(self, message) -> None:
        data = message.get("data") or {}
        inner = data.get("data") or {}
        # The library's datachannel already ran the voxel decoder for utlidar
        # topics, so `inner` carries positions/uvs; resolution and origin sit on the
        # outer data dict alongside them.
        positions = inner.get("positions")
        uvs = inner.get("uvs")
        if positions is None or uvs is None:
            return
        now = time.monotonic()
        if now - self._last_lidar < 1.0 / max(self.args.lidar_hz, 0.1):
            return
        self._last_lidar = now
        try:
            pts = voxel_to_xyzi(positions, uvs,
                                float(data.get("resolution") or 0.05),
                                data.get("origin") or [0.0, 0.0, 0.0])
        except Exception as e:
            print(f"[!] voxel decode failed: {type(e).__name__}: {e}")
            return
        if pts.shape[0] == 0:
            return
        self.counts["lidar"] += 1

        # Accumulate rather than replace. Each voxel_map_compressed message is a
        # PARTIAL local map, so publishing one message as a whole cloud makes the
        # view flicker and jump — points appear and vanish every frame. The
        # container solves this with pointcloud_aggregator; there is no aggregator
        # in this path, so accumulate here.
        #
        # Keyed on the point quantised to the voxel resolution, which both dedups
        # overlapping sweeps and bounds growth. dicts keep insertion order, so
        # trimming from the front evicts the oldest observations.
        import numpy as np
        if self.args.accumulate:
            res = max(float(data.get("resolution") or 0.05), 1e-3)
            for x, y, z, i in pts:
                self._cloud[(round(float(x) / res), round(float(y) / res),
                             round(float(z) / res))] = (float(x), float(y),
                                                        float(z), float(i))
            arr = np.array(list(self._cloud.values()), dtype=np.float32)

            # Both caps below select by DISTANCE, never by age or array position.
            # That matters more than it looks: an oldest-first eviction (or a
            # `[::step]` downsample) picks a different arbitrary subset every
            # message, so the cloud churns violently even though the underlying
            # geometry is stable. Nearest-N is stable frame to frame and changes
            # smoothly as the robot moves — the same policy PointCloudAggregator
            # uses in lidar_to_pointcloud_node.
            #
            # Distance is measured from the robot's current pose rather than the
            # odom origin (which is where it happened to start), so the detail you
            # keep is the detail around the dog.
            if len(self._cloud) > self.args.max_points:
                arr = _nearest(arr, self._pose, self.args.max_points)
                self._cloud = {k: v for k, v in zip(
                    (tuple(np.round(p[:3] / res).astype(int)) for p in arr),
                    (tuple(float(c) for c in p) for p in arr))}
            # The published cloud is chosen separately from the accumulated map:
            # the map can be large in memory, but every message crosses the
            # internet at 16 bytes/point (the container's own cap is 1e6 points,
            # which would be 16 MB per message here). _stable_view is what keeps
            # the view from churning — see its docstring.
            out = _stable_view(self._cloud, self._pose, self.args.cloud_radius,
                               res, self.args.cloud_res, self.args.publish_points)
            if out.shape[0] >= self.args.publish_points and not self._warned_cap:
                self._warned_cap = True
                print(f"[!] the point cloud is hitting --publish-points "
                      f"({self.args.publish_points}). Raise --cloud-res or lower "
                      f"--cloud-radius, or the view will churn at the cap.")
        else:
            out = pts
            if out.shape[0] > self.args.publish_points:
                out = _nearest(out, self._pose, self.args.publish_points)

        self.counts["points"] = int(out.shape[0])
        if self.fox:
            self.fox.publish("/point_cloud2", cdr_encode_pointcloud2(out))

    async def _on_track(self, track) -> None:
        from aiortc.mediastreams import MediaStreamError
        min_interval = 1.0 / max(self.args.fps, 0.1)
        try:
            while True:
                frame = await track.recv()
                self.counts["frames"] += 1
                if self.counts["frames"] == 1:
                    print(f"[ok] first video frame ({frame.width}x{frame.height})")
                now = time.monotonic()
                if now - self._last_frame < min_interval:
                    continue
                self._last_frame = now
                jpeg = await asyncio.to_thread(encode_jpeg, frame, self.args.quality)
                if self.fox:
                    self.fox.publish("/camera/image_raw/compressed",
                                     cdr_encode_compressed_image(jpeg))
        except MediaStreamError:
            print("[i] video track ended")
        except Exception as e:
            print(f"[!] video loop error: {type(e).__name__}: {e}")

    # --- cloud -> dog ------------------------------------------------------

    def _on_cmd_vel(self, cdr: bytes) -> None:
        vx, vy, _vz, _wx, _wy, wz = cdr_decode_twist(cdr)
        self.counts["cmd_vel"] += 1
        if vx == 0.0 and vy == 0.0 and wz == 0.0:
            # The container's handle_cmd_vel ignores the all-zero Twist too, so a
            # stop is expressed as StopMove via /webrtc_req rather than a zero
            # velocity. Matching that keeps behaviour identical to the container.
            return
        # Exactly the command the container sends — same builder, same rounding.
        cmd = gen_mov_command(round(vx, 2), round(vy, 2), round(wz, 2),
                              self.args.obstacle_avoidance)
        self._send_raw(cmd)

    def _on_webrtc_req(self, cdr: bytes) -> None:
        req = cdr_decode_webrtc_req(cdr)
        self.counts["webrtc_req"] += 1
        topic = req.get("topic") or "rt/api/sport/request"
        param_str = req.get("parameter") or ""
        try:
            parameter = "" if param_str == "" else json.loads(param_str)
        except json.JSONDecodeError:
            print(f"[!] /webrtc_req parameter is not JSON: {param_str!r}")
            return
        payload = {
            "header": {"identity": {"id": int(req.get("id") or 0) or int(time.time() * 1000) % 2147483648,
                                    "api_id": int(req.get("api_id") or 0)}},
            "parameter": parameter,
        }
        self._publish_dc(topic, payload, "msg")
        print(f"[i] forwarded api_id={req.get('api_id')} to {topic}")

    def _send_raw(self, command_json: str) -> None:
        """gen_mov_command returns a ready JSON string for the data channel."""
        ch = self.conn.datachannel.channel if self.conn else None
        if ch is None or ch.readyState != "open":
            return
        try:
            ch.send(command_json)
        except Exception as e:
            print(f"[!] data channel send failed: {e}")

    def _publish_dc(self, topic: str, data, msg_type: str = "msg") -> None:
        ch = self.conn.datachannel.channel if self.conn else None
        if ch is None or ch.readyState != "open":
            return
        try:
            ch.send(json.dumps({"type": msg_type, "topic": topic, "data": data}))
        except Exception as e:
            print(f"[!] data channel send failed: {e}")

    # --- lifecycle ---------------------------------------------------------

    async def connect_dog(self) -> None:
        self.conn = UnitreeWebRTCConnection(
            WebRTCConnectionMethod.LocalAP,
            aes_128_key=self.args.aes_key or None)
        print("[..] connecting to the dog over its AP (no cloud calls)...")
        t0 = time.monotonic()
        await self.conn.connect()
        print(f"[ok] dog link up in {time.monotonic() - t0:.1f}s")
        self.conn.datachannel.pub_sub.subscribe(
            RTC_TOPIC["LF_SPORT_MOD_STATE"], self._on_sportstate)
        self.conn.datachannel.pub_sub.subscribe(
            RTC_TOPIC["LOW_STATE"], self._on_lowstate)
        self.conn.datachannel.pub_sub.subscribe(
            RTC_TOPIC["ROBOTODOM"], self._on_odom)
        if not self.args.no_lidar:
            self.conn.datachannel.pub_sub.subscribe(
                RTC_TOPIC["ULIDAR_ARRAY"], self._on_lidar)
            # The container does this right after connect (webrtc_adapter.connect).
            # Without it the robot stays in traffic-saving mode and the voxel map
            # never arrives, which looks exactly like a decode bug.
            #
            # It lives on the DATACHANNEL, not on UnitreeWebRTCConnection — the
            # method on the SDK's own Go2Connection wrapper is a different thing
            # with the same name.
            #
            # Non-fatal by design: the library's version awaits a reply and then
            # indexes response['info']['execution'], so it can hang or KeyError on
            # an unexpected shape. Losing the point cloud is much better than
            # failing to start the bridge at all.
            try:
                ok = await asyncio.wait_for(
                    self.conn.datachannel.disableTrafficSaving(True), timeout=5.0)
                print(f"[ok] lidar subscribed, traffic saving disabled (ack={ok})")
            except asyncio.TimeoutError:
                print("[!] disableTrafficSaving timed out — continuing. If "
                      "lidar=0 in the heartbeat, that is why.")
            except Exception as e:
                print(f"[!] disableTrafficSaving failed ({type(e).__name__}: {e}) "
                      "— continuing without it; the point cloud may stay empty.")
        if not self.args.no_video:
            self.conn.video.add_track_callback(self._on_track)
            self.conn.video.switchVideoChannel(True)
            print("[..] requested the front camera")

    async def connect_bridge(self) -> None:
        self.fox = FoxgloveClient(self.args.bridge)
        await self.fox.connect()
        self.fox.advertise("/camera/image_raw/compressed",
                           "sensor_msgs/msg/CompressedImage", COMPRESSED_IMAGE_SCHEMA)
        self.fox.advertise("/go2_states", "go2_interfaces/msg/Go2State", GO2_STATE_SCHEMA)
        self.fox.advertise("/lowstate", "go2_interfaces/msg/LowState", LOW_STATE_SCHEMA)
        self.fox.advertise("/tf", "tf2_msgs/msg/TFMessage", TF_SCHEMA)
        self.fox.advertise("/odom", "nav_msgs/msg/Odometry", ODOM_SCHEMA)
        self.fox.advertise("/joint_states", "sensor_msgs/msg/JointState",
                           JOINT_STATE_SCHEMA)
        if not self.args.no_lidar:
            self.fox.advertise("/point_cloud2", "sensor_msgs/msg/PointCloud2",
                               POINTCLOUD2_SCHEMA)
        self.fox.want("/cmd_vel_out", self._on_cmd_vel)
        self.fox.want("/webrtc_req", self._on_webrtc_req)
        print("[ok] advertised: /camera/image_raw/compressed /go2_states "
              "/lowstate /tf /odom /joint_states"
              + ("" if self.args.no_lidar else " /point_cloud2"))
        print("[..] waiting for the bridge to advertise /cmd_vel_out + /webrtc_req")

    async def heartbeat(self) -> None:
        while True:
            await asyncio.sleep(10)
            soc = (self.low.get("bms_state") or {}).get("soc")
            fox = self.fox
            print(f"[i] in: frames={self.counts['frames']} state={self.counts['state']} "
                  f"low={self.counts['low']} tf={self.counts['tf']} joints={self.counts['joints']} lidar={self.counts['lidar']}"
                  f"({self.counts['points']}pts) | "
                  f"out: published={fox.published if fox else 0} | "
                  f"back: cmd_vel={self.counts['cmd_vel']} req={self.counts['webrtc_req']} "
                  f"| soc={soc}%")
            if fox and fox.published == 0:
                print("    [!] nothing has been published upstream yet — check the "
                      "bridge status lines above for a rejected advertise")

    async def run(self) -> int:
        await self.connect_dog()
        await self.connect_bridge()
        if self.args.dry_run:
            print("\n[dry-run] both links are up. Exiting without streaming.")
            return 0
        print("\n[ok] bridging. Ctrl-C to stop.\n")
        hb = asyncio.ensure_future(self.heartbeat())
        try:
            await self.fox.run()
        finally:
            hb.cancel()
        return 0

    async def shutdown(self) -> None:
        if self.fox:
            await self.fox.close()
        if self.conn:
            with contextlib.suppress(Exception):
                await self.conn.disconnect()


def read_docker_env() -> Dict[str, str]:
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)), os.pardir,
                        "docker", ".env")
    out: Dict[str, str] = {}
    try:
        with open(path) as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith("#") or "=" not in line:
                    continue
                k, _, v = line.partition("=")
                out[k.strip()] = v.split("#")[0].strip().strip('"').strip("'")
    except OSError:
        pass
    return out


async def main_async(args) -> int:
    b = Go2ApBridge(args)
    try:
        return await b.run()
    finally:
        await b.shutdown()


def main() -> int:
    env = read_docker_env()
    p = argparse.ArgumentParser(description=__doc__.split("\n")[1])
    p.add_argument("--bridge", default=DEFAULT_BRIDGE,
                   help=f"foxglove_bridge URL (default {DEFAULT_BRIDGE}, what "
                        f"`make tunnel` provides)")
    p.add_argument("--aes-key",
                   default=os.environ.get("AES_128_KEY") or env.get("AES_128_KEY", ""),
                   help="per-device AES-128 key (default: from docker/.env)")
    p.add_argument("--fps", type=float, default=2.0,
                   help="camera frames/s to send upstream (default 2.0, which is "
                        "coco_detector's own throttle)")
    p.add_argument("--quality", type=int, default=70, help="JPEG quality")
    p.add_argument("--state-hz", type=float, default=5.0,
                   help="/go2_states publish rate (default 5)")
    p.add_argument("--no-video", action="store_true")
    p.add_argument("--no-lidar", action="store_true",
                   help="skip the voxel map / point cloud")
    p.add_argument("--tf-hz", type=float, default=10.0,
                   help="/tf publish rate (default 10)")
    p.add_argument("--no-accumulate", dest="accumulate", action="store_false",
                   help="publish each voxel message as-is instead of accumulating "
                        "(each message is a PARTIAL map, so expect flicker)")
    p.add_argument("--lidar-hz", type=float, default=1.0,
                   help="/point_cloud2 publish rate (default 1 — it is a slowly "
                        "changing map, and each message is ~190 KB)")
    p.add_argument("--max-points", type=int, default=150000,
                   help="points to keep in the accumulated map (memory only). The "
                        "container's aggregator keeps 1e6; this is lower only "
                        "because the map is rebuilt each session")
    p.add_argument("--cloud-res", type=float, default=0.10,
                   help="voxel size of the PUBLISHED cloud in metres (default 0.10). "
                        "A fixed cell size is what makes the view stable; the robot's "
                        "own voxel map is finer (~0.05)")
    p.add_argument("--cloud-radius", type=float, default=8.0,
                   help="publish points within this many metres of the dog "
                        "(default 8). A radius crop is stable where a nearest-N "
                        "cap is not — see _stable_view")
    p.add_argument("--publish-points", type=int, default=12000,
                   help="points per published message (default 12000 ~= 190 KB; "
                        "this crosses the internet, so it is the bandwidth knob)")
    p.add_argument("--obstacle-avoidance", action="store_true",
                   help="use the obstacle-avoidance move API (api_id 1003)")
    p.add_argument("--dry-run", action="store_true",
                   help="bring both links up, then exit")
    args = p.parse_args()

    if not args.aes_key:
        print("[!] no AES_128_KEY (firmware >=1.1.15 needs it). Set it in "
              "docker/.env or pass --aes-key.")
        return 2
    try:
        return asyncio.run(main_async(args))
    except KeyboardInterrupt:
        print("\n[i] stopped")
        return 0


if __name__ == "__main__":
    sys.exit(main())
