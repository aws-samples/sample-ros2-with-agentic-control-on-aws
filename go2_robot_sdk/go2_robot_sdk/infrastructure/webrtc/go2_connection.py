# Copyright (c) 2024, RoboVerse community
# SPDX-License-Identifier: BSD-3-Clause
# Modified by Amazon.com, Inc. or its affiliates.

"""
Go2 WebRTC connection — thin adapter over `unitree_webrtc_connect`.

Earlier revisions of this module reimplemented the LAN signaling,
con_notify decryption, RSA wrap, AES-128 v3 path, and data-channel
validation by hand. That homegrown stack worked on Go2 firmware
< 1.1.15 but never got the `data2 == 3` LAN handshake right on
firmware ≥ 1.1.15 (data channel stayed in `connecting` even after
ICE completed). The upstream `unitree_webrtc_connect` library has
all of that working.

Public surface (unchanged for the rest of the SDK):
- `Go2Connection.data_channel`  — raw aiortc data channel; the
  WebRTCAdapter sends JSON commands directly via `.send()`.
- `Go2Connection.connect()`     — establishes the connection AND
  waits for validation. Callers can publish immediately after.
- `Go2Connection.disconnect()`  — clean shutdown.
- `Go2Connection.publish(topic, data, msg_type)` — convenience.
- callbacks: `on_validated(robot_num)`, `on_message(msg, parsed, robot_num)`,
  `on_open()`, `on_video_frame(track, robot_num)`.
"""

import asyncio
import json
import logging
import os
from typing import Any, Callable, Dict, Optional, Union

# Import order matters: unitree_webrtc_connect MUST be imported before
# aiortc.mediastreams. Importing mediastreams first poisons aiortc's
# DTLS path — peer connection stalls at "connecting" forever after ICE
# completes. Bisected with a standalone repro script, since removed.
from unitree_webrtc_connect.webrtc_driver import (
    UnitreeWebRTCConnection,
    WebRTCConnectionMethod,
)
from aiortc.mediastreams import MediaStreamError

from .data_decoder import deal_array_buffer as legacy_deal_array_buffer

logger = logging.getLogger(__name__)

# Cached Unitree cloud access token, shared across reconnects.
#
# Remote mode used to re-login on every single reconnect: constructing
# UnitreeWebRTCConnection with username+password makes its __init__ call
# fetch_token(), i.e. a full POST login/email. With the control loop
# retrying every 15s, a robot that stays offline produced ~240 password
# logins/hour from one IP — which gets the account WAF-blocked by
# Unitree's edge (HTTP 567 on *every* request, including unauthenticated
# ones). The token is reusable, so fetch it once and hand it to the
# library via `access_token=`; only re-login when it actually stops working.
_TOKEN_CACHE: Dict[str, str] = {}


class Go2ConnectionError(Exception):
    """Custom exception for Go2 connection errors"""
    pass


class CloudRejectedError(Go2ConnectionError):
    """Unitree's cloud refused the request — WAF block or rate limit.

    Tencent's EdgeOne edge fronts `global-robot-api.unitree.com` and serves
    the block in two shapes: HTTP 567 (which the library turns into
    `HTTPError: HTTP Error 567`) and, once it decides to serve the challenge
    page instead, a 2xx whose body is HTML. The second shape reaches us as a
    bare `json.JSONDecodeError: Expecting value: line 1 column 1 (char 0)`
    from `resp.json()`, which reads like a bug in our code and — worse — got
    classified as a transient failure, so the control loop retried the login
    every 15s and kept the block alive. Both shapes are the same condition,
    so both must land here.
    """
    pass


def invalidate_remote_token() -> None:
    """Drop the cached cloud token so the next connect re-logs-in.

    Called when the cloud rejects the token (expiry) — as opposed to the
    robot simply dropping its data channel, which needs no cloud call.
    """
    _TOKEN_CACHE.clear()


class Go2Connection:
    """WebRTC connection to a Go2 robot, backed by unitree_webrtc_connect."""

    # B107: `token: str = ""` below is an empty default, not a credential.
    # Bandit's hardcoded_password_default check matches the parameter NAME; the
    # real token arrives at run time from the cloud login (see _TOKEN_CACHE).
    def __init__(  # nosec B107
        self,
        robot_ip: str,
        robot_num: int,
        token: str = "",
        on_validated: Optional[Callable] = None,
        on_message: Optional[Callable] = None,
        on_open: Optional[Callable] = None,
        on_video_frame: Optional[Callable] = None,
        decode_lidar: bool = True,
        aes_128_key: str = "",
    ):
        self.robot_ip = robot_ip
        self.robot_num = str(robot_num)
        self.token = token
        self.aes_128_key = aes_128_key
        self.decode_lidar = decode_lidar

        self.on_validated = on_validated
        self.on_message = on_message
        self.on_open = on_open
        self.on_video_frame = on_video_frame

        self._conn: Optional[UnitreeWebRTCConnection] = None
        # `data_channel` is the raw aiortc RTCDataChannel; populated
        # after `connect()` so callers can do `data_channel.send(json)`.
        self.data_channel = None

    @property
    def pc(self):
        """Expose the underlying RTCPeerConnection for state inspection."""
        return self._conn.pc if self._conn else None

    def _build_connection(self) -> UnitreeWebRTCConnection:
        """Construct the UnitreeWebRTCConnection based on CONN_TYPE env var.

        CONN_TYPE=remote  — Unitree TURN server; requires UNITREE_EMAIL,
                            UNITREE_SERIAL, and either UNITREE_PASSWORD or
                            UNITREE_SECRET_NAME (AWS Secrets Manager JSON key).
        CONN_TYPE=local (default) — LAN, using robot IP + AES key. Which LAN
                            flavour is picked by LOCAL_MODE:
                              sta (default) — robot and host share a network
                              ap            — host is joined to the robot's own
                                              hotspot; robot is at 192.168.12.1

        AP mode matters because it is the only path that touches Unitree's
        cloud zero times (no login, no token, no TURN), which is what makes it
        immune to the EdgeOne WAF blocks the remote path keeps hitting. It is
        NOT equivalent to pointing ROBOT_IP at 192.168.12.1: the library also
        sends an empty SDP `id` for LocalAP where LocalSTA sends
        "STA_localNetwork" (see webrtc_driver.get_answer_from_local_peer).
        """
        conn_type = os.environ.get("CONN_TYPE", "local").lower()
        if conn_type == "remote":
            email = os.environ.get("UNITREE_EMAIL", "")
            serial = os.environ.get("UNITREE_SERIAL", "")
            cached = _TOKEN_CACHE.get(email)
            if cached:
                # Reuse the token: build with no credentials (so the library
                # skips fetch_token in __init__) and inject it directly. Only
                # `self.token` is read by connect() for the Remote flow.
                conn = UnitreeWebRTCConnection(
                    WebRTCConnectionMethod.Remote,
                    serialNumber=serial,
                )
                conn.token = cached
                return conn
            password = os.environ.get("UNITREE_PASSWORD") or self._fetch_remote_password()
            conn = UnitreeWebRTCConnection(
                WebRTCConnectionMethod.Remote,
                serialNumber=serial,
                username=email,
                password=password,
            )
            if conn.token:
                _TOKEN_CACHE[email] = conn.token
            return conn
        # UNITREE_LOCAL_MODE is accepted as an alias so docker/.env can hold one
        # spelling that voice_agent/config.py also understands.
        local_mode = (
            os.environ.get("LOCAL_MODE")
            or os.environ.get("UNITREE_LOCAL_MODE")
            or "sta"
        ).lower()
        if local_mode == "ap":
            # The library pins ip=192.168.12.1 for LocalAP itself, so robot_ip
            # is ignored here on purpose.
            logger.info("Connecting to robot %s over its AP (192.168.12.1)",
                        self.robot_num)
            return UnitreeWebRTCConnection(
                WebRTCConnectionMethod.LocalAP,
                aes_128_key=self.aes_128_key or None,
            )
        return UnitreeWebRTCConnection(
            WebRTCConnectionMethod.LocalSTA,
            ip=self.robot_ip,
            aes_128_key=self.aes_128_key or None,
        )

    @staticmethod
    def _fetch_remote_password() -> str:
        """Fetch Unitree password from AWS Secrets Manager.

        Secret is JSON: {"<email>": "<password>"} or {"password": "<password>"}.
        Secret name comes from UNITREE_SECRET_NAME env var.
        """
        secret_name = os.environ.get("UNITREE_SECRET_NAME", "")
        email = os.environ.get("UNITREE_EMAIL", "")
        if not secret_name:
            raise Go2ConnectionError(
                "Remote mode requires UNITREE_PASSWORD or UNITREE_SECRET_NAME."
            )
        import boto3

        from ...aws_solution import boto_config

        client = boto3.client("secretsmanager", config=boto_config())
        response = client.get_secret_value(SecretId=secret_name)
        secret = json.loads(response["SecretString"])
        return secret.get(email) or secret.get("password") or next(iter(secret.values()))

    async def connect(self) -> None:
        """Establish WebRTC connection and wait for validation.

        We retry up to 3 times because the library's internal
        `wait_datachannel_open` is hardcoded to 15s, which can be tight
        when the SDK starts under heavy ROS 2 init load (Nav2, RViz,
        Foxglove all booting in parallel)."""
        from unitree_webrtc_connect import (
            DataChannelTimeoutError,
            NoSdpAnswerError,
            RobotBusyError,
        )

        retryable = (DataChannelTimeoutError, NoSdpAnswerError, RobotBusyError)
        last_error = None
        for attempt in range(1, 6):
            try:
                # Remote mode logs in to the cloud here (inside
                # UnitreeWebRTCConnection.__init__), so a WAF block surfaces
                # from `_build_connection`, before `connect()` is reached.
                self._conn = self._build_connection()
                await self._conn.connect()
                break
            except json.JSONDecodeError as e:
                # Non-JSON body from the cloud: an HTML block/challenge page.
                # Retrying fast is what sustains the block — hand it straight
                # to the caller so it backs off on the cloud path.
                raise CloudRejectedError(
                    f"Unitree cloud returned a non-JSON response for robot "
                    f"{self.robot_num} (EdgeOne WAF block/challenge page, same "
                    f"condition as HTTP 567): {e}"
                ) from e
            except retryable as e:
                last_error = e
                logger.warning(
                    f"Robot {self.robot_num} connect attempt {attempt}/5 "
                    f"failed: {type(e).__name__}: {e}"
                )
                try:
                    await self._conn.disconnect()
                except Exception:
                    pass
                self._conn = None
                if attempt < 5:
                    # Robot needs ~10-15s to release a stale peer.
                    backoff = min(15, 5 * attempt)
                    logger.info(f"Retrying in {backoff}s...")
                    await asyncio.sleep(backoff)
        else:
            raise Go2ConnectionError(
                f"Failed to connect to robot {self.robot_num} after 5 "
                f"attempts: {last_error}"
            )

        self.data_channel = self._conn.datachannel.channel
        self._wire_message_forwarding()
        self._wire_video_track_forwarding()

        # Tell the robot to start the front-camera video stream. Without
        # this, the video transceiver in the SDP completes but the robot
        # never sends frames, so /camera/image_raw stays empty.
        if self.on_video_frame:
            self.publish("", "on", "vid")

        if self.on_open:
            try:
                self.on_open()
            except Exception as e:
                logger.error(f"Error in on_open callback: {e}")

        # Validation already completed inside `unitree_webrtc_connect.connect()`
        # (it awaits `wait_datachannel_open` which only returns after the
        # validation handshake succeeds). Fire the SDK's on_validated now.
        if self.on_validated:
            try:
                self.on_validated(self.robot_num)
            except Exception as e:
                logger.error(f"Error in on_validated callback: {e}")

        logger.info(
            f"Successfully established WebRTC connection to robot {self.robot_num}"
        )

    def _wire_message_forwarding(self) -> None:
        """Hook the data channel's on(message) so we forward to on_message."""
        if not self.on_message:
            return

        channel = self.data_channel
        sdk_callback = self.on_message
        robot_num = self.robot_num
        decode_lidar = self.decode_lidar

        @channel.on("message")
        def _forward(message: Union[str, bytes]) -> None:
            try:
                if isinstance(message, str):
                    try:
                        parsed = json.loads(message)
                    except json.JSONDecodeError:
                        return
                elif isinstance(message, (bytes, bytearray)):
                    parsed = legacy_deal_array_buffer(
                        bytes(message), perform_decode=decode_lidar
                    )
                else:
                    return

                sdk_callback(message, parsed, robot_num)
            except Exception as e:
                logger.error(f"Error forwarding data channel message: {e}")

    def _wire_video_track_forwarding(self) -> None:
        """Forward incoming video tracks to on_video_frame, if requested.

        The upstream library already registers its own `pc.on('track')`
        handler that consumes the track via `track.recv()` in a loop, so
        we can't register a second one — both would race on the same
        track. Instead we plug into the library's `WebRTCVideoChannel`
        callback list, which is invoked once per registered callback
        each time the video track handler advances.
        """
        if not self.on_video_frame:
            return

        on_video_frame = self.on_video_frame
        robot_num = self.robot_num

        async def _track_callback(track):
            try:
                await on_video_frame(track, robot_num)
            except MediaStreamError:
                logger.debug("Video track ended")
            except Exception as e:
                logger.error(f"Error in video frame callback: {e}")

        self._conn.video.add_track_callback(_track_callback)

    def publish(self, topic: str, data: Any, msg_type: str = "msg") -> None:
        """Send a JSON message over the data channel."""
        if not self.data_channel or self.data_channel.readyState != "open":
            logger.warning(
                f"Data channel is not open. State is "
                f"{self.data_channel.readyState if self.data_channel else 'None'}"
            )
            return

        payload_str = json.dumps({"type": msg_type, "topic": topic, "data": data})
        try:
            self.data_channel.send(payload_str)
        except Exception as e:
            logger.error(f"Failed to publish message: {e}")

    async def request_keyframe(self) -> bool:
        """Ask the robot to emit an H.264 keyframe (IDR) via an RTCP PLI.

        Over the remote/TURN path, packet loss and reconnects leave the aiortc
        decoder stuck mid-GOP ("failed to decode" until the next keyframe). The
        robot only sends keyframes occasionally, so the feed can stay black for
        a long time. Sending a Picture Loss Indication nudges the encoder to
        emit an IDR immediately so the decoder can lock on. Best-effort.
        """
        try:
            pc = self.pc
            if pc is None:
                return False
            for receiver in pc.getReceivers():
                track = getattr(receiver, "track", None)
                if track is not None and track.kind == "video":
                    ssrcs = receiver.getSynchronizationSources()
                    if not ssrcs:
                        continue
                    await receiver._send_rtcp_pli(ssrcs[0].source)
                    return True
            return False
        except Exception as e:
            logger.debug(f"request_keyframe failed: {e}")
            return False

    async def disableTrafficSaving(self, switch: bool) -> bool:
        """Disable / enable robot's traffic-saving mode."""
        try:
            self.publish(
                "",
                {
                    "req_type": "disable_traffic_saving",
                    "instruction": "on" if switch else "off",
                },
                "rtc_inner_req",
            )
            return True
        except Exception as e:
            logger.error(f"Failed to set traffic saving: {e}")
            return False

    async def disconnect(self) -> None:
        """Close the connection and free resources."""
        if self._conn:
            try:
                await self._conn.disconnect()
            except Exception as e:
                logger.error(f"Error disconnecting: {e}")
            finally:
                self._conn = None
                self.data_channel = None
        logger.info(f"Disconnected from robot {self.robot_num}")
