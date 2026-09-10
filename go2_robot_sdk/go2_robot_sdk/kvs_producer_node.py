# Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
# SPDX-License-Identifier: MIT-0
"""ROS2 node: streams /camera/image_raw to AWS Kinesis Video Streams.

Subscribes to the camera topic, buffers frames, encodes to H.264 MKV via
ffmpeg, and uploads to KVS using PutMedia with SigV4 auth. Runs in a
background thread so it doesn't block the ROS2 executor.

Required env vars:
  KVS_STREAM_NAME  — KVS stream name (default: go2-robot-01-camera)
  AWS_DEFAULT_REGION — AWS region (default: us-east-1)

Credentials come from the standard AWS credential chain, which boto3 resolves
itself — this node never reads a credential variable. On the deployed path that
chain resolves to the EC2 INSTANCE PROFILE (Go2Ec2Stack's RobotRole, which holds
exactly kinesisvideo:PutMedia on this robot's stream and nothing else), so there
is no key to distribute, expire or rotate. That is the pattern to copy. For a
local run see the credentials section of docker/.env.example — short-lived SSO
credentials or a mounted profile, not static keys.

Optional:
  KVS_IMAGE_TOPIC  — ROS2 image topic to stream (default: /camera/image_raw).
                     Set to /annotated_image to stream the COCO overlay feed.
  KVS_FPS          — target frame rate (default: 15)
  KVS_WIDTH        — output width (default: 1280)
  KVS_HEIGHT       — output height (default: 720)
  KVS_SEGMENT_SEC  — seconds per upload chunk (default: 10)
"""

import datetime
import json
import os
import subprocess
import tempfile
import threading
import time

import boto3
import cv2
import numpy as np
import rclpy
import requests
from botocore.auth import SigV4Auth
from botocore.awsrequest import AWSRequest
from botocore.session import Session
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image

from .aws_solution import USER_AGENT_STRING, boto_config


class KvsProducerNode(Node):
    def __init__(self):
        super().__init__("kvs_producer_node")

        self._stream_name = os.environ.get("KVS_STREAM_NAME", "go2-robot-01-camera")
        self._image_topic = os.environ.get("KVS_IMAGE_TOPIC", "/camera/image_raw")
        self._region = os.environ.get("AWS_DEFAULT_REGION", "us-east-1")
        self._fps = int(os.environ.get("KVS_FPS", "15"))
        self._width = int(os.environ.get("KVS_WIDTH", "1280"))
        self._height = int(os.environ.get("KVS_HEIGHT", "720"))
        self._segment_sec = int(os.environ.get("KVS_SEGMENT_SEC", "10"))

        self._bridge = CvBridge()
        self._lock = threading.Lock()
        self._frames: list = []
        self._latest_frame = None
        self._running = True
        self._endpoint = None

        # Subscription reliability must match the source publisher or ROS2
        # silently delivers no frames. go2_driver_node publishes
        # /camera/image_raw BEST_EFFORT; coco_detector publishes
        # /annotated_image RELIABLE — so default reliability to RELIABLE when
        # streaming the annotated topic. Overridable via KVS_QOS_RELIABLE.
        qos_env = os.environ.get("KVS_QOS_RELIABLE")
        if qos_env is not None:
            reliable = qos_env.lower() in ("1", "true", "yes")
        else:
            reliable = "annotated" in self._image_topic
        qos = QoSProfile(
            reliability=(
                QoSReliabilityPolicy.RELIABLE
                if reliable
                else QoSReliabilityPolicy.BEST_EFFORT
            ),
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.subscription = self.create_subscription(
            Image, self._image_topic, self._on_image, qos
        )

        self._sample_thread = threading.Thread(target=self._sample_loop, daemon=True)
        self._sample_thread.start()

        self._upload_thread = threading.Thread(target=self._upload_loop, daemon=True)
        self._upload_thread.start()

        self.get_logger().info(
            f"KVS Producer started → {self._stream_name} "
            f"(topic {self._image_topic}, "
            f"{self._width}x{self._height}@{self._fps}fps, "
            f"{self._segment_sec}s segments)"
        )

    def _on_image(self, msg: Image):
        # Keep only the latest frame. A steady-rate sampler (_sample_loop)
        # feeds the encoder, so a slow/bursty source topic (e.g. the ~2fps
        # /annotated_image feed) still produces continuous, gap-free video.
        # Appending raw incoming frames instead would yield short segments
        # with empty timeline, which starves the HLS player and disconnects it.
        frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        frame = cv2.resize(frame, (self._width, self._height))
        with self._lock:
            self._latest_frame = frame

    def _sample_loop(self):
        # Append the most recent frame at a fixed cadence so each segment holds
        # exactly segment_sec * fps frames of real wall-clock time.
        interval = 1.0 / self._fps
        while self._running:
            time.sleep(interval)
            with self._lock:
                frame = self._latest_frame
                if frame is None:
                    continue
                self._frames.append(frame)

    def _get_endpoint(self) -> str:
        if self._endpoint is None:
            client = boto3.client(
                "kinesisvideo", region_name=self._region, config=boto_config()
            )
            resp = client.get_data_endpoint(
                StreamName=self._stream_name, APIName="PUT_MEDIA"
            )
            self._endpoint = resp["DataEndpoint"]
        return self._endpoint

    def _upload_loop(self):
        time.sleep(3)

        try:
            endpoint = self._get_endpoint()
        except Exception as e:
            self.get_logger().error(f"Failed to get KVS endpoint: {e}")
            return

        self.get_logger().info("KVS upload loop running")

        while self._running:
            time.sleep(self._segment_sec)

            with self._lock:
                batch = self._frames.copy()
                self._frames.clear()

            if not batch:
                continue

            # mkstemp, not mktemp: it creates the file atomically with 0600, so
            # nothing can slip a symlink in between naming and writing. ffmpeg
            # is called with -y, so the empty placeholder is fine.
            fd, mkv_path = tempfile.mkstemp(suffix=".mkv")
            os.close(fd)
            try:
                self._encode_frames(batch, mkv_path)
                if os.path.exists(mkv_path) and os.path.getsize(mkv_path) > 0:
                    self._upload_mkv(mkv_path, endpoint)
            except Exception as e:
                self.get_logger().warn(f"Upload segment failed: {e}")
            finally:
                try:
                    os.unlink(mkv_path)
                except OSError:
                    pass

    def _encode_frames(self, frames: list, output_path: str):
        # The argv list is written out here rather than built into a variable first,
        # so the program name stays visibly a literal. argv-list form with no
        # shell=True: no shell is spawned, so the non-constant elements (geometry,
        # fps, and the tempfile-generated output_path) cannot be reinterpreted as
        # shell syntax.
        proc = subprocess.Popen(
            [
                "ffmpeg", "-y", "-loglevel", "error",
                "-f", "rawvideo", "-pix_fmt", "bgr24",
                "-s", f"{self._width}x{self._height}",
                "-r", str(self._fps),
                "-i", "pipe:0",
                "-pix_fmt", "yuv420p",
                "-c:v", "libx264", "-preset", "veryfast",
                "-tune", "zerolatency", "-profile:v", "baseline",
                "-g", str(self._fps), "-bf", "0", "-b:v", "2000k",
                "-f", "matroska", "-cluster_time_limit", "500",
                output_path,
            ],
            stdin=subprocess.PIPE, stderr=subprocess.PIPE,
        )
        for frame in frames:
            proc.stdin.write(frame.tobytes())
        proc.stdin.close()
        proc.wait()

    def _upload_mkv(self, mkv_path: str, endpoint: str):
        url = f"{endpoint}/putMedia"
        session = Session()
        credentials = session.get_credentials().get_frozen_credentials()
        now = datetime.datetime.now(datetime.timezone.utc)

        headers = {
            "x-amzn-stream-name": self._stream_name,
            "x-amzn-fragment-timecode-type": "RELATIVE",
            "x-amzn-producer-start-timestamp": now.strftime("%Y-%m-%dT%H:%M:%S.000Z"),
            "Content-Type": "application/json",
        }
        request = AWSRequest(method="POST", url=url, headers=headers, data=b"")
        SigV4Auth(credentials, "kinesisvideo", self._region).add_auth(request)

        signed_headers = dict(request.headers)
        # PutMedia is the one AWS call in this package that does not go through
        # boto3 (it is a streaming POST to the KVS data endpoint), so the solution
        # string has to be put on the header by hand. Appended to requests' own
        # user agent, and set AFTER signing on purpose: botocore excludes
        # user-agent from SigV4 (SIGNED_HEADERS_BLACKLIST), so adding it here
        # keeps it out of SignedHeaders and cannot invalidate the signature.
        signed_headers["User-Agent"] = (
            f"{requests.utils.default_user_agent()} {USER_AGENT_STRING}"
        )
        signed_headers["x-amzn-stream-name"] = self._stream_name
        signed_headers["x-amzn-fragment-timecode-type"] = "RELATIVE"
        signed_headers["x-amzn-producer-start-timestamp"] = now.strftime("%Y-%m-%dT%H:%M:%S.000Z")
        signed_headers["Transfer-Encoding"] = "chunked"

        def chunks():
            with open(mkv_path, "rb") as f:
                while chunk := f.read(16384):
                    yield chunk

        resp = requests.post(
            url, headers=signed_headers, data=chunks(), stream=True, timeout=30
        )

        persisted = 0
        for line in resp.iter_lines():
            if line:
                try:
                    ack = json.loads(line)
                    if ack.get("EventType") == "PERSISTED":
                        persisted += 1
                except json.JSONDecodeError:
                    pass

        if persisted > 0:
            self.get_logger().info(f"Uploaded {persisted} fragments to KVS")

    def destroy_node(self):
        self._running = False
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = KvsProducerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
