# Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
# SPDX-License-Identifier: MIT-0
"""Republish raw ROS2 images as compressed JPEG for low-bandwidth transports.

When the SDK container runs remotely (Fargate, or an EC2 dev box) the Foxglove
bridge is reached over an SSM port-forward tunnel, which cannot sustain the
~10 MB/s of raw `sensor_msgs/Image` the camera produces (0.69 MB/frame @ 15fps).
Frames queue and the live feed lags badly.

This node subscribes to the raw image topics and republishes them as
`sensor_msgs/CompressedImage` (JPEG). At quality 50 a frame drops from ~690 KB
to ~24 KB — a ~29x bandwidth reduction — which fits comfortably through the
tunnel while keeping full frame rate. Lichtblick/Foxglove decode JPEG natively.

Subscription QoS is matched per source: the camera driver publishes BEST_EFFORT,
while the COCO detector publishes RELIABLE. A mismatch here silently delivers no
frames, so each input is subscribed with the reliability its publisher uses.

Launched behind the `compressed:=true` flag in robot.launch.py (default on).
"""

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CompressedImage


# Each (source topic, input reliability). Output is always BEST_EFFORT/depth 1,
# which matches how the Foxglove bridge subscribes to image topics.
_SOURCES = [
    ("/camera/image_raw", ReliabilityPolicy.BEST_EFFORT),  # go2_driver_node
    ("/annotated_image", ReliabilityPolicy.RELIABLE),      # coco_detector_node
]

_JPEG_QUALITY = 50


def _qos(reliability: ReliabilityPolicy) -> QoSProfile:
    return QoSProfile(
        reliability=reliability,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )


class ImageRepublisherNode(Node):
    """Subscribes to raw image topics and republishes them as JPEG."""

    def __init__(self):
        super().__init__("image_republisher_node")
        self.declare_parameter("jpeg_quality", _JPEG_QUALITY)
        self._quality = int(
            self.get_parameter("jpeg_quality").get_parameter_value().integer_value
        )
        self._bridge = CvBridge()
        self._pubs = {}

        out_qos = _qos(ReliabilityPolicy.BEST_EFFORT)
        for src, in_reliability in _SOURCES:
            dst = f"{src}/compressed"
            self._pubs[src] = self.create_publisher(CompressedImage, dst, out_qos)
            # Bind src via default arg so the callback knows its source topic.
            self.create_subscription(
                Image,
                src,
                lambda msg, s=src: self._republish(msg, s),
                _qos(in_reliability),
            )
            self.get_logger().info(f"Republishing {src} -> {dst} (JPEG q{self._quality})")

    def _republish(self, msg: Image, src: str) -> None:
        try:
            img = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:  # noqa: BLE001 - log and skip a bad frame
            self.get_logger().warn(f"cv_bridge failed on {src}: {exc}")
            return
        ok, encoded = cv2.imencode(
            ".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), self._quality]
        )
        if not ok:
            return
        out = CompressedImage()
        out.header = msg.header
        out.format = "jpeg"
        out.data = encoded.tobytes()
        self._pubs[src].publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ImageRepublisherNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
