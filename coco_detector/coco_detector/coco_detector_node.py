# Copyright (c) 2024 Julian Francis
# SPDX-License-Identifier: MIT
# Modified by Amazon.com, Inc. or its affiliates.
"""Detects COCO objects in image and publishes in ROS2.

Subscribes to /image and publishes Detection2DArray message on topic /detected_objects.
Also publishes (by default) annotated image with bounding boxes on /annotated_image.
Uses PyTorch detection models from torchvision (see MODELS below).
Bounding Boxes use image convention, ie center.y = 0 means top of image.
"""

import collections
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CompressedImage
from vision_msgs.msg import BoundingBox2D, ObjectHypothesis, ObjectHypothesisWithPose
from vision_msgs.msg import Detection2D, Detection2DArray
from cv_bridge import CvBridge
import torch
from torchvision.models import detection as detection_model
from torchvision.utils import draw_bounding_boxes

BEST_EFFORT_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

Detection = collections.namedtuple("Detection", "label, bbox, score")

# Selectable detector backbones, all from torchvision (already a container
# dependency — no new packages, no separate inference service).
#
# `resize` is the model's internal shorter-side resize. Faster R-CNN letterboxes
# internally, so lowering it buys speed at some cost in small-object recall; the
# published mAP figures below are at the model's default resize.
#
# Measured on one CPU thread with a 720x1280 frame (matches the 1-vCPU Fargate
# task), plus recall on a handful of COCO val + indoor scenes:
#
#   key         model                       mAP   1-thread   notes
#   ---------------------------------------------------------------------------
#   fast        frcnn_mobilenet_320         22.8    ~60 ms   old default; misses
#                                                            most non-person
#                                                            classes, called a
#                                                            dog a "cat"
#   balanced    frcnn_resnet50_fpn @480     37.0   ~380 ms   DEFAULT. ~2.6 fps,
#                                                            comfortably above
#                                                            the 2 fps throttle
#   accurate    frcnn_resnet50_fpn @800     37.0   ~890 ms   best small-object
#                                                            recall; ~1.1 fps
#   best        frcnn_resnet50_fpn_v2       46.7  ~3900 ms   highest mAP but
#                                                            head-bound: ~0.25
#                                                            fps even at a
#                                                            reduced resize, so
#                                                            it is too slow for
#                                                            the greeter/tracking
#                                                            loops. Offline use
#                                                            or a GPU only.
MODELS = {
    "fast": ("fasterrcnn_mobilenet_v3_large_320_fpn",
             "FasterRCNN_MobileNet_V3_Large_320_FPN_Weights", None),
    "balanced": ("fasterrcnn_resnet50_fpn", "FasterRCNN_ResNet50_FPN_Weights", 480),
    "accurate": ("fasterrcnn_resnet50_fpn", "FasterRCNN_ResNet50_FPN_Weights", 800),
    "best": ("fasterrcnn_resnet50_fpn_v2", "FasterRCNN_ResNet50_FPN_V2_Weights", None),
}

class CocoDetectorNode(Node):
    """Detects COCO objects in image and publishes on ROS2.

    Subscribes to /image and publishes Detection2DArray on /detected_objects.
    Also publishes augmented image with bounding boxes on /annotated_image.
    """

    # pylint: disable=R0902 disable too many instance variables warning for this class
    def __init__(self):
        super().__init__("coco_detector_node")
        self.declare_parameter('device', 'cpu')
        # 0.9 was far too strict: it is a *post-NMS* class confidence, and the
        # stronger backbones spread probability mass over more true positives
        # rather than saturating at 1.0. On indoor scenes 0.9 discarded real
        # detections the model had already found — people at 0.82, a cup at 0.86
        # — which is why the detector appeared to only ever see people and
        # chairs. 0.5 is torchvision's own reporting threshold.
        self.declare_parameter('detection_threshold', 0.5)
        self.declare_parameter('publish_annotated_image', True)
        # Which backbone to load; see MODELS above for the speed/accuracy table.
        self.declare_parameter('model', 'balanced')
        # Faster R-CNN on CPU runs ~1-3 fps, but the camera streams ~15 fps.
        # Without throttling, inference can't keep up and frames queue, so
        # detections (and the annotated image) lag further and further behind
        # the live feed. Process at most this many frames per second and drop
        # the rest — detection at ~2 fps is plenty for "what do you see".
        self.declare_parameter('max_detection_fps', 2.0)
        # Which topic to take frames from, and in which form.
        #
        # The default is unchanged from before this parameter existed: raw
        # sensor_msgs/Image on /camera/image_raw, which is what the driver publishes
        # when the container owns the robot link. Nothing about the normal EC2 or
        # local path is affected by adding this.
        #
        # It exists for the inverted AP-mode setup (scripts/go2_ap_bridge.py), where
        # a laptop holds the robot link and pushes frames to this ROS graph over the
        # internet. Raw 720p is ~1.3 MB/frame and not viable over that link, so the
        # bridge publishes CompressedImage instead. Point this at the compressed
        # topic rather than adding a republisher hop:
        #   -p image_topic:=/camera/image_raw/compressed -p image_compressed:=true
        self.declare_parameter('image_topic', '/camera/image_raw')
        # Left as an explicit parameter rather than inferred from the topic name so
        # a topic that happens to end in /compressed can't silently change how it is
        # decoded. Defaults false = the original raw Image path.
        self.declare_parameter('image_compressed', False)
        self.device = self.get_parameter('device').get_parameter_value().string_value
        self.detection_threshold = \
            self.get_parameter('detection_threshold').get_parameter_value().double_value
        max_fps = self.get_parameter('max_detection_fps').get_parameter_value().double_value
        self._min_interval_ns = int(1e9 / max_fps) if max_fps > 0 else 0
        self._last_processed_ns = 0
        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self._compressed = \
            self.get_parameter('image_compressed').get_parameter_value().bool_value
        self.subscription = self.create_subscription(
            CompressedImage if self._compressed else Image,
            image_topic,
            self.listener_callback,
            BEST_EFFORT_QOS)
        self.detected_objects_publisher = \
            self.create_publisher(Detection2DArray, "detected_objects", 10)
        if self.get_parameter('publish_annotated_image').get_parameter_value().bool_value:
            self.annotated_image_publisher = \
                self.create_publisher(Image, "annotated_image", 10)
        else:
            self.annotated_image_publisher = None
        self.bridge = CvBridge()
        model_key = self.get_parameter('model').get_parameter_value().string_value
        if model_key not in MODELS:
            self.get_logger().warn(
                f"Unknown model '{model_key}'; expected one of "
                f"{sorted(MODELS)}. Falling back to 'balanced'.")
            model_key = 'balanced'
        builder_name, weights_name, resize = MODELS[model_key]
        weights = getattr(detection_model, weights_name).DEFAULT
        # Only pass min_size/max_size when overriding, so each model keeps its
        # own trained default otherwise. The 1:1.67 ratio matches torchvision's
        # stock 800/1333 and covers the Go2's 16:9 frame without extra padding.
        kwargs = {} if resize is None else {
            "min_size": resize, "max_size": int(resize * 1.67)}
        self.model = getattr(detection_model, builder_name)(
            weights=weights, progress=True, **kwargs).to(self.device)
        self.class_labels = weights.meta["categories"]
        self.model.eval()
        self.get_logger().info(
            f"Node has started. model={model_key} ({builder_name}"
            f"{f', resize={resize}' if resize else ''}) "
            f"device={self.device} threshold={self.detection_threshold} "
            f"input={image_topic}"
            f"{' (compressed)' if self._compressed else ' (raw)'}")

    def mobilenet_to_ros2(self, detection, header):
        """Converts a Detection tuple(label, bbox, score) to a ROS2 Detection2D message."""

        detection2d = Detection2D()
        detection2d.header = header
        object_hypothesis_with_pose = ObjectHypothesisWithPose()
        object_hypothesis = ObjectHypothesis()
        object_hypothesis.class_id = self.class_labels[detection.label]
        object_hypothesis.score = detection.score.detach().item()
        object_hypothesis_with_pose.hypothesis = object_hypothesis
        detection2d.results.append(object_hypothesis_with_pose)
        bounding_box = BoundingBox2D()
        bounding_box.center.position.x = float((detection.bbox[0] + detection.bbox[2]) / 2)
        bounding_box.center.position.y = float((detection.bbox[1] + detection.bbox[3]) / 2)
        bounding_box.center.theta = 0.0
        bounding_box.size_x = float(2 * (bounding_box.center.position.x - detection.bbox[0]))
        bounding_box.size_y = float(2 * (bounding_box.center.position.y - detection.bbox[1]))
        detection2d.bbox = bounding_box
        return detection2d

    def publish_annotated_image(self, filtered_detections, header, image):
        """Draws the bounding boxes on the image and publishes to /annotated_image"""

        if len(filtered_detections) > 0:
            pred_boxes = torch.stack([detection.bbox for detection in filtered_detections])
            # Include the score so the threshold can be tuned by eye from the
            # annotated stream (Foxglove / KVS) without reading topic dumps.
            pred_labels = [
                f"{self.class_labels[detection.label]} {float(detection.score):.2f}"
                for detection in filtered_detections]
            annotated_image = draw_bounding_boxes(torch.tensor(image), pred_boxes,
                                                  pred_labels, colors="yellow")
        else:
            annotated_image = torch.tensor(image)
        ros2_image_msg = self.bridge.cv2_to_imgmsg(annotated_image.numpy().transpose(1, 2, 0),
                                                   encoding="rgb8")
        ros2_image_msg.header = header
        self.annotated_image_publisher.publish(ros2_image_msg)

    def listener_callback(self, msg):
        """Reads image and publishes on /detected_objects and /annotated_image."""
        # Throttle: skip this frame if we processed one too recently. Keeps
        # detections close to live instead of letting a backlog accumulate.
        if self._min_interval_ns:
            now_ns = self.get_clock().now().nanoseconds
            if now_ns - self._last_processed_ns < self._min_interval_ns:
                return
            self._last_processed_ns = now_ns
        if self._compressed:
            # compressed_imgmsg_to_cv2 gives BGR (JPEG decodes through OpenCV), so
            # the channel order has to be flipped to match the rgb8 the raw path
            # requests — otherwise the model sees blue and red swapped and quietly
            # gets worse rather than failing.
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")
            cv_image = cv_image[:, :, ::-1]
        else:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
        image = cv_image.copy().transpose((2, 0, 1))
        batch_image = np.expand_dims(image, axis=0)
        tensor_image = torch.tensor(batch_image/255.0, dtype=torch.float, device=self.device)
        mobilenet_detections = self.model(tensor_image)[0]  # pylint: disable=E1102 disable not callable warning
        filtered_detections = [Detection(label_id, box, score) for label_id, box, score in
            zip(mobilenet_detections["labels"],
            mobilenet_detections["boxes"],
            mobilenet_detections["scores"]) if score >= self.detection_threshold]
        detection_array = Detection2DArray()
        detection_array.header = msg.header
        detection_array.detections = \
            [self.mobilenet_to_ros2(detection, msg.header) for detection in filtered_detections]
        self.detected_objects_publisher.publish(detection_array)
        if self.annotated_image_publisher is not None:
            self.publish_annotated_image(filtered_detections, msg.header, image)


rclpy.init()
coco_detector_node = CocoDetectorNode()
rclpy.spin(coco_detector_node)
coco_detector_node.destroy_node()
rclpy.shutdown()
