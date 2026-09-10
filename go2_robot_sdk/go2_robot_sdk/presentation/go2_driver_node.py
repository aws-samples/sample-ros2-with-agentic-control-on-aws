# Copyright (c) 2024, RoboVerse community
# SPDX-License-Identifier: BSD-3-Clause
# Modified by Amazon.com, Inc. or its affiliates.

import asyncio
import json
import logging
import os
import re
from typing import Dict, Any

# Do NOT eagerly import `aiortc.MediaStreamTrack` at module load.
# Importing aiortc.mediastreams BEFORE unitree_webrtc_connect poisons
# aiortc's DTLS path — peer connection stalls at "connecting" forever.
# We use the type only for annotation; importing it lazily inside the
# method below is safe.
from cv_bridge import CvBridge

from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.qos_overriding_options import QoSOverridingOptions
from rcl_interfaces.msg import SetParametersResult
from tf2_ros import TransformBroadcaster

from geometry_msgs.msg import Twist, PoseStamped
from go2_interfaces.msg import Go2State, IMU
from go2_interfaces.msg import LowState, VoxelMapCompressed, WebRtcReq
from sensor_msgs.msg import PointCloud2, JointState, Joy, Image, CameraInfo
from nav_msgs.msg import Odometry

from ..domain.entities import RobotConfig, RobotData, CameraData
from ..application.services import RobotDataService, RobotControlService
from ..infrastructure.ros2.ros2_publisher import ROS2Publisher
from ..infrastructure.webrtc.webrtc_adapter import WebRTCAdapter
from ..infrastructure.webrtc.go2_connection import (
    CloudRejectedError,
    invalidate_remote_token,
)

logging.basicConfig(level=logging.WARN)
logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)


class Go2DriverNode(Node):
    """Main Go2 driver node - entry point to the application"""

    def __init__(self, event_loop=None):
        super().__init__('go2_driver_node')  # Clean architecture main driver
        self.event_loop = event_loop
        
        # Configuration initialization
        self.config = self._setup_configuration()
        
        # Infrastructure initialization
        self.publishers_dict = self._setup_publishers()
        self.broadcaster = TransformBroadcaster(self, qos=QoSProfile(depth=10))
        self.bridge = CvBridge()
        
        # Architecture layers initialization
        self.ros2_publisher = ROS2Publisher(
            node=self,
            config=self.config,
            publishers=self.publishers_dict,
            broadcaster=self.broadcaster
        )
        
        self.robot_data_service = RobotDataService(self.ros2_publisher)
        
        self.webrtc_adapter = WebRTCAdapter(
            config=self.config,
            on_validated_callback=self._on_robot_validated,
            on_video_frame_callback=self._on_video_frame if self.config.enable_video else None,
            event_loop=self.event_loop
        )
        
        self.robot_control_service = RobotControlService(self.webrtc_adapter)
        
        # Set callback for data
        self.webrtc_adapter.set_data_callback(self._on_robot_data_received)
        
        # Subscribers initialization
        self._setup_subscribers()
        
        # State
        self.joy_state = Joy()

    def _setup_configuration(self) -> RobotConfig:
        """Configuration setup"""
        robot_ip = os.getenv('ROBOT_IP', os.getenv('GO2_IP', ''))
        token = os.getenv('ROBOT_TOKEN', os.getenv('GO2_TOKEN', ''))
        conn_type = os.getenv('CONN_TYPE', '')
        aes_128_key = os.getenv('AES_128_KEY', '')

        # Declare parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('robot_ip', robot_ip),
                ('token', token),
                ('conn_type', conn_type),
                ('aes_128_key', aes_128_key),
                ('enable_video', True),
                ('decode_lidar', True),
                ('publish_raw_voxel', False),
                ('obstacle_avoidance', False),
            ]
        )

        self.add_on_set_parameters_callback(self._on_set_parameters)

        # Get parameter values
        config = RobotConfig.from_params(
            robot_ip=self.get_parameter('robot_ip').get_parameter_value().string_value,
            token=self.get_parameter('token').get_parameter_value().string_value,
            conn_type=self.get_parameter('conn_type').get_parameter_value().string_value,
            enable_video=self.get_parameter('enable_video').get_parameter_value().bool_value,
            decode_lidar=self.get_parameter('decode_lidar').get_parameter_value().bool_value,
            publish_raw_voxel=self.get_parameter('publish_raw_voxel').get_parameter_value().bool_value,
            obstacle_avoidance=self.get_parameter('obstacle_avoidance').get_parameter_value().bool_value,
            aes_128_key=self.get_parameter('aes_128_key').get_parameter_value().string_value,
        )

        # Log configuration
        self.get_logger().info(f"Robot IPs: {config.robot_ip_list}")
        self.get_logger().info(f"Connection type: {config.conn_type}")
        self.get_logger().info(f"Connection mode: {config.conn_mode}")
        self.get_logger().info(f"Enable video: {config.enable_video}")
        self.get_logger().info(f"Decode lidar: {config.decode_lidar}")
        self.get_logger().info(f"Publish raw voxel: {config.publish_raw_voxel}")
        self.get_logger().info(f"Obstacle avoidance: {config.obstacle_avoidance}")

        return config

    def _setup_publishers(self) -> Dict[str, list]:
        """ROS2 publishers setup"""
        qos_profile = QoSProfile(depth=10)
        best_effort_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        publishers = {
            'joint_state': [],
            'robot_state': [],
            'lidar': [],
            'odometry': [],
            'imu': [],
            'camera': [],
            'camera_info': [],
            'voxel': [],
            'low_state': [],
        }

        num_robots = len(self.config.robot_ip_list)
        
        for i in range(num_robots):
            # Define topics depending on connection mode
            if self.config.conn_mode == 'single':
                joint_topic = 'joint_states'
                robot_state_topic = 'go2_states'
                lidar_topic = 'point_cloud2'
                odom_topic = 'odom'
                imu_topic = 'imu'
                camera_topic = 'camera/image_raw'
                camera_info_topic = 'camera/camera_info'
                voxel_topic = '/utlidar/voxel_map_compressed'
                low_state_topic = 'lowstate'
            else:
                prefix = f'robot{i}'
                joint_topic = f'{prefix}/joint_states'
                robot_state_topic = f'{prefix}/go2_states'
                lidar_topic = f'{prefix}/point_cloud2'
                odom_topic = f'{prefix}/odom'
                imu_topic = f'{prefix}/imu'
                camera_topic = f'{prefix}/camera/image_raw'
                camera_info_topic = f'{prefix}/camera/camera_info'
                voxel_topic = f'{prefix}/utlidar/voxel_map_compressed'
                low_state_topic = f'{prefix}/lowstate'

            # Create publishers
            publishers['joint_state'].append(
                self.create_publisher(JointState, joint_topic, qos_profile))
            publishers['robot_state'].append(
                self.create_publisher(Go2State, robot_state_topic, qos_profile))
            publishers['lidar'].append(
                self.create_publisher(
                    PointCloud2, lidar_topic, best_effort_qos,
                    qos_overriding_options=QoSOverridingOptions.with_default_policies()))
            publishers['odometry'].append(
                self.create_publisher(Odometry, odom_topic, qos_profile))
            publishers['imu'].append(
                self.create_publisher(IMU, imu_topic, qos_profile))
            publishers['low_state'].append(
                self.create_publisher(LowState, low_state_topic, qos_profile))

            if self.config.enable_video:
                publishers['camera'].append(
                    self.create_publisher(
                        Image, camera_topic, best_effort_qos,
                        qos_overriding_options=QoSOverridingOptions.with_default_policies()))
                publishers['camera_info'].append(
                    self.create_publisher(
                        CameraInfo, camera_info_topic, best_effort_qos,
                        qos_overriding_options=QoSOverridingOptions.with_default_policies()))

            if self.config.publish_raw_voxel:
                publishers['voxel'].append(
                    self.create_publisher(VoxelMapCompressed, voxel_topic, best_effort_qos))

        return publishers

    def _setup_subscribers(self) -> None:
        """ROS2 subscribers setup"""
        qos_profile = QoSProfile(depth=10)

        # Command subscribers
        num_robots = len(self.config.robot_ip_list)
        
        if self.config.conn_mode == 'single':
            self.create_subscription(
                Twist, 'cmd_vel_out',
                lambda msg: self._on_cmd_vel(msg, "0"), qos_profile)
            self.create_subscription(
                WebRtcReq, 'webrtc_req',
                lambda msg: self._on_webrtc_req(msg, "0"), qos_profile)
        else:
            for i in range(num_robots):
                self.create_subscription(
                    Twist, f'robot{i}/cmd_vel_out',
                    lambda msg, robot_id=str(i): self._on_cmd_vel(msg, robot_id), qos_profile)
                self.create_subscription(
                    WebRtcReq, f'robot{i}/webrtc_req',
                    lambda msg, robot_id=str(i): self._on_webrtc_req(msg, robot_id), qos_profile)

        # Joystick subscriber
        self.create_subscription(Joy, 'joy', self._on_joy, qos_profile)

        # CycloneDDS support
        if self.config.conn_type == 'cyclonedds':
            self.create_subscription(
                LowState, 'lowstate',
                self._on_cyclonedds_low_state, qos_profile)
            self.create_subscription(
                PoseStamped, '/utlidar/robot_pose',
                self._on_cyclonedds_pose, qos_profile)
            self.create_subscription(
                PointCloud2, '/utlidar/cloud',
                self._on_cyclonedds_lidar, qos_profile)

    def _on_set_parameters(self, params) -> SetParametersResult:
        """Callback for parameter changes"""
        result = SetParametersResult(successful=True)

        try:
            for p in params:
                if p.name == 'obstacle_avoidance':
                    self.get_logger().info(f'New obstacle_avoidance value: {p.value}')
                    self.config.obstacle_avoidance = p.value
                    
                    try:
                        self.robot_control_service.set_obstacle_avoidance(p.value, "0")
                    except Exception as e:
                        self.get_logger().error(f"Failed to set obstacle avoidance: {e}")
                        result.successful = False
                        result.reason = str(e)
                        break
                    
                    result.successful = True
                    result.reason = 'Updated obstacle_avoidance'
                    break
        except Exception as e:
            self.get_logger().error(f"Error setting parameters: {e}")
            result.successful = False
            result.reason = str(e)
            
        return result

    def _on_cmd_vel(self, msg: Twist, robot_id: str) -> None:
        """Callback for movement commands"""
        self.robot_control_service.handle_cmd_vel(
            msg.linear.x, msg.linear.y, msg.angular.z, 
            robot_id, self.config.obstacle_avoidance
        )

    def _on_webrtc_req(self, msg: WebRtcReq, robot_id: str) -> None:
        """Callback for WebRTC requests"""
        self.robot_control_service.handle_webrtc_request(
            msg.api_id, msg.parameter, msg.topic, msg.id, robot_id
        )

    def _on_joy(self, msg: Joy) -> None:
        """Callback for joystick"""
        self.joy_state = msg

    def _on_robot_validated(self, robot_id: str) -> None:
        """Callback after robot validation"""
        self.get_logger().info(f"Robot {robot_id} validated and ready")

    def _on_robot_data_received(self, msg: Dict[str, Any], robot_id: str) -> None:
        """Callback for receiving data from robot"""
        self.robot_data_service.process_webrtc_message(msg, robot_id)

    async def _on_video_frame(self, track: "MediaStreamTrack", robot_id: str) -> None:
        """Callback for processing video frames.

        Over the remote/TURN path the aiortc decoder often starts mid-GOP and
        stays stuck ("failed to decode") until the robot happens to send a
        keyframe. To recover the feed quickly we proactively request a keyframe
        (RTCP PLI) right away and keep retrying on a short interval until the
        first frame actually decodes; once frames flow we stop nudging.
        """
        logger.info(f"Video track started for robot {robot_id}; requesting keyframe")

        # Mutable flag shared with the nudger task (avoids closure-rebind issues).
        state = {"decoding": False}

        async def _nudge_until_decoding():
            # Periodically ask for a keyframe until the decoder locks on.
            for _ in range(20):  # ~20s of nudging, then give up to avoid spam
                if state["decoding"]:
                    return
                await self.webrtc_adapter.request_keyframe(robot_id)
                await asyncio.sleep(1.0)

        nudge_task = asyncio.ensure_future(_nudge_until_decoding())

        try:
            while True:
                try:
                    frame = await track.recv()
                    if not state["decoding"]:
                        state["decoding"] = True
                        logger.info(f"First video frame decoded for robot {robot_id}")
                    img = frame.to_ndarray(format="bgr24")

                    camera_data = CameraData(
                        image=img,
                        height=img.shape[0],
                        width=img.shape[1],
                        encoding="bgr8"
                    )

                    robot_data = RobotData(
                        robot_id=robot_id,
                        timestamp=0.0,
                        camera_data=camera_data
                    )

                    self.ros2_publisher.publish_camera_data(robot_data)
                    await asyncio.sleep(0)

                except Exception as e:
                    # Track ended (reconnect / robot dropped). Exit so a fresh
                    # track callback can take over on the next connection.
                    logger.info(f"Video track ended for robot {robot_id}: {e}")
                    break
        finally:
            state["decoding"] = True  # stop the nudger
            nudge_task.cancel()

    # CycloneDDS callbacks
    def _on_cyclonedds_low_state(self, msg: LowState) -> None:
        """Processing LowState for CycloneDDS"""
        # You can add processing for CycloneDDS here if needed
        pass

    def _on_cyclonedds_pose(self, msg: PoseStamped) -> None:
        """Processing pose for CycloneDDS"""
        # You can add processing for CycloneDDS here if needed
        pass

    def _on_cyclonedds_lidar(self, msg: PointCloud2) -> None:
        """Processing lidar for CycloneDDS"""
        # You can add processing for CycloneDDS here if needed
        pass

    async def connect_robots(self) -> None:
        """Connect to robots"""
        # Both 'webrtc' (LAN/LocalSTA) and 'remote' (Unitree TURN server) use the
        # WebRTC adapter; only 'cyclonedds' bypasses it. The adapter / Go2Connection
        # picks the concrete connection method from CONN_TYPE internally.
        if self.config.conn_type in ('webrtc', 'remote'):
            for i, robot_ip in enumerate(self.config.robot_ip_list):
                try:
                    await self.webrtc_adapter.connect(str(i))
                except Exception as e:
                    # Don't crash the node if the robot isn't reachable yet (e.g.
                    # remote: "device not online" right after power-on). The
                    # control loop monitors health and keeps retrying the
                    # connection, so startup stays resilient.
                    self.get_logger().warning(
                        f"Initial connect to robot {i} failed: {e}. "
                        f"Control loop will keep retrying."
                    )

    async def _reconnect_robot(self, robot_id: str) -> bool:
        """Tear down and re-establish a robot's WebRTC connection.

        The Go2 (especially over the remote/TURN path) periodically drops its
        peer connection. Returns True on success, False so the caller can back
        off and retry — never raises, so a transient outage doesn't kill the node.

        Sets `self._cloud_blocked` when the failure came from Unitree's cloud
        rejecting us outright (HTTP 5xx, notably the 567 its WAF returns once
        it decides we're abusive). That distinction matters: a dropped data
        channel deserves a quick retry, but hammering a blocked auth endpoint
        every few seconds is what earns — and sustains — the block.
        """
        self._cloud_blocked = False
        try:
            await self.webrtc_adapter.disconnect(robot_id)
        except Exception as e:
            self.get_logger().warning(f"Error tearing down robot {robot_id}: {e}")
        try:
            await self.webrtc_adapter.connect(robot_id)
            self.get_logger().info(f"Reconnected to robot {robot_id}")
            return True
        except Exception as e:
            if self._is_cloud_rejection(e):
                self._cloud_blocked = True
                # The cached token is useless if the cloud is refusing us;
                # drop it so recovery starts from a clean login.
                invalidate_remote_token()
            self.get_logger().warning(f"Reconnect to robot {robot_id} failed: {e}")
            return False

    @staticmethod
    def _is_cloud_rejection(exc: Exception) -> bool:
        """True if `exc` looks like Unitree's cloud refusing the request.

        The library surfaces these as HTTPError, whose str() is
        'HTTP Error 567: ...'. 567 is the Tencent EdgeOne WAF block page;
        429/5xx are the other rate-limit shapes worth backing off from.

        The same block also arrives as a 2xx carrying the HTML challenge
        page, which blows up in `resp.json()`; `Go2Connection` re-raises that
        as CloudRejectedError, and a bare JSONDecodeError from anywhere else
        in the cloud handshake means the same thing.
        """
        if isinstance(exc, (CloudRejectedError, json.JSONDecodeError)):
            return True
        status = getattr(exc, "code", None) or getattr(exc, "status", None)
        if isinstance(status, int) and (status == 429 or status >= 500):
            return True
        return bool(re.search(r"HTTP Error (?:429|5\d\d)", str(exc)))

    async def run_robot_control_loop(self, robot_id: str) -> None:
        """Main robot control loop.

        Watches the WebRTC channel health and reconnects when the robot drops
        the connection. Only applies to the WebRTC/remote transports; CycloneDDS
        has no adapter connection to monitor.
        """
        watch_connection = self.config.conn_type in ('webrtc', 'remote')
        backoff = 1.0  # seconds; grows up to 15s between reconnect attempts
        # Unitree's cloud rate-limits: back off much further (and log once
        # rather than every attempt) so a block can age out instead of being
        # refreshed by our own retries.
        CLOUD_BACKOFF_CAP = 300.0
        cloud_strikes = 0
        while True:
            try:
                # If the channel has dropped, reconnect (with backoff) before
                # trying to send anything — sends into a dead channel fail silently.
                if watch_connection and not self.webrtc_adapter.is_connected(robot_id):
                    if not getattr(self, "_cloud_blocked", False):
                        self.get_logger().warning(
                            f"Robot {robot_id} connection lost; attempting reconnect..."
                        )
                    if await self._reconnect_robot(robot_id):
                        backoff = 1.0
                        cloud_strikes = 0
                    elif getattr(self, "_cloud_blocked", False):
                        cloud_strikes += 1
                        backoff = min(max(backoff, 15.0) * 2, CLOUD_BACKOFF_CAP)
                        if cloud_strikes == 3:
                            self.get_logger().error(
                                "Unitree cloud is refusing our requests (HTTP 567 = "
                                "WAF block). Backing off up to "
                                f"{int(CLOUD_BACKOFF_CAP)}s between attempts; the robot "
                                "will stay offline until it clears. Use CONN_TYPE=local "
                                "on the LAN to bypass the cloud entirely."
                            )
                        await asyncio.sleep(backoff)
                        continue
                    else:
                        await asyncio.sleep(backoff)
                        backoff = min(backoff * 2, 15.0)
                        continue

                # Process joystick commands
                if self.joy_state.buttons:
                    self.robot_control_service.handle_joy_command(
                        self.joy_state.buttons, robot_id
                    )

                # Process WebRTC commands
                self.webrtc_adapter.process_webrtc_commands(robot_id)

                await asyncio.sleep(0.1)

            except Exception as e:
                # Log and keep the loop alive — a transient error must not kill
                # the node (which would require a full container restart).
                self.get_logger().error(f"Error in control loop for robot {robot_id}: {e}")
                await asyncio.sleep(1.0) 