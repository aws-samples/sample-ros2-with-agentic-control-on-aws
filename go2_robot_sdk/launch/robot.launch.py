# Copyright (c) 2024, RoboVerse community
# SPDX-License-Identifier: BSD-3-Clause
# Modified by Amazon.com, Inc. or its affiliates.

import os
from typing import List
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import FrontendLaunchDescriptionSource, PythonLaunchDescriptionSource


class Go2LaunchConfig:
    """Configuration container for Go2 robot launch parameters"""
    
    def __init__(self):
        # Environment variables
        self.robot_token = os.getenv('ROBOT_TOKEN', '')
        self.robot_ip = os.getenv('ROBOT_IP', '')
        self.map_name = os.getenv('MAP_NAME', '3d_map')
        self.save_map = os.getenv('MAP_SAVE', 'true')
        self.conn_type = os.getenv('CONN_TYPE', 'webrtc')

        # Two connection types reach the robot without a LAN IP, so ROBOT_IP is
        # normally empty for both. Seed a single placeholder so the IP list is
        # non-empty: that keeps conn_mode == "single" and avoids passing empty-list
        # parameters to ROS2 nodes (which raises a tuple-type error). Getting this
        # wrong is quiet and nasty — an empty list makes conn_mode "multi", which
        # namespaces every topic under robot0/ and loads multi_go2.urdf.
        #
        #   remote — reached via Unitree's TURN server, identified by UNITREE_SERIAL
        #            (the driver ignores robot_ip; see go2_connection._build_connection)
        #   bridge — the robot link lives somewhere else entirely. Used when a
        #            laptop on the dog's AP owns the WebRTC session and feeds this
        #            ROS graph through foxglove_bridge's clientPublish
        #            (scripts/go2_ap_bridge.py). go2_driver_node still starts, but
        #            connect_robots() and the reconnect watchdog both skip any
        #            conn_type outside ('webrtc','remote'), so it holds no link and
        #            just keeps /cmd_vel_out and /webrtc_req present in the graph —
        #            which is what lets the bridge subscribe to them immediately.
        if not self.robot_ip and self.conn_type in ('remote', 'bridge'):
            self.robot_ip = self.conn_type
        self.robot_ip_list = self._parse_ip_list(self.robot_ip)

        # An empty list is never valid, and left alone it fails 30 lines later with
        # "Expected 'value' to be one of [float, int, str, bool, bytes], but got
        # '()'" — a ROS parameter type error that says nothing about the cause. It
        # happens whenever conn_type needs no LAN IP but is not in the placeholder
        # list above (the way CONN_TYPE=bridge did against an image predating that
        # guard). Say what is actually wrong instead.
        if not self.robot_ip_list:
            raise RuntimeError(
                f"No robot IP resolved: CONN_TYPE={self.conn_type!r} with an empty "
                f"ROBOT_IP. Set ROBOT_IP, or use a CONN_TYPE that seeds a "
                f"placeholder ('remote' or 'bridge'). If you are running "
                f"CONN_TYPE=bridge, this image predates that support — rebuild and "
                f"push it (make push-image && make restart-ec2)."
            )
        
        # Derived configurations
        self.conn_mode = self._determine_connection_mode()
        self.rviz_config = self._get_rviz_config()
        self.urdf_file = self._get_urdf_file()
        
        # Package paths
        self.package_dir = get_package_share_directory('go2_robot_sdk')
        self.config_paths = self._get_config_paths()
        
        print(f"� Go2 Launch Configuration:")
        print(f"   Robot IPs: {self.robot_ip_list}")
        print(f"   Connection: {self.conn_type} ({self.conn_mode})")
        print(f"   URDF: {self.urdf_file}")
    
    def _parse_ip_list(self, robot_ip: str) -> List[str]:
        """Parse robot IP addresses from environment variable"""
        return robot_ip.replace(" ", "").split(",") if robot_ip else []
    
    def _determine_connection_mode(self) -> str:
        """Determine connection mode based on IP list and connection type"""
        return "single" if len(self.robot_ip_list) == 1 and self.conn_type != "cyclonedx" else "multi"
    
    def _get_rviz_config(self) -> str:
        """Get appropriate RViz configuration file"""
        if self.conn_type == 'cyclonedx':
            return "cyclonedx_config.rviz"
        elif self.conn_mode == 'single':
            return "single_robot_conf.rviz"
        else:
            return "multi_robot_conf.rviz"
    
    def _get_urdf_file(self) -> str:
        """Get appropriate URDF file"""
        return 'go2.urdf' if self.conn_mode == 'single' else 'multi_go2.urdf'
    
    def _get_config_paths(self) -> dict:
        """Get all configuration file paths"""
        return {
            'joystick': os.path.join(self.package_dir, 'config', 'joystick.yaml'),
            'twist_mux': os.path.join(self.package_dir, 'config', 'twist_mux.yaml'),
            'slam': os.path.join(self.package_dir, 'config', 'mapper_params_online_async.yaml'),
            'nav2': os.path.join(self.package_dir, 'config', 'nav2_params.yaml'),
            'rviz': os.path.join(self.package_dir, 'config', self.rviz_config),
            'urdf': os.path.join(self.package_dir, 'urdf', self.urdf_file),
        }


class Go2NodeFactory:
    """Factory for creating Go2 robot nodes"""
    
    def __init__(self, config: Go2LaunchConfig):
        self.config = config
    
    def create_launch_arguments(self) -> List[DeclareLaunchArgument]:
        """Create all launch arguments"""
        # Which camera topic the detector listens on, which is NOT the same topic in
        # every connection mode. With a local link (webrtc/remote) go2_driver_node
        # publishes raw /camera/image_raw in this graph. Under CONN_TYPE=bridge the
        # frames instead arrive from a laptop on the dog's AP, already JPEG-encoded
        # (scripts/go2_ap_bridge.py sends only /camera/image_raw/compressed — raw
        # would never fit through the SSM tunnel), so on the raw topic the detector
        # sees nothing at all and every consumer downstream of it goes quiet:
        # no /detected_objects, so no 'find a person', no start_tracking / 'follow
        # me' and no greeter; no /annotated_image, so no boxes in Lichtblick and
        # nothing for the KVS producer to stream, which takes the 'kvs describe
        # scene' commands with it. Only the voice agent's own 'what do you see?'
        # survives, because it reads the compressed topic off foxglove_bridge itself
        # and never touches the detector — which is exactly why this failed so
        # asymmetrically and looked like a camera problem rather than a topic name.
        #
        # Derived from CONN_TYPE rather than passed at the call site because the mode
        # already crosses into the container as an env var: one source of truth, and
        # the fix reaches local Docker AP-bridge runs too. Still overridable.
        coco_bridge_in = self.config.conn_type == 'bridge'
        if coco_bridge_in:
            print("   Detector input: /camera/image_raw/compressed (bridge mode)")
        return [
            DeclareLaunchArgument('rviz2', default_value='true', description='Launch RViz2'),
            DeclareLaunchArgument('nav2', default_value='true', description='Launch Nav2'),
            DeclareLaunchArgument('slam', default_value='true', description='Launch SLAM'),
            DeclareLaunchArgument('foxglove', default_value='true', description='Launch Foxglove Bridge'),
            DeclareLaunchArgument('joystick', default_value='true', description='Launch joystick'),
            DeclareLaunchArgument('teleop', default_value='true', description='Launch teleoperation'),
            DeclareLaunchArgument('kvs', default_value='false', description='Launch KVS producer (stream camera to AWS; configure via KVS_* env vars)'),
            DeclareLaunchArgument('coco', default_value='true', description='Launch COCO object detector (/detected_objects)'),
            DeclareLaunchArgument('coco_model', default_value='balanced', description='Detector backbone: fast | balanced | accurate | best (accuracy ascending, speed descending)'),
            DeclareLaunchArgument('coco_threshold', default_value='0.5', description='Minimum detection confidence to publish'),
            # Where the detector gets frames — see coco_bridge_in above. Raw for a
            # local link, compressed under CONN_TYPE=bridge. Override both together
            # to point it anywhere else.
            DeclareLaunchArgument(
                'coco_image_topic',
                default_value='/camera/image_raw/compressed' if coco_bridge_in else '/camera/image_raw',
                description='Detector input topic'),
            DeclareLaunchArgument(
                'coco_compressed',
                default_value='true' if coco_bridge_in else 'false',
                description='Input topic is sensor_msgs/CompressedImage'),
            # 'cpu' stays the default so local Mac/Docker runs are unaffected —
            # there is no CUDA there. The cloud stack passes coco_device:=cuda
            # (see deployment/lib/go2-ec2-stack.ts), which is what moves
            # inference onto the g4dn instance's T4.
            DeclareLaunchArgument('coco_device', default_value='cpu', description='Torch device for the detector: cpu | cuda'),
            DeclareLaunchArgument('compressed', default_value='true', description='Republish camera/annotated images as compressed JPEG (needed for remote/tunneled bridge)'),
        ]
    
    def create_robot_state_nodes(self) -> List[Node]:
        """Create robot state publisher nodes"""
        nodes = []
        use_sim_time = LaunchConfiguration('use_sim_time', default='false')
        
        if self.config.conn_mode == 'single':
            # Single robot configuration
            robot_desc = self._load_urdf_content(self.config.config_paths['urdf'])
            
            nodes.extend([
                Node(
                    package='robot_state_publisher',
                    executable='robot_state_publisher',
                    name='go2_robot_state_publisher',
                    output='screen',
                    parameters=[{
                        'use_sim_time': use_sim_time,
                        'robot_description': robot_desc
                    }],
                    arguments=[self.config.config_paths['urdf']]
                ),
                self._create_pointcloud_to_laserscan_node()
            ])
        else:
            # Multi-robot configuration
            base_urdf = self._load_urdf_content(self.config.config_paths['urdf'])
            
            for i, _ in enumerate(self.config.robot_ip_list):
                robot_desc = base_urdf.format(robot_num=f"robot{i}")
                
                nodes.extend([
                    Node(
                        package='robot_state_publisher',
                        executable='robot_state_publisher',
                        name='go2_robot_state_publisher',
                        output='screen',
                        namespace=f"robot{i}",
                        parameters=[{
                            'use_sim_time': use_sim_time,
                            'robot_description': robot_desc
                        }],
                        arguments=[self.config.config_paths['urdf']]
                    ),
                    self._create_pointcloud_to_laserscan_node(f"robot{i}")
                ])
        
        return nodes
    
    def _load_urdf_content(self, urdf_path: str) -> str:
        """Load URDF file content"""
        with open(urdf_path, 'r') as file:
            return file.read()
    
    def _create_pointcloud_to_laserscan_node(self, namespace: str = None) -> Node:
        """Create pointcloud to laserscan conversion node"""
        if namespace:
            # Multi-robot setup
            return Node(
                package='pointcloud_to_laserscan',
                executable='pointcloud_to_laserscan_node',
                name=f'{namespace}_pointcloud_to_laserscan',
                remappings=[
                    ('cloud_in', f'{namespace}/point_cloud2'),
                    ('scan', f'{namespace}/scan'),
                ],
                parameters=[{
                    'target_frame': f'{namespace}/base_link',
                    'max_height': 0.1
                }],
                output='screen',
            )
        else:
            # Single robot setup
            return Node(
                package='pointcloud_to_laserscan',
                executable='pointcloud_to_laserscan_node',
                name='go2_pointcloud_to_laserscan',
                remappings=[
                    ('cloud_in', 'point_cloud2'),
                    ('scan', 'scan'),
                ],
                parameters=[{
                    'target_frame': 'base_link',
                    'max_height': 0.5
                }],
                output='screen',
            )
    
    def create_core_nodes(self) -> List[Node]:
        """Create core Go2 robot nodes"""
        return [
            # Main robot driver (clean architecture)
            Node(
                package='go2_robot_sdk',
                executable='go2_driver_node',
                name='go2_driver_node',
                output='screen',
                parameters=[{
                    'robot_ip': self.config.robot_ip,
                    'token': self.config.robot_token,
                    'conn_type': self.config.conn_type
                }],
            ),
            # LiDAR processing node (new separate package)
            Node(
                package='lidar_processor',
                executable='lidar_to_pointcloud',
                name='lidar_to_pointcloud',
                parameters=[{
                    'robot_ip_lst': self.config.robot_ip_list,
                    'map_name': self.config.map_name,
                    'map_save': self.config.save_map
                }],
            ),
            # Advanced point cloud aggregator
            Node(
                package='lidar_processor',
                executable='pointcloud_aggregator',
                name='pointcloud_aggregator',
                parameters=[{
                    'max_range': 20.0,
                    'min_range': 0.1,
                    'height_filter_min': -2.0,
                    'height_filter_max': 3.0,
                    'downsample_rate': 5,
                    'publish_rate': 10.0
                }],
            ),
            # Scene description ("what do you see?") now runs host-side in the
            # voice_agent's describe_scene tool: it reads /camera/image_raw off
            # the foxglove bridge and calls Bedrock directly, so Nova Sonic
            # speaks the answer. No container node / AWS creds required here.
            # (The old tts_node / ElevenLabs speaker path was removed with it —
            # nothing publishes to /tts anymore.)
        ]

    def create_teleop_nodes(self) -> List[Node]:
        """Create teleoperation and joystick nodes"""
        use_sim_time = LaunchConfiguration('use_sim_time', default='false')
        with_joystick = LaunchConfiguration('joystick', default='true')
        with_teleop = LaunchConfiguration('teleop', default='true')
        
        return [
            # Joystick node
            Node(
                package='joy',
                executable='joy_node',
                condition=IfCondition(with_joystick),
                parameters=[self.config.config_paths['joystick']]
            ),
            # Teleop twist joy node
            Node(
                package='teleop_twist_joy',
                executable='teleop_node',
                name='go2_teleop_node',
                condition=IfCondition(with_joystick),
                parameters=[self.config.config_paths['twist_mux']],
            ),
            # Twist multiplexer
            Node(
                package='twist_mux',
                executable='twist_mux',
                output='screen',
                condition=IfCondition(with_teleop),
                parameters=[
                    {'use_sim_time': use_sim_time},
                    self.config.config_paths['twist_mux']
                ],
            ),
        ]
    
    def create_visualization_nodes(self) -> List[Node]:
        """Create visualization nodes (RViz, Foxglove)"""
        with_rviz2 = LaunchConfiguration('rviz2', default='true')
        
        return [
            # RViz2
            Node(
                package='rviz2',
                executable='rviz2',
                condition=IfCondition(with_rviz2),
                name='go2_rviz2',
                output='screen',
                arguments=['-d', self.config.config_paths['rviz']],
                parameters=[{'use_sim_time': False}]
            ),
        ]
    
    def create_kvs_nodes(self) -> List[Node]:
        """Create KVS producer node (streams camera to AWS KVS).

        The node reads its config from env vars (KVS_IMAGE_TOPIC,
        KVS_SEGMENT_SEC, KVS_STREAM_NAME, ...). Set those before launching to
        stream the COCO overlay at low latency, e.g.:
          KVS_IMAGE_TOPIC=/annotated_image KVS_SEGMENT_SEC=2 ros2 launch ... kvs:=true
        On Fargate the task definition sets these env vars directly.
        """
        with_kvs = LaunchConfiguration('kvs', default='false')
        return [
            Node(
                package='go2_robot_sdk',
                executable='kvs_producer_node',
                name='kvs_producer_node',
                output='screen',
                condition=IfCondition(with_kvs),
            ),
        ]

    def create_coco_nodes(self) -> List[Node]:
        """Create COCO object-detector node.

        Subscribes to coco_image_topic — raw /camera/image_raw with a local robot
        link, /camera/image_raw/compressed under CONN_TYPE=bridge — and publishes
        /detected_objects
        (vision_msgs/Detection2DArray) plus /annotated_image. Runs a torchvision
        Faster R-CNN; the 'balanced' default (ResNet50-FPN at a reduced internal
        resize) is ~14 mAP better than the old MobileNet-320 while still clearing
        the node's 2 fps throttle on a CPU. Override with coco_model:=.

        On CPU the backbone choice is a frame-rate budget (see the MODELS table in
        coco_detector_node.py for measured per-frame times). With coco_device:=cuda
        that budget mostly disappears, which is why the cloud stack runs the
        slowest, most accurate backbone.
        """
        with_coco = LaunchConfiguration('coco', default='true')
        return [
            Node(
                package='coco_detector',
                executable='coco_detector_node',
                name='coco_detector_node',
                output='screen',
                parameters=[{
                    'model': LaunchConfiguration('coco_model', default='balanced'),
                    'device': LaunchConfiguration('coco_device', default='cpu'),
                    'detection_threshold': ParameterValue(
                        LaunchConfiguration('coco_threshold', default='0.5'),
                        value_type=float),
                    # Declaring coco_image_topic/coco_compressed is not enough: without
                    # these two lines the launch arguments went nowhere and the node
                    # kept its own defaults, so even passing them explicitly on the
                    # command line — as this file and scripts/go2_ap_bridge.py both
                    # used to instruct — left the detector on the raw topic. That is
                    # what made AP mode look like a camera problem: frames arrived,
                    # nothing consumed them.
                    'image_topic': LaunchConfiguration('coco_image_topic'),
                    'image_compressed': ParameterValue(
                        LaunchConfiguration('coco_compressed'),
                        value_type=bool),
                }],
                condition=IfCondition(with_coco),
            ),
        ]

    def create_compressed_image_nodes(self) -> List[Node]:
        """Create the JPEG image-republisher node.

        Converts /camera/image_raw and /annotated_image into compressed
        (/camera/image_raw/compressed, /annotated_image/compressed) so the
        Foxglove bridge can serve them over a low-bandwidth SSM tunnel without
        lag. Essential for remote (Fargate / EC2) operation; harmless locally.
        """
        with_compressed = LaunchConfiguration('compressed', default='true')
        return [
            Node(
                package='go2_robot_sdk',
                executable='image_republisher_node',
                name='image_republisher_node',
                output='screen',
                condition=IfCondition(with_compressed),
            ),
        ]

    def create_include_launches(self) -> List[IncludeLaunchDescription]:
        """Create included launch descriptions"""
        use_sim_time = LaunchConfiguration('use_sim_time', default='false')
        with_foxglove = LaunchConfiguration('foxglove', default='true')
        with_slam = LaunchConfiguration('slam', default='true')
        with_nav2 = LaunchConfiguration('nav2', default='true')
        
        foxglove_launch = os.path.join(
            get_package_share_directory('foxglove_bridge'),
            'launch', 'foxglove_bridge_launch.xml'
        )
        
        return [
            # Foxglove Bridge
            IncludeLaunchDescription(
                FrontendLaunchDescriptionSource(foxglove_launch),
                condition=IfCondition(with_foxglove),
            ),
            # SLAM Toolbox
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    os.path.join(get_package_share_directory('slam_toolbox'),
                                'launch', 'online_async_launch.py')
                ]),
                condition=IfCondition(with_slam),
                launch_arguments={
                    'slam_params_file': self.config.config_paths['slam'],
                    'use_sim_time': use_sim_time,
                }.items(),
            ),
            # Nav2
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    os.path.join(get_package_share_directory('nav2_bringup'),
                                'launch', 'navigation_launch.py')
                ]),
                condition=IfCondition(with_nav2),
                launch_arguments={
                    'params_file': self.config.config_paths['nav2'],
                    'use_sim_time': use_sim_time,
                }.items(),
            ),
        ]


def generate_launch_description():
    """Generate the launch description for Go2 robot system"""
    
    # Initialize configuration and factory
    config = Go2LaunchConfig()
    factory = Go2NodeFactory(config)
    
    # Create all components
    launch_args = factory.create_launch_arguments()
    robot_state_nodes = factory.create_robot_state_nodes()
    core_nodes = factory.create_core_nodes()
    teleop_nodes = factory.create_teleop_nodes()
    visualization_nodes = factory.create_visualization_nodes()
    kvs_nodes = factory.create_kvs_nodes()
    coco_nodes = factory.create_coco_nodes()
    compressed_image_nodes = factory.create_compressed_image_nodes()
    include_launches = factory.create_include_launches()

    # Combine all elements
    launch_entities = (
        launch_args +
        robot_state_nodes +
        core_nodes +
        teleop_nodes +
        visualization_nodes +
        kvs_nodes +
        coco_nodes +
        compressed_image_nodes +
        include_launches
    )
    
    return LaunchDescription(launch_entities)