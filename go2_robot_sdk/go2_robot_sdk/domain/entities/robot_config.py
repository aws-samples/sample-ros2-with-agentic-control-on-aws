# Copyright (c) 2024, RoboVerse community
# SPDX-License-Identifier: BSD-3-Clause
# Modified by Amazon.com, Inc. or its affiliates.

from dataclasses import dataclass
from typing import List


@dataclass
class RobotConfig:
    """Robot configuration parameters"""
    robot_ip_list: List[str]
    token: str
    conn_type: str
    enable_video: bool
    decode_lidar: bool
    publish_raw_voxel: bool
    obstacle_avoidance: bool
    conn_mode: str  # 'single' or 'multi'
    # Per-device AES-128 key (32 hex chars). Required for Go2 firmware
    # >= 1.1.15 / G1 >= 1.5.1 LAN handshake (con_notify returns data2=3).
    # Comma-separated list aligned with robot_ip_list, or single value
    # applied to all robots. Empty for older firmware.
    aes_128_key_list: List[str] = None

    @classmethod
    def from_params(cls, robot_ip: str, token: str, conn_type: str,
                   enable_video: bool, decode_lidar: bool,
                   publish_raw_voxel: bool, obstacle_avoidance: bool,
                   aes_128_key: str = ""):
        """Создание конфигурации из параметров"""
        robot_ip_list = robot_ip.replace(" ", "").split(",")
        conn_mode = "single" if (
            len(robot_ip_list) == 1 and conn_type != "cyclonedds") else "multi"

        keys = [k for k in aes_128_key.replace(" ", "").split(",") if k]
        if len(keys) == 0:
            aes_128_key_list = [""] * len(robot_ip_list)
        elif len(keys) == 1:
            aes_128_key_list = keys * len(robot_ip_list)
        elif len(keys) == len(robot_ip_list):
            aes_128_key_list = keys
        else:
            raise ValueError(
                f"aes_128_key count ({len(keys)}) must be 0, 1, or match "
                f"robot_ip count ({len(robot_ip_list)})"
            )

        return cls(
            robot_ip_list=robot_ip_list,
            token=token,
            conn_type=conn_type,
            enable_video=enable_video,
            decode_lidar=decode_lidar,
            publish_raw_voxel=publish_raw_voxel,
            obstacle_avoidance=obstacle_avoidance,
            conn_mode=conn_mode,
            aes_128_key_list=aes_128_key_list,
        )
