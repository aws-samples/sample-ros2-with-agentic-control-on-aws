# Copyright (c) 2024, RoboVerse community
# SPDX-License-Identifier: BSD-3-Clause
# Modified by Amazon.com, Inc. or its affiliates.
"""
Infrastructure layer - adapters for external systems.

NOTE: We deliberately do NOT re-export submodules here. Eager
re-export pulled `ros2_publisher` (which depends on rclpy + the
WASM-based lidar decoder) into the import path of every webrtc-only
caller. That triggered an import-time side effect that wedged
aiortc's DTLS task at "ICE completed → peer connecting" forever.
Import the concrete modules from their full paths instead.
"""
