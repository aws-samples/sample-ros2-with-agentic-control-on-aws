# Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
# SPDX-License-Identifier: MIT-0
"""Unitree Go2 Air — Nova Sonic voice + text agent (ROS2 transport).

Routes every robot command through the `foxglove_bridge` that this SDK's
ROS2 container exposes (default ws://localhost:8766) instead of Unitree's
WebRTC TURN server. The container owns the robot connection (LAN + AES-128
key, see docker/.env), so this agent needs no Unitree credentials — only AWS
credentials for Amazon Nova Sonic (Bedrock).

For the direct-WebRTC variant (no container), see main_sonic.py.

Run from a real terminal, from the repo root:
    python -m voice_agent.main                  # voice + text
    python -m voice_agent.main --text-only      # skip mic
"""

import argparse
import asyncio
import sys

from . import config
from .client_registry import set_client
from .go2_ros2_client import Go2ROS2Client
from .session import configure_logging, run_agent


async def run(text_only: bool, voice: str, log_level: str) -> None:
    configure_logging(
        log_level,
        app_loggers=("__main__", "voice_agent"),
        quiet_loggers=("botocore", "strands", "aiohttp", "asyncio"),
    )

    print("=" * 60)
    print("  Unitree Go2 Air — Nova Sonic agent (ROS2 transport)")
    print("=" * 60)
    print(f"  Connection:  Foxglove ws://{config.ROS_BRIDGE_HOST}:{config.ROS_BRIDGE_PORT}")
    print(f"  Model:       {config.NOVA_SONIC_MODEL_ID} ({config.NOVA_SONIC_REGION})")
    print(f"  Voice:       {voice}")
    print(f"  Mode:        {'text-only' if text_only else 'voice + text'}")
    print()

    print("[*] Connecting to Go2 via Foxglove bridge...")
    client = Go2ROS2Client()
    if not client.connect():
        print("[!] Failed to connect to the bridge. Is the ROS2 container running?")
        print(f"    Expected ws://{config.ROS_BRIDGE_HOST}:{config.ROS_BRIDGE_PORT}")
        sys.exit(1)
    set_client(client)  # tools' get_client() now returns the ROS2 client
    print("[✓] Robot connected!")
    print()

    await run_agent(client, text_only=text_only, voice=voice)


def main() -> None:
    parser = argparse.ArgumentParser(description="Go2 Nova Sonic agent (ROS2)")
    parser.add_argument("--text-only", action="store_true", help="Disable mic")
    parser.add_argument("--voice", default="en-us.matthew", help="Nova Sonic voice ID")
    parser.add_argument("--log-level", default="INFO", help="Logging level")
    args = parser.parse_args()

    try:
        asyncio.run(run(args.text_only, args.voice, args.log_level))
    except KeyboardInterrupt:
        print("\n[*] Bye!")


if __name__ == "__main__":
    main()
