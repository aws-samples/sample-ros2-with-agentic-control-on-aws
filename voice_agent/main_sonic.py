# Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
# SPDX-License-Identifier: MIT-0
"""Unitree Go2 Air — Nova Sonic voice + text agent (direct WebRTC transport).

Same UX as main.py, but talks straight to the robot over WiFi via
unitree_webrtc_connect — bypassing this SDK's ROS2 container entirely. Use this
when you don't have the container running and want the agent to drive the dog
directly (Unitree TURN server in remote mode, or AP/STA in local mode).

Because it connects to the robot directly, this entry point DOES need Unitree
credentials — set them via environment / .env (see config.py):
    UNITREE_CONN_TYPE=remote|local
    remote: UNITREE_EMAIL, UNITREE_SERIAL, and a password
            (UNITREE_PASSWORD, or UNITREE_SECRET_NAME + AWS_SECRETS_PROFILE)
    local:  UNITREE_LOCAL_MODE=ap|sta, UNITREE_ROBOT_IP, UNITREE_AES_128_KEY

Run from a real terminal, from the repo root:
    python -m voice_agent.main_sonic                  # voice + text
    python -m voice_agent.main_sonic --text-only      # skip mic
"""

import argparse
import asyncio
import sys

from . import config
from .client_registry import set_client
from .go2_webrtc_client import Go2Client
from .session import configure_logging, run_agent


async def run(text_only: bool, voice: str, log_level: str) -> None:
    configure_logging(
        log_level,
        app_loggers=("__main__", "voice_agent"),
        # unitree_webrtc_connect logs every sportmodestate frame on the root
        # logger at INFO (~20 Hz), which floods stdout — pin it to WARNING.
        quiet_loggers=(
            "unitree_webrtc_connect", "aiortc", "aioice", "botocore", "strands",
        ),
    )

    print("=" * 60)
    print("  Unitree Go2 Air — Nova Sonic agent (direct WebRTC)")
    print("=" * 60)
    print(f"  Connection:  {config.CONN_TYPE} mode")
    if config.CONN_TYPE == "remote":
        print(f"  Robot:       {config.ROBOT_SERIAL}")
    else:
        print(f"  Robot IP:    {config.ROBOT_IP} ({config.LOCAL_MODE})")
    print(f"  Model:       {config.NOVA_SONIC_MODEL_ID} ({config.NOVA_SONIC_REGION})")
    print(f"  Voice:       {voice}")
    print(f"  Mode:        {'text-only' if text_only else 'voice + text'}")
    print()

    print("[*] Connecting to Go2 via WebRTC...")
    client = Go2Client()
    if not client.connect():
        print("[!] Failed to connect to robot. Check your UNITREE_* settings.")
        sys.exit(1)
    set_client(client)  # tools' get_client() now returns the WebRTC client
    print("[✓] Robot connected!")
    print()

    await run_agent(client, text_only=text_only, voice=voice)


def main() -> None:
    parser = argparse.ArgumentParser(description="Go2 Nova Sonic agent (direct WebRTC)")
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
