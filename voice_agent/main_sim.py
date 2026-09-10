# Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
# SPDX-License-Identifier: MIT-0
"""Unitree Go2 — Nova Sonic voice + text agent against a MuJoCo simulation.

The same agent, prompt and tools as `main.py`, pointed at a simulated dog from
the Strands Labs robots project (https://strandsagents.com/docs/labs/robots/)
instead of the real one. No robot, no container, no Unitree credentials, no
network — only AWS credentials for Nova Sonic (and Bedrock, for
"what do you see?").

    make voice-sim                        # voice + text, with a viewer window
    make voice-sim-text                   # text only
    make sim-demo                         # scripted, no agent and no AWS

    # or directly, from the repo root:
    .venv-sim/bin/mjpython -m voice_agent.main_sim              # viewer
    .venv-sim/bin/python   -m voice_agent.main_sim --no-viewer  # headless

macOS needs `mjpython` (shipped with the mujoco wheel) for the viewer window:
MuJoCo's passive viewer has to own the UI thread, and plain `python` cannot give
it one. Without it everything still runs — you just cannot watch, so use
`--demo --save-frames` or `describe_scene` to see what the dog sees.

See sim_dog.py for what the simulation models honestly (poses, contacts,
balance, the camera) and what it animates (the trot).
"""

import argparse
import asyncio
import sys
from pathlib import Path

from . import config

# Where --save-frames writes. A directory rather than the working directory, so a
# demo run does not scatter twenty PNGs across the repo root (gitignored).
FRAME_DIR = Path(__file__).resolve().parents[1] / "sim_frames"
from .client_registry import set_client
from .go2_sim_client import Go2SimClient
from .session import configure_logging, run_agent

# Told to the model up front rather than discovered through tool errors.
SIM_PROMPT_SUFFIX = """

IMPORTANT — you are driving a SIMULATED robot dog in a MuJoCo physics
simulation, not real hardware. Say so if the user seems to think otherwise.
The dog stands, sits, lies down, walks, turns, waves, stretches and dances, and
its camera sees a simulated lab room, so "what do you see?" works normally.
Two things differ from the real dog, and you must be honest about them rather
than pretending:
- Flips and pounces are not simulated; offer a dance, a wave or a stretch.
- There is no object detector, so you cannot list detected objects, follow
  anyone, search for an object, or run greeter mode. You CAN still look and
  describe what you see.
The room has furniture and walls, and walking into them stops the dog short —
when a move reports blocked, tell the user and turn before walking again.
"""


def _demo(client: Go2SimClient, save_frames: bool) -> None:
    """Exercise the transport with no agent, no mic and no AWS credentials.

    The fastest way to tell "is the simulation healthy?" apart from "is Nova
    Sonic misbehaving?", and what CI would run.
    """
    steps = [
        ("stand up", lambda: client.stand_up()),
        ("walk forward 3s", lambda: client.move(vx=0.35, duration=3.0)),
        ("turn left 2s", lambda: client.move(vyaw=0.6, duration=2.0)),
        ("sit", lambda: client.sit()),
        ("wave hello", lambda: client.perform_special_motion("hello")),
        ("stand up", lambda: client.stand_up()),
        ("dance", lambda: client.perform_special_motion("dance1")),
        ("try a backflip", lambda: client.perform_special_motion("back_flip")),
        ("crouch", lambda: client.stand_down()),
        ("state", lambda: client.get_state()),
    ]
    if save_frames:
        FRAME_DIR.mkdir(exist_ok=True)
    for label, action in steps:
        result = action()
        status = result.get("status")
        detail = {k: v for k, v in result.items()
                  if k not in ("status", "action", "note", "message")}
        print(f"  {label:22s} {status:8s} {detail}")
        if result.get("note"):
            print(f"  {'':22s}          note: {result['note'][:90]}")
        if save_frames:
            tag = label.replace(" ", "_")
            client._dog.grab_world_frame().save(FRAME_DIR / f"{tag}_room.png")
            client._dog.grab_frame().save(FRAME_DIR / f"{tag}_head.png")
    if save_frames:
        print(f"  frames written to {FRAME_DIR}/")


async def run(text_only: bool, voice: str, log_level: str, viewer: bool,
              scene: str) -> None:
    configure_logging(
        log_level,
        app_loggers=("__main__", "voice_agent"),
        quiet_loggers=("botocore", "strands", "aiohttp", "asyncio"),
    )

    print("=" * 60)
    print("  Unitree Go2 — Nova Sonic agent (MuJoCo simulation)")
    print("=" * 60)
    print(f"  Robot:       {config.SIM_ROBOT} (simulated — no hardware)")
    print(f"  Scene:       {scene}")
    print(f"  Model:       {config.NOVA_SONIC_MODEL_ID} ({config.NOVA_SONIC_REGION})")
    print(f"  Voice:       {voice}")
    print(f"  Mode:        {'text-only' if text_only else 'voice + text'}")
    print()

    print("[*] Building the simulation (first run downloads the Go2 model)...")
    client = Go2SimClient(scene=scene, viewer=viewer)
    client.connect()
    set_client(client)  # tools' get_client() now returns the sim client
    state = client.get_state()
    print(f"[✓] Simulation running at {state['sim']['realtime_factor']}x realtime"
          f"{' with a viewer window' if state['sim']['viewer_open'] else ' (headless)'}")
    print()

    await run_agent(client, text_only=text_only, voice=voice,
                    prompt_suffix=SIM_PROMPT_SUFFIX)


def main() -> None:
    parser = argparse.ArgumentParser(description="Go2 Nova Sonic agent (simulation)")
    parser.add_argument("--text-only", action="store_true", help="Disable mic")
    parser.add_argument("--voice", default="en-us.matthew", help="Nova Sonic voice ID")
    parser.add_argument("--log-level", default="INFO", help="Logging level")
    parser.add_argument("--scene", default=config.SIM_SCENE,
                        choices=("lab", "empty"), help="World to drop the dog into")
    parser.add_argument("--no-viewer", dest="viewer", action="store_false",
                        help="Do not open the interactive MuJoCo window")
    parser.add_argument("--demo", action="store_true",
                        help="Run a scripted movement sequence and exit — no agent, "
                             "no microphone, no AWS credentials needed")
    parser.add_argument("--save-frames", action="store_true",
                        help="With --demo, write a head and room view per step")
    parser.set_defaults(viewer=True)
    args = parser.parse_args()

    if args.demo:
        print("[*] Building the simulation...")
        client = Go2SimClient(scene=args.scene, viewer=args.viewer)
        client.connect()
        print(f"[✓] {client.get_state()['sim']}")
        try:
            _demo(client, args.save_frames)
        finally:
            client.disconnect()
        print("[*] Done.")
        return

    try:
        asyncio.run(run(args.text_only, args.voice, args.log_level, args.viewer,
                        args.scene))
    except KeyboardInterrupt:
        print("\n[*] Bye!")
    except Exception as e:  # noqa: BLE001
        print(f"[!] {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
