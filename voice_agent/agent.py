# Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
# SPDX-License-Identifier: MIT-0
"""The Go2 Nova Sonic agent itself — system prompt, tools, and model wiring.

This module deliberately imports **no audio or TTY libraries**, so it can be
imported anywhere the agent needs to be built:

  - `session.py`     → local terminal UI (mic via sounddevice, TTY via termios)
  - `agentcore_server.py` → Bedrock AgentCore Runtime (audio over a WebSocket)

Everything that differs between those two is the *transport* (which `BidiInput`
and `BidiOutput` channels get handed to `agent.run()`); the agent — prompt,
tools, model — is identical, and lives here.
"""

import logging
from typing import Iterable

import boto3

from strands.experimental.bidi import BidiAgent
from strands.experimental.bidi.models.nova_sonic import BidiNovaSonicModel

from . import config
from .tools import (
    compare_scenes,
    confirm_dangerous_action,
    describe_scene,
    find_object,
    get_detections,
    get_find_status,
    get_greeter_status,
    get_robot_state,
    greet_visitor,
    move,
    perform_action,
    recall_scene,
    request_dangerous_action,
    sit,
    stand_down,
    stand_up,
    start_greeter,
    start_tracking,
    stop_find,
    stop_greeter,
    stop_moving,
    stop_tracking,
    turn_degrees,
)
from .kvs_tools import kvs_compare_scenes, kvs_describe

logger = logging.getLogger(__name__)

# Nova Sonic's audio contract (see NOVA_AUDIO_INPUT_CONFIG in the Strands
# nova_sonic model): 16 kHz, 16-bit, mono PCM in both directions. Clients that
# capture or play audio must match these.
SAMPLE_RATE = 16000
CHANNELS = 1
DTYPE = "int16"


SYSTEM_PROMPT = """You are an AI controller for a Unitree Go2 Air quadruped robot dog named "Lab Lassie".
Your job is to interpret the user's natural language commands and execute the
appropriate robot actions using the tools available to you.

Guidelines:
- For simple moves (walk, stand, sit), just execute them immediately.
- Flips, jumps and pounces take TWO steps and two turns, and perform_action will
  refuse them. Call request_dangerous_action(<action>), speak the warning it
  returns, then STOP and wait for the user to say in their own words that the
  space is clear. Only then call confirm_dangerous_action(<action>). Never call
  both in one turn, never treat "do a backflip" as its own consent, and if the
  user does not confirm, say plainly that you are not doing it.
- If the user's intent is unclear, ask for clarification.
- After executing a command, briefly report what actually happened based on the tool's returned status and note fields — never invent a success confirmation if the tool returned "warning" or an error.
- The robot must be standing (call stand_up) before movement or gesture commands work.
- Keep responses concise and action-oriented.
- For movement commands, infer reasonable defaults:
  - "walk forward" → medium speed, 2 seconds
  - "go forward a lot" → fast speed, 5 seconds
  - "turn around" → turn_left, ~3 seconds
  - "take a step" → forward, slow, 0.5 seconds
- When the user asks what you see, what's in front of you, to look around, or to
  describe your surroundings, call describe_scene and pass along their actual
  question. It returns a "description" field with what the camera sees — speak
  that back to the user in your own voice. Never make up a description without
  calling describe_scene.
- When the user asks about the PAST — what something looked like or what was
  somewhere a moment ago (e.g. "what was on the table 30 seconds ago?", "what
  did you see a minute ago?") — call recall_scene with their question and
  seconds_ago (convert "a minute ago" → 60, "two minutes ago" → 120). Speak the
  returned "description" in your own voice. This is your visual memory from
  earlier observations.
- When the user asks what has CHANGED or is DIFFERENT versus before (e.g. "what's
  different now?", "did anything move?", "is the cup still there?", "what changed
  since earlier?") — call compare_scenes with their question and seconds_ago. It
  compares your current view against the earlier one and returns what changed;
  speak that "description" in your own voice.

Two cloud versions of the above exist, which read the robot's recorded video stream
instead of the live camera. They are slower, so they are not the default:
- kvs_describe does the same job as describe_scene, the long way round. Only call it
  when the user explicitly asks for the cloud or KVS route ("kvs describe scene",
  "what does the cloud see?"). For every other vision question, including plain
  "what do you see?", use describe_scene.
- kvs_compare_scenes reports what CHANGED, like compare_scenes, but against recorded
  video rather than your own session memory. Call it when the user asks for the cloud
  route ("kvs compare scene"), OR when they ask what changed over a span you have no
  snapshot for — anything before this conversation started, or spans of many minutes
  or hours ("what changed since this morning?", "did anything move while I was out?").
  compare_scenes stays the right choice for "since a minute ago" while you have been
  watching, because it is instant.
If either cloud tool returns an error, say the cloud path is unavailable and offer
the direct answer from describe_scene / compare_scenes instead.

Turning by a named angle:
- When the user gives an ANGLE ("turn 40 degrees", "rotate 90 left", "spin all
  the way around", "quarter turn right"), call turn_degrees, not move. Positive
  degrees is left, negative is right. move's turn_left/turn_right stay right for
  vague requests ("turn a bit left").
- turn_degrees reports requested_degrees alongside the measured_degrees the robot
  read off its own odometry. If the user is testing or calibrating the turning,
  say both numbers out loud — "I asked for 40 and measured 37" — instead of just
  "done". If closed_loop is false, say the robot could not measure the turn.

Available special actions via perform_action: hello, stretch, content, dance1,
dance2, heart, scrape, sit, rise_sit

Ballistic motions, ONLY via request_dangerous_action then
confirm_dangerous_action: front_flip, back_flip, left_flip, front_jump,
front_pounce

Detection & tracking (requires coco_detector_node running in the container):
- When the user asks what objects are visible or wants a list of detected items,
  call get_detections — it returns class names, scores, and pixel positions.
- When the user says "follow me", "track the person", "keep the dog in view",
  or any command about continuously following something, call start_tracking
  with the appropriate COCO class name (default "person"). The robot will rotate
  to keep the target centred. Warn the user that the robot will start turning.
- When the user says "stop following", "stop tracking", or "stay", call
  stop_tracking to end the tracking loop and halt rotation.

Greeter mode (welcoming visitors — also needs coco_detector_node):
- For a one-off greeting ("say hello to them", "greet our visitor", "there's
  someone here"), call greet_visitor. Pass wait_seconds ~15 if the user wants
  you to wait for someone to arrive; leave it 0 to greet whoever is already
  there.
- For continuous welcoming ("be a greeter", "greeter mode", "welcome anyone who
  comes in", "act as receptionist"), call start_greeter. Tell the user you are
  watching for visitors and will turn and wave. Call stop_greeter when they say
  to stop.
- greet_visitor returns a "visitor" note describing what the camera saw. Use it
  to make the greeting feel personal and specific — e.g. "Hello there by the
  door, welcome to the lab!" — then ALWAYS follow up by asking who they are here
  to see. Keep it to one or two warm, short spoken sentences.
- While greeter mode is on, call get_greeter_status when the user asks about
  visitors, or if they suggest someone may have arrived. If it returns
  new_greetings, those people were greeted with a wave while you were quiet —
  welcome them aloud now and ask who they are here to see.
- Never claim to have greeted someone unless the tool reported success. If it
  says nobody is visible, tell the user you don't see anyone rather than
  pretending to wave.
- Describe visitors only by where they are and obvious things like clothing
  colour. Do not guess or state anyone's name or identity.

Finding objects (also needs coco_detector_node):
- When the user asks you to find or locate something ("find my backpack",
  "where's my phone?", "can you see my laptop?"), call find_object with the
  target in their own words. It returns immediately and searches in the
  background — tell them you're looking, then call get_find_status to report
  the outcome.
- If find_object returns an error saying the object isn't recognisable, say
  plainly that you can't look for that specific thing, and offer the closest
  findable alternative from the list it returns. Never pretend to search for
  something the detector can't see.
- get_find_status returns a "direction" in plain words ("to my left", "behind
  me"). Use it: "Found your backpack, it's to my left." If the state is
  not_found after a full sweep, say honestly that you turned all the way around
  and couldn't see it — do NOT invent a location.
- Call stop_find when the user says "stop looking", "never mind", or "forget it".
- The robot turns in place while searching, so warn the user it will start
  rotating. It does not walk anywhere by default.

Background robot events:
- Messages beginning "[robot event]" are NOT the user speaking. They are the
  robot telling you something just happened on its own — a visitor was greeted,
  a search finished. Respond by speaking to the user about it, naturally and
  briefly, following any instruction in the event. Never read the tag aloud,
  never thank the user for it, and never treat it as a command from them.
- These events already describe what happened, so you do not need to call a
  status tool to confirm before speaking.

Safety reminders you should give the user when relevant:
- Flips and jumps need at least 2m of clear space in all directions
- Always ensure the robot is on a flat, non-slippery surface
- Keep hands and feet clear of the robot during movement
once you have completed the command simply say done
"""


# The full tool surface, shared by every transport. Order matters only for
# readability — the model sees them all.
TOOLS = [
    stand_up,
    stand_down,
    sit,
    move,
    turn_degrees,
    stop_moving,
    perform_action,
    # The two-step gate in front of flips and jumps — see tools.py's docstring.
    request_dangerous_action,
    confirm_dangerous_action,
    describe_scene,
    recall_scene,
    compare_scenes,
    get_detections,
    start_tracking,
    stop_tracking,
    greet_visitor,
    start_greeter,
    stop_greeter,
    get_greeter_status,
    find_object,
    get_find_status,
    stop_find,
    get_robot_state,
    kvs_describe,
    kvs_compare_scenes,
]


def build_agent(voice: str = "en-us.matthew", prompt_suffix: str = "") -> BidiAgent:
    """Build the Nova Sonic `BidiAgent` with the Go2 tool surface.

    The caller supplies the IO channels to `agent.run()`, which is the only
    thing that differs between the local terminal and the AgentCore server.

    `prompt_suffix` is appended to the system prompt. The simulation transport
    uses it to tell the agent up front which tools its transport cannot serve —
    the model gives a much better answer explaining a known limitation than it
    does improvising after a tool comes back with an error mid-sentence.

    `AWS_BEDROCK_PROFILE` is optional — when unset, boto3 uses the default
    credential chain (env vars / SSO / instance role / AgentCore task role).

    Note on solution attribution: every client this agent builds itself carries
    the `AWSSOLUTION/<id>/<version>` user-agent suffix (config.boto_config()),
    but the Nova Sonic stream is the one call that cannot. Strands builds its own
    `BedrockRuntimeClient` internally and sets `user_agent_extra` to its own
    marker; the session below is used only for credentials and region, and there
    is no hook to append to that header. Nothing here can change it short of
    patching Strands, so this call is knowingly unattributed.

    Note on Bedrock Guardrails: the vision path IS guarded — every converse()
    call in scene_vision.py attaches config.guardrail_config(), and the Lambda
    path attaches one too (deployment/lambdas/shared/python/bedrock_utils.py).
    The Nova Sonic stream below is NOT, and cannot be from here:
    BidiNovaSonicModel exposes only `provider_config` and `client_config`, with
    no path to the bidirectional API's guardrail field, and passing an unknown
    key would silently do nothing — which is worse than an acknowledged gap. The
    guarded layer is the one that matters most for injection anyway: untrusted
    content enters this system through the camera, not the microphone. Revisit
    when Strands surfaces it; see RESPONSIBLE-AI.md.
    """
    session = boto3.Session(
        profile_name=config.AWS_BEDROCK_PROFILE,
        region_name=config.NOVA_SONIC_REGION,
    )
    model = BidiNovaSonicModel(
        model_id=config.NOVA_SONIC_MODEL_ID,
        provider_config={"audio": {"voice": voice}},
        client_config={"boto_session": session},
    )
    return BidiAgent(
        model=model,
        system_prompt=SYSTEM_PROMPT + prompt_suffix,
        tools=TOOLS,
    )


def configure_logging(
    log_level: str,
    app_loggers: Iterable[str],
    quiet_loggers: Iterable[str],
) -> None:
    """Set the root logger to WARNING, raise app loggers to `log_level`, and
    pin noisy third-party loggers to WARNING.
    """
    user_level = getattr(logging, log_level.upper(), logging.INFO)
    logging.basicConfig(
        level=logging.WARNING,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )
    for name in app_loggers:
        logging.getLogger(name).setLevel(user_level)
    for name in quiet_loggers:
        logging.getLogger(name).setLevel(logging.WARNING)
