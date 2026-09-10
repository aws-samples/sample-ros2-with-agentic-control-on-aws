# Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
# SPDX-License-Identifier: MIT-0
"""Strands Agent tools for controlling the Unitree Go2 Air.

Each function decorated with @tool becomes a callable tool that the LLM
can invoke based on natural language instructions. The tools resolve the
active robot client through `client_registry.get_client()`, so they stay
transport-agnostic — `main.py` installs the ROS2/Foxglove transport.

## Two things about this file that are safety controls, not style

**Ballistic motions need two tool calls.** Flips, jumps and pounces throw ~15 kg
of robot through the air and are the one thing here that can injure someone. The
system prompt asks the model to warn about clear space, but prose is not a
control: a single injected instruction — text on a sign the camera reads, see
notifications.quote_untrusted — bypasses prose entirely. So `perform_action`
REFUSES the actions in DANGEROUS_ACTIONS, and reaching them requires
`request_dangerous_action` and then `confirm_dangerous_action` naming the same
action inside a short window, with the user answering out loud in between. One
utterance cannot do it.

**Nothing here logs what was said.** Under AgentCore, stdout goes to CloudWatch,
and these tools' arguments are the user's transcribed speech (`question=`,
`target=`). Every call site logs metadata — lengths, names, flags — at debug, via
`logger`, never the content. Do not add a `print(f"... {question!r}")` back.
"""

import logging
import threading
import time

from strands import tool

from .client_registry import get_client

logger = logging.getLogger(__name__)

# Motions that leave the ground or throw the robot's mass around. Reaching any of
# these takes the two-step confirmation below.
DANGEROUS_ACTIONS = frozenset({
    "front_flip", "back_flip", "left_flip", "front_jump", "front_pounce",
})

# How long a request stays confirmable. Short on purpose: it has to survive one
# spoken exchange ("I've cleared 2 metres — go ahead") and no longer, so a
# request the user ignored cannot be redeemed later by something else.
DANGEROUS_CONFIRM_SECONDS = 60.0

_pending_lock = threading.Lock()
# {"action": str, "expires": float} for the single outstanding request, or None.
# Deliberately one slot: two live requests would let a confirmation for the safe
# one be redirected to the other.
_pending_dangerous: dict | None = None


def _arm_dangerous(action_name: str) -> None:
    global _pending_dangerous
    with _pending_lock:
        _pending_dangerous = {
            "action": action_name,
            "expires": time.monotonic() + DANGEROUS_CONFIRM_SECONDS,
        }


def _claim_dangerous(action_name: str) -> tuple[bool, str]:
    """Consume the outstanding request if it matches. Returns (ok, why_not).

    Single-use whatever the outcome: a mismatched or expired confirmation clears
    the slot too, so a rejected attempt cannot be retried against a stale request.
    """
    global _pending_dangerous
    with _pending_lock:
        pending, _pending_dangerous = _pending_dangerous, None

    if pending is None:
        return False, (
            f"No pending request for {action_name}. Call "
            "request_dangerous_action first, then ask the user out loud to "
            "confirm the space is clear."
        )
    if pending["action"] != action_name:
        return False, (
            f"The outstanding request was for {pending['action']}, not "
            f"{action_name}. Nothing was performed. Start again with "
            "request_dangerous_action if that is what the user wants."
        )
    if time.monotonic() > pending["expires"]:
        return False, (
            f"The request for {action_name} expired after "
            f"{int(DANGEROUS_CONFIRM_SECONDS)} seconds. Ask the user again, "
            "then request and confirm afresh."
        )
    return True, ""


@tool
def stand_up() -> dict:
    """Make the robot stand up from any position (lying down, sitting, crouching).

    This performs RecoveryStand followed by BalanceStand to prepare the robot
    for receiving movement and gesture commands.

    Use this when the user wants the robot to get up, stand, or recover.
    """
    logger.debug("tool stand_up")
    client = get_client()
    return client.stand_up()


@tool
def stand_down() -> dict:
    """Make the robot crouch down to a low position (safe to power off).

    Use this when the user wants the robot to lie down, crouch, rest,
    or prepare for power off.
    """
    logger.debug("tool stand_down")
    client = get_client()
    return client.stand_down()


@tool
def sit() -> dict:
    """Make the robot sit like a dog.

    Use this when the user wants the robot to sit.
    """
    logger.debug("tool sit")
    client = get_client()
    return client.sit()


@tool
def move(
    direction: str = "forward",
    speed: str = "medium",
    duration_seconds: float = 2.0,
) -> dict:
    """Move the robot in a specified direction at a given speed for a duration.

    Args:
        direction: One of "forward", "backward", "left", "right",
                   "turn_left", "turn_right".
        speed: One of "slow", "medium", "fast".
        duration_seconds: How long to move in seconds (max 10).

    Use this when the user wants the robot to walk, move, go somewhere,
    turn, or navigate in any direction.
    """
    logger.debug("tool move: direction=%s speed=%s duration=%s",
                 direction, speed, duration_seconds)

    speed_map = {"slow": 0.2, "medium": 0.35, "fast": 0.5}
    vel = speed_map.get(speed, 0.35)

    direction_map = {
        "forward": (vel, 0.0, 0.0),
        "backward": (-vel, 0.0, 0.0),
        "left": (0.0, vel, 0.0),
        "right": (0.0, -vel, 0.0),
        "turn_left": (0.0, 0.0, 0.6),
        "turn_right": (0.0, 0.0, -0.6),
    }

    # An unrecognised direction is an ERROR, never a default. This used to fall
    # back to forward motion, so a typo or a hallucinated direction drove a
    # physical robot forward into whatever was in front of it. Nothing that
    # actuates should default to moving.
    vector = direction_map.get(direction)
    if vector is None:
        return {
            "status": "error",
            "action": "move",
            "note": (
                f"I don't know the direction {direction!r}. Use one of: "
                f"{', '.join(sorted(direction_map))}. Nothing moved."
            ),
        }
    vx, vy, vyaw = vector

    # Clamped at BOTH ends. The upper bound was always here; without the lower
    # one a negative duration reached the client and silently no-opped, so the
    # agent reported a move that never happened.
    duration_seconds = max(0.0, min(duration_seconds, 10.0))

    client = get_client()
    return client.move(vx=vx, vy=vy, vyaw=vyaw, duration=duration_seconds)


@tool
def turn_degrees(degrees: float, speed: float = 0.0) -> dict:
    """Rotate the robot in place by a precise angle and report what it measured.

    Diagnostic counterpart to move's "turn_left"/"turn_right", which take a
    duration and cannot tell you how far the robot actually got. This one closes
    the loop against the robot's odometry, so the result says both what was asked
    for and what the robot measured itself doing. Rotation only — the robot never
    translates, so it is safe in tight spaces.

    Args:
        degrees: How far to rotate. POSITIVE = left / counter-clockwise,
                 negative = right / clockwise. Clamped to +/-360.
        speed: Angular speed in rad/s. Leave 0 for the default facing speed
               (0.6). Small values (0.2-0.3) turn gently; the robot's own safety
               clamp caps this at 0.8.

    Use this when the user names an angle — "turn 40 degrees", "rotate 90 to the
    left", "spin around", "turn a quarter turn right" — or when they are
    calibrating or testing the turning behaviour.

    Returns requested_degrees, measured_degrees, and closed_loop (False means
    odometry was unavailable and the turn ran on a timer, so it likely fell
    short). When the user is testing, SPEAK BOTH NUMBERS — the gap between them
    is the whole point of the tool.

    Also returns "ended": "target" means the robot measured itself reaching the
    angle and stopped. "deadline" means it did NOT — the turn ran out its time
    limit and the robot has turned much further than asked, roughly double. Say
    so plainly when you see it, and read out the note; it is a fault in the
    robot's odometry, not something the user did.
    """
    logger.debug("tool turn_degrees: degrees=%s speed=%s", degrees, speed)

    # Bounded like move's duration: a hallucinated 3600 would spin the robot ten
    # times over. One full revolution is the most any phrasing can legitimately
    # mean.
    degrees = max(-360.0, min(360.0, degrees))
    client = get_client()

    turn = getattr(client, "turn_degrees", None)
    if turn is None:
        return {
            "status": "error",
            "action": "turn_degrees",
            "note": (
                "This robot transport cannot measure its own rotation. Use move "
                "with turn_left or turn_right and a duration instead."
            ),
        }

    result = turn(degrees, speed if speed > 0 else None)
    result.setdefault("status", "success")
    result.setdefault("action", "turn_degrees")

    # The camera FOV estimate is what converts a pixel offset into a bearing
    # everywhere else (facing a visitor, centring on a found object), and it is
    # refined from turns like this one. Surfacing it makes this tool useful for
    # diagnosing "the robot turns too far when it faces me".
    hfov = getattr(client, "_hfov", None)
    if hfov is not None:
        result["camera_hfov_estimate"] = round(hfov, 1)
    return result


@tool
def stop_moving() -> dict:
    """Immediately stop all robot movement.

    Use this when the user wants the robot to stop, halt, freeze,
    or cease moving.
    """
    logger.debug("tool stop_moving")
    client = get_client()
    return client.stop()


@tool
def perform_action(action_name: str) -> dict:
    """Perform a special action or trick.

    Args:
        action_name: The name of the action. Available actions:
            - "hello" - Wave hello gesture
            - "stretch" - Stretch like waking up
            - "content" - Happy/content gesture
            - "dance1" - Dance routine 1
            - "dance2" - Dance routine 2
            - "heart" - Heart gesture with front paws
            - "scrape" - Scrape the ground
            - "sit" - Sit down
            - "rise_sit" - Stand from sitting

    Flips, jumps and pounces are NOT available here — they go through
    request_dangerous_action instead. This tool will refuse them.

    Use this when the user wants the robot to do a trick, dance, wave, or
    perform any special motion that keeps its feet on the ground.
    """
    logger.debug("tool perform_action: action=%s", action_name)

    # Refused, not warned about. This is the gate the system prompt's "flips need
    # 2 m of clear space" line cannot be: prose is advice to a model, and an
    # injected instruction skips advice.
    if action_name in DANGEROUS_ACTIONS:
        return {
            "status": "error",
            "action": "perform_action",
            "requires_confirmation": True,
            "note": (
                f"{action_name} throws the robot off the ground and cannot be "
                "done from this tool. Call request_dangerous_action("
                f"{action_name!r}), tell the user out loud what it needs, and "
                "only call confirm_dangerous_action once they have said the "
                "space is clear."
            ),
        }

    client = get_client()
    return client.perform_special_motion(action_name)


@tool
def request_dangerous_action(action_name: str) -> dict:
    """Step 1 of 2 for a flip, jump or pounce: ask the user to clear the space.

    These motions throw the robot into the air. This tool performs NO motion at
    all — it records that the user asked for one, and returns the warning you
    must speak. After calling it, say out loud what the robot is about to do and
    how much space it needs, then WAIT for the user to confirm. Only when they
    have actually said the space is clear may you call confirm_dangerous_action.

    Never call both tools in the same turn, and never assume consent — if the
    user does not confirm, do nothing and say you are not going to do it.

    Args:
        action_name: One of "front_flip", "back_flip", "left_flip",
                     "front_jump", "front_pounce".
    """
    logger.debug("tool request_dangerous_action: action=%s", action_name)

    if action_name not in DANGEROUS_ACTIONS:
        return {
            "status": "error",
            "action": "request_dangerous_action",
            "note": (
                f"{action_name} is not a ballistic motion — call perform_action "
                "for it directly. This tool is only for: "
                f"{', '.join(sorted(DANGEROUS_ACTIONS))}."
            ),
        }

    _arm_dangerous(action_name)
    return {
        "status": "success",
        "action": "request_dangerous_action",
        "pending": action_name,
        "expires_in_seconds": int(DANGEROUS_CONFIRM_SECONDS),
        "note": (
            f"Ready to {action_name.replace('_', ' ')}, but NOT doing it yet. "
            "Tell the user out loud: it needs at least 2 metres clear in every "
            "direction, a flat non-slip floor, and everyone's hands and feet "
            "well away. Ask them to confirm the space is clear, then wait for "
            "their answer. Call confirm_dangerous_action only after they say "
            f"yes, and within {int(DANGEROUS_CONFIRM_SECONDS)} seconds."
        ),
    }


@tool
def confirm_dangerous_action(action_name: str) -> dict:
    """Step 2 of 2: perform the flip/jump the user has just confirmed out loud.

    Only call this after request_dangerous_action for the SAME action and after
    the user has audibly confirmed the space is clear, in their own words, in a
    later turn. It fails if there is no matching outstanding request or if more
    than a minute has passed — in which case ask again rather than retrying.

    Args:
        action_name: The same action name passed to request_dangerous_action.
    """
    logger.debug("tool confirm_dangerous_action: action=%s", action_name)

    ok, why_not = _claim_dangerous(action_name)
    if not ok:
        return {"status": "error", "action": "confirm_dangerous_action",
                "note": why_not}

    logger.info("performing confirmed ballistic motion: %s", action_name)
    client = get_client()
    return client.perform_special_motion(action_name)


@tool
def describe_scene(question: str = "") -> dict:
    """Have the robot look through its camera and describe aloud what it sees.

    Use this whenever the user asks what the robot can see, what is in front of
    it, to look around, or to describe its surroundings. Pass the user's actual
    question (e.g. "what's on the table?") so the description can answer it; an
    empty question gives a general description.

    Returns a dict with a "description" field containing what the camera sees.
    Speak that description back to the user in your own voice.
    """
    logger.debug("tool describe_scene: question_len=%d", len(question))
    client = get_client()
    return client.describe_scene(question)


@tool
def recall_scene(question: str = "", seconds_ago: float = 30.0) -> dict:
    """Recall what the robot saw in the recent past ("robot memory").

    Use this when the user asks about the past — what something looked like or
    what was somewhere a moment ago (e.g. "what was on the table 30 seconds
    ago?", "what did you see a minute ago?", "has anything changed since
    earlier?"). The robot builds this memory from describe_scene observations
    during the session.

    Args:
        question: The user's actual question about the past scene (e.g. "what
                  was on the table?"). Empty gives a general recollection.
        seconds_ago: How far back to look, in seconds (e.g. 30, 120). Convert
                     phrases like "a minute ago" → 60, "two minutes ago" → 120.

    Returns a dict with a "description" field — what the camera saw at that time.
    Speak that back to the user in your own voice.
    """
    logger.debug("tool recall_scene: question_len=%d seconds_ago=%s",
                 len(question), seconds_ago)
    client = get_client()
    return client.recall_scene(question, seconds_ago)


@tool
def compare_scenes(question: str = "", seconds_ago: float = 30.0) -> dict:
    """Compare what the robot sees now against what it saw earlier ("what changed?").

    Use this when the user asks what has CHANGED or what is DIFFERENT compared to
    before (e.g. "what's different now?", "did anything move?", "what changed
    since earlier?", "is the cup still there?"). This sends both the past view
    and the current view to the vision model and asks what changed.

    Args:
        question: The user's actual question about the difference (e.g. "did the
                  laptop move?"). Empty asks for a general "what changed?".
        seconds_ago: How far back the "before" view should be, in seconds.
                     Convert "a minute ago" → 60, "two minutes ago" → 120.

    Returns a dict with a "description" field — what changed between then and now.
    Speak that back to the user in your own voice.
    """
    logger.debug("tool compare_scenes: question_len=%d seconds_ago=%s",
                 len(question), seconds_ago)
    client = get_client()
    return client.compare_scenes(question, seconds_ago)


@tool
def get_detections() -> dict:
    """Return the latest COCO object detections from the robot's camera.

    Requires the coco_detector_node to be running in the ROS2 container:
        ros2 run coco_detector coco_detector_node

    Use this when the user asks what objects are visible, what is in the scene,
    or wants a list of detected items with their positions.

    Returns a list of detected objects, each with class_id (e.g. "person",
    "dog", "chair"), confidence score, and pixel coordinates of the bounding
    box centre.
    """
    client = get_client()
    return client.get_detections()


@tool
def start_tracking(target: str = "person") -> dict:
    """Start real-time tracking: the robot rotates to keep a target centred.

    Subscribes to /detected_objects (coco_detector_node must be running) and
    starts a proportional-control loop that issues yaw corrections at ~10 Hz
    to keep the named object class centred in the camera frame.

    Args:
        target: COCO class name to follow. Common choices: "person", "cat",
                "dog", "bottle", "chair". Defaults to "person".

    Use this when the user says "follow me", "track the person", "keep that
    dog in view", or any command about continuously following an object.
    Say 'stop tracking' or call stop_tracking() to end.
    """
    client = get_client()
    return client.start_tracking(target)


@tool
def stop_tracking() -> dict:
    """Stop the active object-tracking loop and halt robot rotation.

    Use this when the user says "stop following", "stop tracking", "stay here",
    or any command to end continuous object-following behaviour.
    """
    client = get_client()
    return client.stop_tracking()


@tool
def greet_visitor(wait_seconds: float = 0.0) -> dict:
    """Greet a visitor once: turn to face them and wave hello.

    The robot finds the nearest person with its camera, rotates in place to
    centre them, then performs its wave gesture. It stays where it is — no
    walking, so this is safe in tight spaces.

    Args:
        wait_seconds: How long to wait for someone to appear (0 = greet whoever
                      is visible right now, and report back if nobody is).
                      Use ~10-20 when the user says "wait for someone".

    Use this when the user says "greet them", "say hello to them", "welcome our
    visitor", "there's someone here", or similar one-off greeting requests.

    Requires coco_detector_node running in the ROS2 container.

    Returns a dict with a "visitor" field — a short note on what the camera saw.
    Work that into a warm spoken greeting in your own voice, then ask who they
    are here to see.
    """
    logger.debug("tool greet_visitor: wait_seconds=%s", wait_seconds)
    client = get_client()
    return client.greet_visitor(wait_seconds)


@tool
def start_greeter() -> dict:
    """Turn on greeter mode: watch for visitors and greet each new arrival.

    The robot stays put watching its camera. Each time a new person appears it
    turns to face them and waves, then re-arms itself once they've moved away,
    so it won't wave repeatedly at the same person. Rotation only — it never
    walks anywhere.

    Use this when the user says "be a greeter", "greeter mode", "welcome
    visitors", "greet anyone who comes in", or "act as a receptionist".

    Requires coco_detector_node running in the ROS2 container. Greetings happen
    in the background — call get_greeter_status to find out who has been greeted
    and to pick up greetings you have not spoken yet. Say 'stop greeting' to end.
    """
    logger.debug("tool start_greeter")
    client = get_client()
    return client.start_greeter()


@tool
def stop_greeter() -> dict:
    """Turn off greeter mode and stop watching for visitors.

    Use this when the user says "stop greeting", "stop being a greeter",
    "exit greeter mode", or any command to end the welcoming behaviour.
    """
    logger.debug("tool stop_greeter")
    client = get_client()
    return client.stop_greeter()


@tool
def get_greeter_status() -> dict:
    """Get greeter mode status and any greetings not yet spoken aloud.

    Use this when the user asks whether greeter mode is on, how many visitors
    have been greeted, or who the robot has seen. Also call it when the user
    hints someone may have arrived (e.g. "did anyone come in?").

    Returns a dict with a "greeter" field containing state (watching/idle),
    greeted_count, last_greeting, and "new_greetings" — visitors greeted in the
    background that you have not acknowledged yet. If new_greetings is
    non-empty, welcome those visitors aloud now and ask who they are here to
    see. Reading this clears the new_greetings queue, so act on them right away.
    """
    logger.debug("tool get_greeter_status")
    client = get_client()
    return client.get_greeter_status()


@tool
def find_object(target: str) -> dict:
    """Search for a named object by turning in place and scanning with the camera.

    The robot rotates step by step, checking each direction for the object, and
    turns to point at it when found. It stays in one spot — no walking — so this
    is safe in tight spaces.

    Args:
        target: What to look for, in the user's own words (e.g. "my backpack",
                "phone", "chair"). Common synonyms are understood and mapped to
                what the detector knows.

    Use this when the user says "find my backpack", "where's my phone?",
    "look for a chair", "can you see my laptop anywhere?", or similar.

    The robot can only recognise the 80 COCO object classes. If the target isn't
    one of them the tool returns an error listing what IS findable — tell the
    user plainly that you can't look for that thing, and suggest a close
    alternative from the list rather than pretending to search.

    Searching runs in the background. Report that you've started looking, then
    call get_find_status to find out how it went.
    """
    logger.debug("tool find_object: target_len=%d", len(target or ""))
    client = get_client()
    return client.find_object(target)


@tool
def get_find_status() -> dict:
    """Check how the object search is going, or how it ended.

    Use this after find_object, and whenever the user asks "did you find it?",
    "any luck?", "where is it?", or similar.

    Returns a dict with a "find" field containing state (searching / found /
    not_found / stopped / idle), the target, how far it has swept in degrees,
    and when found: a "direction" in plain words (e.g. "to my left") plus
    whether the robot is facing it. Relay that naturally — if it found the
    object, say where it is; if it swept all the way round without success, say
    so honestly rather than guessing a location.
    """
    logger.debug("tool get_find_status")
    client = get_client()
    return client.get_find_status()


@tool
def stop_find() -> dict:
    """Stop the in-progress object search and halt the robot's rotation.

    Use this when the user says "stop looking", "never mind", "stop searching",
    or "forget it" during a search.
    """
    logger.debug("tool stop_find")
    client = get_client()
    return client.stop_find()


@tool
def get_robot_state() -> dict:
    """Get the current state of the robot (connection, height, speed level, etc.).

    Use this when the user asks about the robot's status, state,
    or whether it's connected and ready.
    """
    logger.debug("tool get_robot_state")
    client = get_client()
    return client.get_state()
